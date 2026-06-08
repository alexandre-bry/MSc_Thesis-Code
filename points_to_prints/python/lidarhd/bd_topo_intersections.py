import logging
from pathlib import Path

from ..utils.duckdb_helpers import DuckDBConnectionManager, DuckDBConnector
from ..utils.utils import Box2154

SCHEMA_NAME = "intersections"

MULTIPOLY_TABLE_NAME = "multipoly"
POLY_TABLE_NAME = "poly"
RINGS_TABLE_NAME = "rings"
EDGES_TABLE_NAME = "edges"
INTERSECTIONS_TABLE_NAME = "intersections"
BUILDING_GROUPS_TABLE_NAME = "building_groups"
GRAPH_EDGES_TABLE_NAME = "graph_edges"
GRAPH_NODE_TO_ROOT_TABLE_NAME = "graph_node_to_root"
GRAPH_COMPONENTS_TABLE_NAME = "graph_components"

BUILDING_ID_COLUMN_NAME = "cleabs"
INITIAL_GEOMETRY_COLUMN_NAME = "geometry"
GEOMETRY_COLUMN_NAME = "geometry"
EXTENT_COLUMN_NAME = "extent"
START_POINT_COLUMN_NAME = "start_point"
END_POINT_COLUMN_NAME = "end_point"

IDX_POLYGON = {
    "name": "idx_polygon",
    "type": "UINT8",
}
IDX_RING = {
    "name": "idx_ring",
    "type": "UINT8",
}
IDX_EDGE = {
    "name": "idx_edge",
    "type": "UINT16",
}
EDGE_KEY = {
    "name": "edge_key",
    "type": "UINT32",
}

GEOMETRY_SIMPLIFICATION_TOLERANCE = 1e-6


def load_bd_topo_to_duckdb(con: DuckDBConnector, bd_topo_file: Path):
    logging.info(f"Loading BD TOPO file '{bd_topo_file}' into DuckDB...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{MULTIPOLY_TABLE_NAME} AS
        SELECT 
            * EXCLUDE ({INITIAL_GEOMETRY_COLUMN_NAME}),
            ST_SimplifyPreserveTopology({INITIAL_GEOMETRY_COLUMN_NAME}, {GEOMETRY_SIMPLIFICATION_TOLERANCE}) AS {GEOMETRY_COLUMN_NAME}
            -- {INITIAL_GEOMETRY_COLUMN_NAME} AS {GEOMETRY_COLUMN_NAME}
        FROM read_parquet($bd_topo_file);
        """,
        {"bd_topo_file": str(bd_topo_file)},
    )
    logging.info(f"Done loading BD TOPO file.")

    # Get the number of rows in the multipoly table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{MULTIPOLY_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of buildings: {num_rows:_}")


def unnest_multipoly_to_poly(con: DuckDBConnector):
    logging.info("Unnesting MultiPolygons into Polygons...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{POLY_TABLE_NAME} AS
        SELECT
            cleabs,
            COALESCE(path[1] - 1, 0)::{IDX_POLYGON['type']} AS {IDX_POLYGON['name']},
            geom AS {GEOMETRY_COLUMN_NAME}
        FROM (
            SELECT cleabs, UNNEST(ST_Dump({GEOMETRY_COLUMN_NAME}), recursive := true)
            FROM {SCHEMA_NAME}.{MULTIPOLY_TABLE_NAME}
        );
        """
    )

    # Get the number of rows in the poly table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{POLY_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of polygons: {num_rows:_}")


def unnest_poly_to_rings(con: DuckDBConnector):
    logging.info("Unnesting polygons into rings...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{RINGS_TABLE_NAME} AS
        SELECT 
            cleabs,
            {IDX_POLYGON['name']},
            {IDX_RING['name']}::{IDX_RING['type']} AS {IDX_RING['name']},
            CASE
                WHEN i.{IDX_RING['name']} = 0 THEN ST_ExteriorRing({GEOMETRY_COLUMN_NAME})
                ELSE ST_InteriorRingN({GEOMETRY_COLUMN_NAME}, i.{IDX_RING['name']})
            END AS {GEOMETRY_COLUMN_NAME}
        FROM {SCHEMA_NAME}.{POLY_TABLE_NAME}
        CROSS JOIN generate_series(0, ST_NInteriorRings({GEOMETRY_COLUMN_NAME})) AS i({IDX_RING['name']});
        """
    )

    # Get the number of rows in the rings table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{RINGS_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of rings: {num_rows:_}")


def unnest_rings_to_edges(con: DuckDBConnector):
    logging.info("Unnesting rings into edges...")
    # Unnest the LinearRingZ into LineStringZ (edges):
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{EDGES_TABLE_NAME} AS
        SELECT
            cleabs,
            {IDX_POLYGON['name']},
            {IDX_RING['name']},
            (idx_point-1)::{IDX_EDGE['type']} AS {IDX_EDGE['name']},
            ST_PointN({GEOMETRY_COLUMN_NAME}, idx_point::INTEGER) AS {START_POINT_COLUMN_NAME},
            ST_PointN({GEOMETRY_COLUMN_NAME}, (idx_point + 1)::INTEGER) AS {END_POINT_COLUMN_NAME},
            ST_MakeLine(
                ST_PointN({GEOMETRY_COLUMN_NAME}, idx_point::INTEGER),
                ST_PointN({GEOMETRY_COLUMN_NAME}, (idx_point + 1)::INTEGER)
            ) AS {GEOMETRY_COLUMN_NAME}
        FROM {SCHEMA_NAME}.{RINGS_TABLE_NAME}
        CROSS JOIN generate_series(1, ST_NPoints({GEOMETRY_COLUMN_NAME}) - 1) AS i(idx_point)
        ORDER BY cleabs, {IDX_POLYGON['name']}, {IDX_RING['name']}, {IDX_EDGE['name']};
        """
    )

    # Create an incremental key on the edges table
    con.execute(
        f"""
        CREATE OR REPLACE SEQUENCE seq_edges_ids START 1;
        ALTER TABLE {SCHEMA_NAME}.{EDGES_TABLE_NAME} ADD COLUMN {EDGE_KEY['name']} {EDGE_KEY['type']};
        UPDATE {SCHEMA_NAME}.{EDGES_TABLE_NAME} SET {EDGE_KEY['name']} = nextval('seq_edges_ids');
        """
    )

    # # Create a r-tree index on the geometry column of the edges table
    # con.execute(
    #     f"""
    #     CREATE INDEX idx_edges_geometry ON {SCHEMA_NAME}.{EDGES_TABLE_NAME} USING RTREE({GEOMETRY_COLUMN_NAME});
    #     """
    # )

    # Get the number of rows in the edges table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{EDGES_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of edges: {num_rows:_}")


def compute_intersections(con: DuckDBConnector):
    logging.info("Computing intersections between edges...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME}_temp AS
        SELECT
            a.{EDGE_KEY['name']} AS {EDGE_KEY['name']}_a,
            b.{EDGE_KEY['name']} AS {EDGE_KEY['name']}_b,
            ST_Intersection(a.{GEOMETRY_COLUMN_NAME}, b.{GEOMETRY_COLUMN_NAME}) AS {GEOMETRY_COLUMN_NAME}
        FROM {SCHEMA_NAME}.{EDGES_TABLE_NAME} a
        JOIN {SCHEMA_NAME}.{EDGES_TABLE_NAME} b
            ON a.{EDGE_KEY['name']} < b.{EDGE_KEY['name']}
        AND ST_Intersects(a.{GEOMETRY_COLUMN_NAME}, b.{GEOMETRY_COLUMN_NAME});
        """
    )
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME} AS
        SELECT *
        FROM {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME}_temp
        WHERE ST_GeometryType({GEOMETRY_COLUMN_NAME}) = 'LINESTRING';
        """
    )

    # Get the number of rows in the intersections table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of intersections: {num_rows:_}")


def export_edges(con: DuckDBConnector, output_edges_file: Path):
    logging.info(f"Exporting edges to '{output_edges_file}'...")
    con.export_parquet(
        schema_name=SCHEMA_NAME,
        table_name=EDGES_TABLE_NAME,
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_edges_file,
    )
    logging.info(f"Exported.")


def export_intersections(con: DuckDBConnector, output_intersections_file: Path):
    logging.info(f"Exporting intersections to '{output_intersections_file}'...")
    con.export_parquet(
        schema_name=SCHEMA_NAME,
        table_name=INTERSECTIONS_TABLE_NAME,
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_intersections_file,
    )
    logging.info(f"Exported.")


def group_buildings(con: DuckDBConnector):
    logging.info("Grouping buildings...")

    query = f"""
        -- Create a flat edge table (undirected, avoid duplicates if needed)
        CREATE TABLE {SCHEMA_NAME}.{GRAPH_EDGES_TABLE_NAME} AS
        SELECT
            cleabs_a AS src,
            cleabs_b AS dst
        FROM (
            SELECT ea.cleabs AS cleabs_a, eb.cleabs AS cleabs_b
            FROM {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME} i
            JOIN {SCHEMA_NAME}.{EDGES_TABLE_NAME} AS ea ON i.{EDGE_KEY['name']}_a = ea.{EDGE_KEY['name']}
            JOIN {SCHEMA_NAME}.{EDGES_TABLE_NAME} AS eb ON i.{EDGE_KEY['name']}_b = eb.{EDGE_KEY['name']}
            GROUP BY (cleabs_a, cleabs_b)
            HAVING cleabs_a < cleabs_b
        ) t;

        -- Compute the connected components with a recursive CTE
        CREATE TABLE {SCHEMA_NAME}.{GRAPH_NODE_TO_ROOT_TABLE_NAME} AS
        WITH RECURSIVE reach(root, node) AS (
            SELECT src AS root, src AS node
            FROM {SCHEMA_NAME}.{GRAPH_EDGES_TABLE_NAME}
            UNION
            SELECT r.root, e.dst
            FROM reach r
            JOIN {SCHEMA_NAME}.{GRAPH_EDGES_TABLE_NAME} e ON e.src = r.node
        )
        SELECT node, MIN(root) AS root_id
        FROM reach
        GROUP BY node;

        -- Add the isolated nodes (buildings with no intersection)
        INSERT INTO {SCHEMA_NAME}.{GRAPH_NODE_TO_ROOT_TABLE_NAME} (node, root_id)
        SELECT cleabs, cleabs
        FROM {SCHEMA_NAME}.{MULTIPOLY_TABLE_NAME}
        WHERE cleabs NOT IN (SELECT node FROM {SCHEMA_NAME}.{GRAPH_NODE_TO_ROOT_TABLE_NAME});

        -- Group the nodes by their root to get the connected components
        -- The group id is a new incremental index
        CREATE TABLE {SCHEMA_NAME}.{GRAPH_COMPONENTS_TABLE_NAME} AS
        SELECT
            row_number() OVER (ORDER BY root_id) AS group_id,
            list(node ORDER BY node) AS cleabs_list
        FROM {SCHEMA_NAME}.{GRAPH_NODE_TO_ROOT_TABLE_NAME}
        GROUP BY root_id
        ORDER BY root_id;

        -- Sum the number of buildings in each group
        SELECT group_id, array_length(cleabs_list) AS num_buildings
        FROM {SCHEMA_NAME}.{GRAPH_COMPONENTS_TABLE_NAME}
        ORDER BY num_buildings DESC;

        -- Compute the extent of each group
        CREATE TABLE {SCHEMA_NAME}.{BUILDING_GROUPS_TABLE_NAME} AS
        SELECT
            c.group_id,
            c.cleabs_list,
            ST_Extent_Agg(m.geometry) AS {EXTENT_COLUMN_NAME}
        FROM (
            SELECT group_id, cleabs_list, UNNEST(cleabs_list) AS cleabs
            FROM {SCHEMA_NAME}.{GRAPH_COMPONENTS_TABLE_NAME}
        ) c JOIN {SCHEMA_NAME}.{MULTIPOLY_TABLE_NAME} m
        ON m.cleabs = c.cleabs
        GROUP BY (c.group_id, c.cleabs_list)
        ORDER BY c.group_id;
    """
    con.execute(query)
    logging.info("Done grouping buildings.")

    # Get the number of building groups
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{BUILDING_GROUPS_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of building groups: {num_rows:_}")


def export_building_groups(con: DuckDBConnector, output_file: Path):
    logging.info(f"Exporting building groups to '{output_file}'...")
    con.export_parquet(
        schema_name=SCHEMA_NAME,
        table_name=BUILDING_GROUPS_TABLE_NAME,
        geom_col_name=EXTENT_COLUMN_NAME,
        output_file=output_file,
    )
    logging.info(f"Exported.")


def compute_export_intersections(
    bd_topo_file: Path,
    output_edges_file: Path,
    output_intersections_file: Path,
    output_building_groups_file: Path,
    overwrite: bool,
):
    for file in [
        output_edges_file,
        output_intersections_file,
        output_building_groups_file,
    ]:
        if file.exists():
            if overwrite:
                logging.warning(f"Output file '{file}' already exists. Overwriting.")
            else:
                logging.error(
                    f"Output file '{file}' already exists. Use --overwrite to overwrite it."
                )
                return

    output_edges_file.parent.mkdir(parents=True, exist_ok=True)
    output_intersections_file.parent.mkdir(parents=True, exist_ok=True)
    output_building_groups_file.parent.mkdir(parents=True, exist_ok=True)

    db_path = output_edges_file.parent / (output_edges_file.stem + ".duckdb")
    with DuckDBConnectionManager(db_path) as con:
        con.create_schema(SCHEMA_NAME)
        load_bd_topo_to_duckdb(con, bd_topo_file)
        unnest_multipoly_to_poly(con)
        unnest_poly_to_rings(con)
        unnest_rings_to_edges(con)
        compute_intersections(con)
        group_buildings(con)
        export_edges(con, output_edges_file)
        export_intersections(con, output_intersections_file)
        export_building_groups(con, output_building_groups_file)


def crop_intersections_files(
    input_edges_file: Path,
    input_intersections_file: Path,
    input_building_groups_file: Path,
    output_edges_file: Path,
    output_intersections_file: Path,
    output_building_groups_file: Path,
    bounding_box: Box2154,
    overwrite: bool,
    skip_existing: bool,
):
    output_files = [
        output_edges_file,
        output_intersections_file,
        output_building_groups_file,
    ]
    existing_output_files = [file for file in output_files if file.exists()]
    if not overwrite and len(existing_output_files) > 0:
        if skip_existing and len(existing_output_files) == len(output_files):
            logging.info("Skipping existing output files.")
            return
        else:
            raise FileExistsError(
                f"Some output files already exist. Use --overwrite to overwrite them."
            )

    output_edges_file.parent.mkdir(parents=True, exist_ok=True)
    output_intersections_file.parent.mkdir(parents=True, exist_ok=True)
    output_building_groups_file.parent.mkdir(parents=True, exist_ok=True)

    # Create a temporary DuckDB database to store the cropped data
    db_path = output_edges_file.parent / (output_edges_file.stem + ".duckdb")
    with DuckDBConnectionManager(db_path) as con:
        con.create_schema(SCHEMA_NAME)

        CROPPED_EDGES_TABLE_NAME = f"cropped_{EDGES_TABLE_NAME}"
        CROPPED_INTERSECTIONS_TABLE_NAME = f"cropped_{INTERSECTIONS_TABLE_NAME}"
        CROPPED_BUILDING_GROUPS_TABLE_NAME = f"cropped_{BUILDING_GROUPS_TABLE_NAME}"

        # Load the edges, intersections, and building groups files into DuckDB
        logging.info("Loading input files into DuckDB...")
        input_files = {
            input_edges_file: EDGES_TABLE_NAME,
            input_intersections_file: INTERSECTIONS_TABLE_NAME,
            input_building_groups_file: BUILDING_GROUPS_TABLE_NAME,
        }
        for file, table_name in input_files.items():
            con.execute(
                f"""
                CREATE OR REPLACE TABLE {SCHEMA_NAME}.{table_name} AS
                SELECT * FROM read_parquet($file);
                """,
                {"file": str(file)},
            )

        # Extract the groups that fit completely in the bounding box
        logging.info(
            "Extracting building groups that fit completely in the bounding box..."
        )
        query = f"""
            CREATE OR REPLACE TABLE {SCHEMA_NAME}.{CROPPED_BUILDING_GROUPS_TABLE_NAME} AS
            SELECT *
            FROM {SCHEMA_NAME}.{BUILDING_GROUPS_TABLE_NAME}
            WHERE ST_Covers(
                ST_MakeEnvelope(
                    {bounding_box.p_min.x},
                    {bounding_box.p_min.y},
                    {bounding_box.p_max.x},
                    {bounding_box.p_max.y}
                ),
                {EXTENT_COLUMN_NAME}
            )
        """
        con.execute(query)

        # Extract the edges that are part of the extracted groups
        logging.info(
            "Extracting edges that are part of the extracted building groups..."
        )
        query = f"""
            CREATE OR REPLACE TABLE {SCHEMA_NAME}.{CROPPED_EDGES_TABLE_NAME} AS
            SELECT
                e.* EXCLUDE(
                    e.{START_POINT_COLUMN_NAME},
                    e.{END_POINT_COLUMN_NAME}
                ),
                ST_X(e.{START_POINT_COLUMN_NAME}) AS start_x,
                ST_Y(e.{START_POINT_COLUMN_NAME}) AS start_y,
                ST_Z(e.{START_POINT_COLUMN_NAME}) AS start_z,
                ST_X(e.{END_POINT_COLUMN_NAME}) AS end_x,
                ST_Y(e.{END_POINT_COLUMN_NAME}) AS end_y,
                ST_Z(e.{END_POINT_COLUMN_NAME}) AS end_z
            FROM intersections.edges e
            JOIN (
                SELECT UNNEST(cleabs_list) AS cleabs
                FROM intersections.cropped_building_groups
            ) g
            ON e.cleabs = g.cleabs;
        """
        con.execute(query)

        # Extract the intersections that are part of the extracted edges
        logging.info("Extracting intersections that are part of the extracted edges...")
        query = f"""
            CREATE OR REPLACE TABLE {SCHEMA_NAME}.{CROPPED_INTERSECTIONS_TABLE_NAME} AS
            SELECT i.*
            FROM {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME} i
            JOIN {SCHEMA_NAME}.{CROPPED_EDGES_TABLE_NAME} e_a ON i.{EDGE_KEY['name']}_a = e_a.{EDGE_KEY['name']}
            JOIN {SCHEMA_NAME}.{CROPPED_EDGES_TABLE_NAME} e_b ON i.{EDGE_KEY['name']}_b = e_b.{EDGE_KEY['name']}
        """
        con.execute(query)

        # Export the cropped edges, intersections, and building groups to Parquet files
        logging.info("Exporting cropped data to Parquet files...")
        con.export_parquet(
            schema_name=SCHEMA_NAME,
            table_name=CROPPED_EDGES_TABLE_NAME,
            geom_col_name=GEOMETRY_COLUMN_NAME,
            output_file=output_edges_file,
        )
        con.export_parquet(
            schema_name=SCHEMA_NAME,
            table_name=CROPPED_INTERSECTIONS_TABLE_NAME,
            geom_col_name=GEOMETRY_COLUMN_NAME,
            output_file=output_intersections_file,
        )
        con.export_parquet(
            schema_name=SCHEMA_NAME,
            table_name=CROPPED_BUILDING_GROUPS_TABLE_NAME,
            geom_col_name=EXTENT_COLUMN_NAME,
            output_file=output_building_groups_file,
        )
