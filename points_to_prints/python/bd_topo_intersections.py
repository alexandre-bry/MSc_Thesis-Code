import logging
from pathlib import Path

import duckdb
from duckdb_helpers import connect_to_duckdb, create_schema, export_parquet

SCHEMA_NAME = "intersections"

MULTIPOLY_TABLE_NAME = "multipoly"
POLY_TABLE_NAME = "poly"
RINGS_TABLE_NAME = "rings"
EDGES_TABLE_NAME = "edges"
INTERSECTIONS_TABLE_NAME = "intersections"

GEOMETRY_COLUMN_NAME = "geometry"


def load_bd_topo_to_duckdb(con: duckdb.DuckDBPyConnection, bd_topo_file: Path):
    logging.info(f"Loading BD TOPO file '{bd_topo_file}' into DuckDB...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{MULTIPOLY_TABLE_NAME} AS
        SELECT * FROM read_parquet($bd_topo_file);
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


def unnest_multipoly_to_poly(con: duckdb.DuckDBPyConnection):
    logging.info("Unnesting MultiPolygons into Polygons...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{POLY_TABLE_NAME} AS
        SELECT
            cleabs, path[1] - 1 AS idx_polygon,
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


def unnest_poly_to_rings(con: duckdb.DuckDBPyConnection):
    logging.info("Unnesting polygons into rings...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{RINGS_TABLE_NAME} AS
        SELECT 
            cleabs,
            idx_polygon,
            idx_ring,
            CASE
                WHEN i.idx_ring = 0 THEN ST_ExteriorRing({GEOMETRY_COLUMN_NAME})
                ELSE ST_InteriorRingN({GEOMETRY_COLUMN_NAME}, i.idx_ring)
            END AS {GEOMETRY_COLUMN_NAME}
        FROM {SCHEMA_NAME}.{POLY_TABLE_NAME}
        CROSS JOIN generate_series(0, ST_NInteriorRings({GEOMETRY_COLUMN_NAME})) AS i(idx_ring);
        """
    )

    # Get the number of rows in the rings table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{RINGS_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of rings: {num_rows:_}")


def unnest_rings_to_edges(con: duckdb.DuckDBPyConnection):
    logging.info("Unnesting rings into edges...")
    # Unnest the LinearRingZ into LineStringZ (edges):
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{EDGES_TABLE_NAME} AS
        SELECT
            cleabs,
            idx_polygon,
            idx_ring,
            idx_point-1 AS idx_edge,
            ST_MakeLine(
                ST_PointN({GEOMETRY_COLUMN_NAME}, idx_point::INTEGER),
                ST_PointN({GEOMETRY_COLUMN_NAME}, (idx_point + 1)::INTEGER)
            ) AS {GEOMETRY_COLUMN_NAME}
        FROM {SCHEMA_NAME}.{RINGS_TABLE_NAME}
        CROSS JOIN generate_series(1, ST_NPoints({GEOMETRY_COLUMN_NAME}) - 1) AS i(idx_point)
        ORDER BY cleabs, idx_polygon, idx_ring, idx_edge;
        """
    )

    # Create an incremental key on the edges table
    con.execute(
        f"""
        CREATE OR REPLACE SEQUENCE seq_edges_ids START 1;
        ALTER TABLE {SCHEMA_NAME}.{EDGES_TABLE_NAME} ADD COLUMN id INTEGER;
        UPDATE {SCHEMA_NAME}.{EDGES_TABLE_NAME} SET id = nextval('seq_edges_ids');
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


def compute_intersections(con: duckdb.DuckDBPyConnection):
    logging.info("Computing intersections between edges...")
    con.execute(
        f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME} AS
        SELECT
            a.id AS id_a,
            b.id AS id_b,
            ST_Intersection(a.{GEOMETRY_COLUMN_NAME}, b.{GEOMETRY_COLUMN_NAME}) AS {GEOMETRY_COLUMN_NAME}
        FROM {SCHEMA_NAME}.{EDGES_TABLE_NAME} a
        JOIN {SCHEMA_NAME}.{EDGES_TABLE_NAME} b
            ON a.id < b.id
            AND ST_Intersects(a.{GEOMETRY_COLUMN_NAME}, b.{GEOMETRY_COLUMN_NAME});
        """
    )

    # Get the number of rows in the intersections table
    result = con.execute(
        f"SELECT COUNT(*) FROM {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME};"
    ).fetchone()
    num_rows = result[0] if result else 0
    logging.info(f"Number of intersections: {num_rows:_}")


def export_edges(con: duckdb.DuckDBPyConnection, output_edges_file: Path):
    logging.info(f"Exporting edges to '{output_edges_file}'...")
    export_parquet(
        con=con,
        table_name=f"{SCHEMA_NAME}.{EDGES_TABLE_NAME}",
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_edges_file,
    )
    logging.info(f"Exported.")


def export_intersections(
    con: duckdb.DuckDBPyConnection, output_intersections_file: Path
):
    logging.info(f"Exporting intersections to '{output_intersections_file}'...")
    export_parquet(
        con=con,
        table_name=f"{SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME}",
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_intersections_file,
    )
    logging.info(f"Exported.")


def compute_export_intersections(
    bd_topo_file: Path, output_edges_file: Path, output_intersections_file: Path
):
    con = connect_to_duckdb(db_file=Path("bd_topo_intersections.duckdb"))
    create_schema(con, schema_name=SCHEMA_NAME)
    load_bd_topo_to_duckdb(con, bd_topo_file)
    unnest_multipoly_to_poly(con)
    unnest_poly_to_rings(con)
    unnest_rings_to_edges(con)
    compute_intersections(con)
    export_edges(con, output_edges_file)
    export_intersections(con, output_intersections_file)
