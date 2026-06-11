import logging
from pathlib import Path

from ..lidar_hd.las_manipulations import get_las_bounds
from ..utils.duckdb_helpers import connect_to_duckdb, create_schema, export_parquet
from ..utils.geom import Box2154

SCHEMA_NAME = "crop"

BD_TOPO_TABLE_NAME = "bd_topo"
EDGES_TABLE_NAME = "edges"
INTERSECTIONS_TABLE_NAME = "intersections"
FINAL_TABLE_NAME = "cropped_buildings"

BUILDING_ID_COLUMN_NAME = "cleabs"
EDGE_ID_COLUMN_NAME = "id"
EDGE_ID_A_COLUMN_NAME = "id_a"
EDGE_ID_B_COLUMN_NAME = "id_b"
GEOMETRY_COLUMN_NAME = "geometry"


def crop_parquet(
    input_parquet_file: Path,
    output_parquet_file: Path,
    bounds: Box2154,
):
    logging.debug(f"{output_parquet_file = }")
    db_path = output_parquet_file.parent / (output_parquet_file.stem + ".duckdb")
    logging.debug(f"{db_path = }")
    con = connect_to_duckdb(db_path)
    create_schema(con, SCHEMA_NAME)
    query = f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{FINAL_TABLE_NAME} AS
        SELECT *
        FROM read_parquet('{input_parquet_file}')
        WHERE ST_Covers(
            ST_MakeEnvelope(
                {bounds.p_min.x},
                {bounds.p_min.y},
                {bounds.p_max.x},
                {bounds.p_max.y}
            ),
            {GEOMETRY_COLUMN_NAME}
        )
    """
    con.execute(query)

    export_parquet(
        con=con,
        table_name=f"{SCHEMA_NAME}.{FINAL_TABLE_NAME}",
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_parquet_file,
    )


def crop_parquet_from_las(
    input_las_file: Path,
    input_parquet_file: Path,
    output_parquet_file: Path,
    overwrite: bool,
    skip_existing: bool,
):
    # Check if output file already exists
    if output_parquet_file.exists():
        if skip_existing:
            logging.info(
                f"Output file '{output_parquet_file}' already exists. Skipping."
            )
            return
        if overwrite:
            logging.warning(
                f"Output file '{output_parquet_file}' already exists. Overwriting."
            )
        else:
            logging.error(
                f"Output file '{output_parquet_file}' already exists. Use --overwrite to overwrite it."
            )
            return

    bounds = get_las_bounds(input_las_file)
    crop_parquet(input_parquet_file, output_parquet_file, bounds)


def crop_bd_topo_files(
    input_las_file: Path,
    input_bd_topo_file: Path,
    input_edges_file: Path,
    input_intersections_file: Path,
    output_edges_file: Path,
    output_intersections_file: Path,
    overwrite: bool,
):
    for file in [output_edges_file, output_intersections_file]:
        if file.exists():
            if overwrite:
                logging.warning(f"Output file '{file}' already exists. Overwriting.")
            else:
                logging.error(
                    f"Output file '{file}' already exists. Use --overwrite to overwrite it."
                )
                return

    bounds = get_las_bounds(input_las_file)
    logging.info(f"Cropping BD TOPO files to bounds: {bounds}")

    # Create a temporary DuckDB database to store the cropped data
    db_path = output_edges_file.parent / (output_edges_file.stem + ".duckdb")
    con = connect_to_duckdb(db_path)
    create_schema(con, SCHEMA_NAME)

    # Select buildings that fully fit in the bounds
    logging.info("Selecting buildings that fully fit in the bounds...")
    query = f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{BD_TOPO_TABLE_NAME} AS
        SELECT {BUILDING_ID_COLUMN_NAME}
        FROM read_parquet('{input_bd_topo_file}')
        WHERE ST_Covers(
            ST_MakeEnvelope(
                {bounds.p_min.x},
                {bounds.p_min.y},
                {bounds.p_max.x},
                {bounds.p_max.y}
            ),
            {GEOMETRY_COLUMN_NAME}
        )
    """
    con.execute(query)

    # Select edges that are part of the selected buildings
    logging.info("Selecting edges that are part of the selected buildings...")
    query = f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{EDGES_TABLE_NAME} AS
        SELECT *
        FROM read_parquet('{input_edges_file}')
        WHERE {BUILDING_ID_COLUMN_NAME} IN (
            SELECT {BUILDING_ID_COLUMN_NAME}
            FROM {SCHEMA_NAME}.{BD_TOPO_TABLE_NAME}
        )
    """
    con.execute(query)

    # Select intersections of two edges in the selected edges
    logging.info("Selecting intersections of two edges in the selected edges...")
    query = f"""
        CREATE OR REPLACE TABLE {SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME} AS
        SELECT *
        FROM read_parquet('{input_intersections_file}')
        WHERE {EDGE_ID_A_COLUMN_NAME} IN (
            SELECT {EDGE_ID_COLUMN_NAME}
            FROM {SCHEMA_NAME}.{EDGES_TABLE_NAME}
        ) AND {EDGE_ID_B_COLUMN_NAME} IN (
            SELECT {EDGE_ID_COLUMN_NAME}
            FROM {SCHEMA_NAME}.{EDGES_TABLE_NAME}
        )
    """
    con.execute(query)

    # Export the cropped edges and intersections to Parquet files
    logging.info(f"Exporting cropped edges to '{output_edges_file}'...")
    export_parquet(
        con=con,
        table_name=f"{SCHEMA_NAME}.{EDGES_TABLE_NAME}",
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_edges_file,
    )

    logging.info(f"Exporting cropped intersections to '{output_intersections_file}'...")
    export_parquet(
        con=con,
        table_name=f"{SCHEMA_NAME}.{INTERSECTIONS_TABLE_NAME}",
        geom_col_name=GEOMETRY_COLUMN_NAME,
        output_file=output_intersections_file,
    )

    # Delete the temporary DuckDB database
    logging.debug(
        f"Closing DuckDB connection and deleting temporary database at '{db_path}'..."
    )
    con.close()
    if db_path.exists():
        db_path.unlink()
    logging.info(f"Done.")
