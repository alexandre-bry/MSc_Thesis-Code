import logging
from pathlib import Path

from ..point_cloud.las_manipulations import get_las_bounds
from ..utils.duckdb_helpers import connect_to_duckdb, create_schema, export_parquet
from ..utils.geom import Box2154
from ..utils.input_output import InputOutput, OutputBehaviour

SCHEMA_NAME = "crop"
FINAL_TABLE_NAME = "cropped_buildings"
GEOMETRY_COLUMN_NAME = "geometry"


def crop_parquet(
    input_parquet_file: Path,
    output_parquet_file: Path,
    bounds: Box2154,
    input_output: InputOutput,
):
    message_prefix = "Distances and edges computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[input_parquet_file],
    )
    input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_parquet_file]],
    )

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
    input_output: InputOutput,
):
    message_prefix = "Distances and edges computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[input_las_file, input_parquet_file],
    )
    input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_parquet_file]],
    )

    bounds = get_las_bounds(input_las_file)
    crop_parquet(
        input_parquet_file=input_parquet_file,
        output_parquet_file=output_parquet_file,
        bounds=bounds,
        input_output=input_output,
    )
