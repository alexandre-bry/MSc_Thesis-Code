import logging
from pathlib import Path

from ..point_cloud import get_las_bounds
from ..utils import (
    Box2154,
    DuckDBConnectionManager,
    InputOutput,
    OutputActionEnum,
    OutputBehaviour,
)

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
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_parquet_file]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    db_path = output_parquet_file.parent / (output_parquet_file.stem + ".duckdb")
    with DuckDBConnectionManager(db_path) as con:
        con.create_schema(SCHEMA_NAME)
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

        con.export_parquet(
            schema_name=SCHEMA_NAME,
            table_name=FINAL_TABLE_NAME,
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
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_parquet_file]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    bounds = get_las_bounds(input_las_file)
    crop_parquet(
        input_parquet_file=input_parquet_file,
        output_parquet_file=output_parquet_file,
        bounds=bounds,
        input_output=input_output,
    )
