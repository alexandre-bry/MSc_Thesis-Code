from pathlib import Path

from ..utils.custom_logging import LoggingContext
from ..utils.duckdb_helpers import DuckDBConnectionManager

SCHEMA_NAME = "bd_topo"

INITIAL_GEOMETRY_COLUMN_NAME = "geom"
GEOMETRY_COLUMN_NAME = "geometry"


def convert_bd_topo_to_parquet_implementation(
    input_path: Path,
    output_path: Path,
    overwrite: bool,
    skip_existing: bool,
):
    db_path = output_path.parent / (output_path.stem + ".duckdb")
    with DuckDBConnectionManager(db_path) as con:
        con.create_schema(SCHEMA_NAME)

        read_query = f"""
            CREATE OR REPLACE TABLE {SCHEMA_NAME}.bd_topo AS
            SELECT 
                * EXCLUDE ({INITIAL_GEOMETRY_COLUMN_NAME}), 
                {INITIAL_GEOMETRY_COLUMN_NAME} AS {GEOMETRY_COLUMN_NAME}
            FROM '{input_path}';
        """

        con.execute(read_query)

        con.export_parquet(
            schema_name=SCHEMA_NAME,
            table_name="bd_topo",
            geom_col_name=GEOMETRY_COLUMN_NAME,
            output_file=output_path,
        )


def convert_bd_topo_to_parquet_call(
    input_path: Path,
    output_path: Path,
    overwrite: bool,
    skip_existing: bool,
    verbose_int: int,
):
    with LoggingContext(verbose=verbose_int):
        convert_bd_topo_to_parquet_implementation(
            input_path=input_path,
            output_path=output_path,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )
