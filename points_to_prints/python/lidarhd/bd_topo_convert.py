import logging
from pathlib import Path
from tempfile import TemporaryDirectory

from ..utils.custom_logging import LoggingContext, run_command_with_tqdm_logging

from ..utils.duckdb_helpers import DuckDBConnectionManager

SCHEMA_NAME = "bd_topo"
TABLE_NAME = "buildings"

INITIAL_GEOMETRY_COLUMN_NAME = "geometrie"
FINAL_GEOMETRY_COLUMN_NAME = "geometry"


def convert_bd_topo_to_parquet_implementation(
    input_path: Path,
    output_path: Path,
    overwrite: bool,
    skip_existing: bool,
):
    if skip_existing and output_path.exists():
        logging.info(f"Output file {output_path} already exists. Skipping conversion.")
        return
    if output_path.exists() and not overwrite:
        logging.warning(
            f"Output file {output_path} already exists. Use --overwrite to replace it."
        )
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Create a temporary directory to store intermediate files
    logging.info(f"Creating temporary directory for processing {input_path.name}.")
    with TemporaryDirectory() as temp_dir:
        temp_dir_path = Path(temp_dir)

        # Extract the buildings from the BD TOPO file
        logging.info(f"Extracting buildings from {input_path.name}.")
        sql_query = f"SELECT ST_ForcePolygonCCW({INITIAL_GEOMETRY_COLUMN_NAME}) AS {FINAL_GEOMETRY_COLUMN_NAME}, * FROM batiment"
        extracted_file_path = temp_dir_path / "extracted.gpkg"
        extraction_command = [
            "gdal",
            "vector",
            "sql",
            "-i",
            str(input_path),
            "-o",
            str(extracted_file_path),
            "--output-layer",
            "buildings",
            "--sql",
            sql_query,
        ]

        return_code = run_command_with_tqdm_logging(extraction_command, display=True)
        if return_code != 0:
            logging.error(f"Failed to extract buildings from {input_path.name}.")
        else:
            logging.info(f"Successfully extracted buildings from {input_path.name}.")

        # Add a column with the integer part of the building ID
        logging.info(f"Processing extracted data from {input_path.name}.")
        db_path = temp_dir_path / "duckdb.duckdb"
        processed_file_path = temp_dir_path / "processed.parquet"
        with DuckDBConnectionManager(db_path) as con:
            con.create_schema(SCHEMA_NAME)

            read_query = f"""
                CREATE OR REPLACE TABLE {SCHEMA_NAME}.{TABLE_NAME} AS (
                    SELECT CAST(array_slice(cleabs, 9, -1) AS INT64) AS cleabs_int, *
                    FROM '{extracted_file_path}'
                );
            """

            con.execute(read_query)

            con.export_parquet(
                schema_name=SCHEMA_NAME,
                table_name=TABLE_NAME,
                geom_col_name=FINAL_GEOMETRY_COLUMN_NAME,
                output_file=processed_file_path,
            )

        # Format the file properly with gpio
        logging.info(
            f"Formatting the processed data from {input_path.name} to Parquet."
        )
        format_command = [
            "gpio",
            "convert",
            str(processed_file_path),
            str(output_path),
        ]

        return_code = run_command_with_tqdm_logging(format_command, display=True)
        if return_code != 0:
            logging.error(
                f"Failed to format the data from {input_path.name} to Parquet."
            )
        else:
            logging.info(
                f"Successfully formatted the data from {input_path.name} to Parquet."
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
