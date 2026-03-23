import logging
from pathlib import Path
from pprint import pformat

import duckdb
from duckdb_helpers import connect_to_duckdb, create_schema, export_parquet
from pdal import Filter, Pipeline, Reader, Writer
from utils import Box2154, Point2154

SCHEMA_NAME = "crop"

FINAL_TABLE_NAME = "edges"
GEOMETRY_COLUMN_NAME = "geometry"


def get_las_bounds(las_file: Path) -> Box2154:
    # Open the LAS file and read the header to get the bounds
    reader = Reader(str(las_file))
    pipeline = Pipeline([reader])
    pipeline.execute()
    metadata = pipeline.metadata

    las_metadata = metadata["metadata"]
    proj_metadata = None
    if "readers.las" in las_metadata:
        proj_metadata = las_metadata["readers.las"]
    elif "readers.copc" in las_metadata:
        proj_metadata = las_metadata["readers.copc"]
    else:
        raise ValueError(
            f"Could not find LAS metadata for file '{las_file}' in PDAL metadata output"
        )

    if (
        "minx" not in proj_metadata
        or "miny" not in proj_metadata
        or "maxx" not in proj_metadata
        or "maxy" not in proj_metadata
    ):
        raise ValueError(
            f"Could not retrieve bounds from LAS metadata for file '{las_file}'"
        )
    minx = int(float(proj_metadata["minx"]))
    miny = int(float(proj_metadata["miny"]))
    maxx = int(float(proj_metadata["maxx"]))
    maxy = int(float(proj_metadata["maxy"]))
    return Box2154(Point2154(minx, miny), Point2154(maxx, maxy))


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
        CREATE TABLE {SCHEMA_NAME}.{FINAL_TABLE_NAME} AS
        SELECT *
        FROM read_parquet('{input_parquet_file}')
        WHERE ST_Intersects(
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
):
    bounds = get_las_bounds(input_las_file)
    crop_parquet(input_parquet_file, output_parquet_file, bounds)


def crop_edges_and_intersections(
    input_intersections_file: Path,
    input_edges_file: Path,
    input_las_file: Path,
    output_intersections_file: Path,
    output_edges_file: Path,
):
    bounds = get_las_bounds(input_las_file)
    crop_parquet(input_edges_file, output_edges_file, bounds)
    crop_parquet(input_intersections_file, output_intersections_file, bounds)
