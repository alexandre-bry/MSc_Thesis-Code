import logging
from pathlib import Path
from typing import Annotated

import typer

from ..lidar_hd.las_manipulations import get_las_bounds
from ..utils.custom_logging import LoggingContext
from .convert import convert_bd_topo_to_parquet_call
from .crop import crop_parquet_from_las
from .intersections import (
    compute_export_intersections,
    crop_intersections_files,
)

app = typer.Typer(no_args_is_help=True)


@app.command(
    "compute_intersections",
    help="Compute the intersections between edges in the BD TOPO using DuckDB and export the edges and intersections to parquet files.",
)
def bd_topo_intersections(
    bd_topo_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo_file",
            help="Path to the BD_TOPO parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_edges_file: Annotated[
        Path,
        typer.Option(
            "-e",
            "--output_edges_file",
            help="Path to the output edges parquet file.",
        ),
    ],
    output_intersections_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--output_intersections_file",
            help="Path to the output intersections parquet file.",
        ),
    ],
    output_groups_file: Annotated[
        Path,
        typer.Option(
            "-g",
            "--output_groups_file",
            help="Path to the output building groups parquet file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        compute_export_intersections(
            bd_topo_file=bd_topo_file,
            output_edges_file=output_edges_file,
            output_intersections_file=output_intersections_file,
            output_building_groups_file=output_groups_file,
            overwrite=overwrite,
        )


@app.command(
    "crop_intersections",
    help="Crop the Parquet files corresponding to the intersections in the BD TOPO to the bounds of a given LAS/LAZ file and export the cropped data to a parquet file.",
)
def bd_topo_crop_intersections(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
            help="Path to the input LAS/LAZ file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    edges_file: Annotated[
        Path,
        typer.Option(
            "-e",
            "--edges_file",
            help="Path to the edges parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    intersections_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--intersections_file",
            help="Path to the intersections parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    groups_file: Annotated[
        Path,
        typer.Option(
            "-g",
            "--groups_file",
            help="Path to the building groups parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_edges_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_edges_file",
            help="Path to the output cropped edges parquet file.",
        ),
    ],
    output_intersections_file: Annotated[
        Path,
        typer.Option(
            "-p",
            "--output_intersections_file",
            help="Path to the output cropped intersections parquet file.",
        ),
    ],
    output_groups_file: Annotated[
        Path,
        typer.Option(
            "-q",
            "--output_groups_file",
            help="Path to the output cropped building groups parquet file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip processing files that already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        bounding_box = get_las_bounds(las_file)
        crop_intersections_files(
            input_edges_file=edges_file,
            input_intersections_file=intersections_file,
            input_building_groups_file=groups_file,
            output_edges_file=output_edges_file,
            output_intersections_file=output_intersections_file,
            output_building_groups_file=output_groups_file,
            bounding_box=bounding_box,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )


@app.command(
    "crop_from_las",
    help="Crop a Parquet file to the bounds of a given LAS/LAZ file and export the cropped data to a parquet file.",
)
def bd_topo_crop_from_las(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
            help="Path to the input LAS/LAZ file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    input_parquet_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input_parquet_file",
            help="Path to the input parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_parquet_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_parquet_file",
            help="Path to the output cropped parquet file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip processing files that already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        crop_parquet_from_las(
            input_las_file=las_file,
            input_parquet_file=input_parquet_file,
            output_parquet_file=output_parquet_file,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )


@app.command(
    "convert",
    help="Convert the BD TOPO data from its original format to a parquet file with a geometry column in WKB format.",
)
def convert_bd_topo(
    input_path: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input_path",
            help="Path to the input BD TOPO file (e.g., a .gpkg file).",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_path: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_path",
            help="Path to the output parquet file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip processing files that already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    convert_bd_topo_to_parquet_call(
        input_path=input_path,
        output_path=output_path,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
    )


if __name__ == "__main__":
    app()
