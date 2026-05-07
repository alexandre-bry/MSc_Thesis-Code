import asyncio
import logging
from pathlib import Path
from typing import Annotated, List, Optional

import typer

from ..utils.custom_logging import LoggingContext, run_command_with_tqdm_logging
from .bd_topo_convert import convert_bd_topo_to_parquet_call
from .bd_topo_crop import crop_parquet_from_las
from .bd_topo_intersections import (
    compute_export_intersections,
    crop_intersections_files,
)
from .download import download_lidar_hd_data
from .las_manipulations import (
    get_las_bounds,
    identity_convert,
    merge_files,
    split_point_cloud_call,
)
from .pipeline import run_pipeline_call

app = typer.Typer()


@app.command(
    "split_las",
    help="Split a .laz file into one file for each value of a specified dimension.",
)
def split_point_cloud_command(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the .laz file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file_template: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_file_template",
            help="Template for the output .laz files. Use # as a placeholder for the dimension value.",
        ),
    ],
    dimension: Annotated[
        str,
        typer.Option(
            "-d",
            "--dimension",
            help="The dimension to split by (e.g., 'Classification').",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
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
    split_point_cloud_call(
        input_file=input_file,
        output_file_template=output_file_template,
        dimension=dimension,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
    )


@app.command("merge_las", help="Merge multiple .laz files into a single .laz file.")
def merge_las(
    input_files: Annotated[
        List[Path],
        typer.Option(
            "-i",
            "--input",
            help="Paths to the .laz files. Can be used multiple times.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_file",
            help="Path to the output .laz file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
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
        output_file.parent.mkdir(parents=True, exist_ok=True)

        merge_files(
            input_files=input_files,
            output_file=output_file,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )
        logging.info(f"Successfully merged files into {output_file}")


@app.command(
    "download_lidar_hd", help="Download LiDAR HD data for a specified bounding box."
)
def download_lidar_hd(
    bbox: Annotated[
        str,
        typer.Option(
            "-b",
            "--bbox",
            help="Bounding box to download data for, in the format 'xmin,ymin,xmax,ymax'.",
        ),
    ],
    output_path_template: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_path",
            help="Path to save the downloaded files. The path can contain the values {xmin}, {ymin}, {xmax}, {ymax}, {file_name} which will be replaced with the corresponding values. The values also have their kilometre equivalents {xmin_km}, {ymin_km}, {xmax_km}, {ymax_km}.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        bbox_values = bbox.split(",")
        if len(bbox_values) != 4:
            raise ValueError(
                "Bounding box must be in the format 'xmin,ymin,xmax,ymax'."
            )
        xmin, ymin, xmax, ymax = map(int, bbox_values)

        asyncio.run(
            download_lidar_hd_data(
                xmin=xmin,
                ymin=ymin,
                xmax=xmax,
                ymax=ymax,
                output_path_template=output_path_template,
                overwrite=overwrite,
            )
        )


@app.command(
    "run_pipeline",
    help="Run the pipeline.",
)
def run_pipeline_command(
    other_data_dir: Annotated[
        Path,
        typer.Option(
            "-d",
            "--other_data_dir",
            help="Directory containing the other data files (BD TOPO parquet file, etc.) needed for the pipeline.",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    tile_dir: Annotated[
        Path,
        typer.Option(
            "-t",
            "--tile_dir",
            help="Directory containing the downloaded tile .laz files.",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip processing files that already have output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
    num_workers: Annotated[Optional[int], typer.Option("--num_workers")] = None,
):
    run_pipeline_call(
        other_data_dir=other_data_dir,
        tile_dir=tile_dir,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
        num_workers=num_workers,
    )


@app.command(
    "bd_topo_intersections",
    help="Compute the intersections between edges in the BD TOPO and export the edges and intersections to parquet files.",
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
    "bd_topo_crop_intersections",
    help="Crop the BD TOPO data to the bounds of a given LAS file and export the cropped data to a parquet file.",
)
def bd_topo_crop_intersections(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
            help="Path to the input LAS file.",
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
    "identity",
    help="A simple command to test the pipeline by copying an input point cloud to an output file without any modifications.",
)
def identity(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the input point cloud file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_file",
            help="Path to the output point cloud file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        identity_convert(
            input_file=input_file, output_file=output_file, overwrite=overwrite
        )


@app.command(
    "bd_topo_crop_from_las",
    help="Crop the BD TOPO data to the bounds of a given LAS file and export the cropped data to a parquet file.",
)
def bd_topo_crop_from_las(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
            help="Path to the input LAS file.",
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
    "bd_topo_convert",
    help="Convert the BD TOPO data from its original format to a parquet file with a geometry column in WKB format.",
)
def bd_topo_convert(
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


@app.command(
    "test_logging",
    help="A simple command to test the logging setup with different verbosity levels.",
)
def test(verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0):
    with LoggingContext(verbose=verbose_int):
        logging.debug("This is a debug message.")
        logging.info("This is an info message.")
        logging.warning("This is a warning message.")
        logging.error("This is an error message.")


if __name__ == "__main__":
    app()
