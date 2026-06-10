import asyncio
import logging
from pathlib import Path
from typing import Annotated, List, Optional

import typer

from ..utils.custom_logging import LoggingContext
from .pipeline import run_pipeline_call

app = typer.Typer(no_args_is_help=True)


@app.command(
    "points_to_prints",
    help="Run the pipeline to compute roofprints and footprints from the BD TOPO and the LiDAR HD.",
)
def run_pipeline_command(
    bd_topo_dir: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo_dir",
            help="Directory containing the BD TOPO data needed for the pipeline.",
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
            help="Directory containing the downloaded LAS/LAZ tile (in `<tile_dir>/lidar_hd/lidar_hd.copc.laz`).",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    stop_after_roofprints: Annotated[
        bool,
        typer.Option(
            "--stop_after_roofprints",
            help="Whether to stop the pipeline after computing roofprints (skipping LoD22 and footprints).",
        ),
    ] = False,
    stop_after_lod22: Annotated[
        bool,
        typer.Option(
            "--stop_after_lod22",
            help="Whether to stop the pipeline after computing LoD22 (skipping footprints).",
        ),
    ] = False,
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
        bd_topo_dir=bd_topo_dir,
        tile_dir=tile_dir,
        stop_after_roofprints=stop_after_roofprints,
        stop_after_lod22=stop_after_lod22,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
        num_workers=num_workers,
    )


@app.command(
    "metrics",
    help="Compute the validation metrics by comparing the scored dataset to the ground-truth dataset.",
)
def compute_metrics_command(
    validation_dataset_indiv_path: Annotated[
        Path,
        typer.Option(
            "-i",
            "--validation_dataset_indiv",
            help="Path to the individual building validation dataset (Parquet file).",
        ),
    ],
    validation_dataset_aggreg_path: Annotated[
        Path,
        typer.Option(
            "-a",
            "--validation_dataset_aggreg",
            help="Path to the aggregated building validation dataset (Parquet file).",
        ),
    ],
    bd_topo_path: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo",
            help="Path to the BD TOPO polygon dataset to compare.",
        ),
    ],
    tiles_dirs: Annotated[
        List[Path],
        typer.Option(
            "-t",
            "--tiles_dir",
            help="List of tile directories containing the pipeline output to compare.",
        ),
    ],
    output_comparison_dir: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_comparison_dir",
            help="Directory where the comparison results will be saved.",
        ),
    ],
    output_format: Annotated[
        str,
        typer.Option(
            "--output_format",
            help="Format to save the comparison results (e.g., 'parquet', 'csv', 'json').",
        ),
    ],
    id_column: Annotated[
        str,
        typer.Option(
            "--id_column",
            help="Name of the column containing the building IDs in the datasets.",
        ),
    ],
    spacing_m: Annotated[
        float,
        typer.Option(
            "--spacing_m",
            help="Spacing in meters to use for the comparison.",
        ),
    ],
    keep_columns: Annotated[
        Optional[List[str]],
        typer.Option(
            "-k",
            "--keep_columns",
            help=(
                "List of additional column names to keep in the output comparison results (in addition to the id_column). If not provided, only the id_column will be kept."
            ),
        ),
    ] = None,
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite existing comparison results.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip comparison if results already exist.",
        ),
    ] = False,
    num_workers: Annotated[
        Optional[int],
        typer.Option(
            "--num-workers",
            help="Maximum number of multiprocessing workers (defaults to the platform default).",
        ),
    ] = None,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
) -> None:
    from .pipeline import compute_metrics_call

    if num_workers is not None and num_workers < 1:
        raise typer.BadParameter("--num-workers must be >= 1.")

    compute_metrics_call(
        validation_dataset_indiv_file=validation_dataset_indiv_path,
        validation_dataset_aggreg_file=validation_dataset_aggreg_path,
        bd_topo_file=bd_topo_path,
        tiles_dirs=tiles_dirs,
        output_comparison_dir=output_comparison_dir,
        output_format=output_format,
        id_column=id_column,
        spacing_m=spacing_m,
        keep_columns=keep_columns,
        overwrite=overwrite,
        skip_existing=skip_existing,
        num_workers=num_workers,
        verbose_int=verbose_int,
    )


if __name__ == "__main__":
    app()
