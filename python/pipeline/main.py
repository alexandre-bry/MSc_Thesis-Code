from pathlib import Path
from typing import Annotated, List, Optional

import typer

from ..utils import InputOutput, Verbose

app = typer.Typer(no_args_is_help=True)


@app.command(
    "download_lidar_hd",
    help="Download LiDAR HD data for a specified bounding box.",
)
def download_lidar_hd_pipeline_command(
    bbox: Annotated[
        str,
        typer.Option(
            "-b",
            "--bbox",
            help="Bounding box to download data for, in the format 'xmin,ymin,xmax,ymax'.",
        ),
    ],
    tiles_dir: Annotated[
        Path,
        typer.Option(
            "-t",
            "--tiles-dir",
            help="Directory where to download the tiles.",
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
            "--skip-existing",
            help="Whether to skip steps if output files already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
    concurrency: Annotated[
        Optional[int],
        typer.Option(
            "--concurrency",
            help="Number of concurrent requests to send.",
        ),
    ] = None,
):
    from .lidar_hd import download_lidar_hd_pipeline_call

    bbox_values = bbox.split(",")
    if len(bbox_values) != 4:
        raise ValueError("Bounding box must be in the format 'xmin,ymin,xmax,ymax'.")
    xmin, ymin, xmax, ymax = map(int, bbox_values)

    download_lidar_hd_pipeline_call(
        xmin=xmin,
        ymin=ymin,
        xmax=xmax,
        ymax=ymax,
        tiles_dir=tiles_dir,
        input_output=InputOutput.from_flags(
            overwrite=overwrite, skip_existing=skip_existing
        ),
        verbose=Verbose.from_int(verbose_int),
        concurrency=concurrency,
    )


@app.command(
    "prepare_bd_topo",
    help="Prepare the BD TOPO data for the pipeline.",
)
def prepare_bd_topo_command(
    bd_topo_source_file: Annotated[
        Path,
        typer.Option(
            "-s",
            "--bd-topo-source-file",
            help="Path to the source BD TOPO file (e.g., GeoPackage) to prepare.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    bd_topo_output_dir: Annotated[
        Path,
        typer.Option(
            "-o",
            "--bd-topo-output-dir",
            help="Directory where the prepared BD TOPO files will be saved.",
            exists=False,
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
            "--skip-existing",
            help="Whether to skip steps if output files already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
) -> None:
    from .bd_topo import prepare_bd_topo_call

    prepare_bd_topo_call(
        bd_topo_source_file=bd_topo_source_file,
        bd_topo_output_dir=bd_topo_output_dir,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


@app.command(
    "points_to_prints",
    help="Run the pipeline to compute roofprints and footprints from the BD TOPO and the LiDAR HD.",
)
def run_pipeline_command(
    bd_topo_dir: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd-topo-dir",
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
            "--tile-dir",
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
            "--stop-after-roofprints",
            help="Whether to stop the pipeline after computing roofprints (skipping LoD22 and footprints).",
        ),
    ] = False,
    stop_after_lod22: Annotated[
        bool,
        typer.Option(
            "--stop-after-lod22",
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
            "--skip-existing",
            help="Whether to skip processing files that already have output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
    num_workers: Annotated[
        Optional[int],
        typer.Option(
            "--num-workers",
            help="Maximum number of multiprocessing workers (defaults to the platform default).",
        ),
    ] = None,
):
    from .pipeline import run_pipeline_call

    run_pipeline_call(
        bd_topo_dir=bd_topo_dir,
        tile_dir=tile_dir,
        stop_after_roofprints=stop_after_roofprints,
        stop_after_lod22=stop_after_lod22,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
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
            "--validation-dataset-indiv",
            help="Path to the individual building validation dataset (Parquet file).",
        ),
    ],
    validation_dataset_aggreg_path: Annotated[
        Path,
        typer.Option(
            "-a",
            "--validation-dataset-aggreg",
            help="Path to the aggregated building validation dataset (Parquet file).",
        ),
    ],
    bd_topo_path: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd-topo",
            help="Path to the BD TOPO polygon dataset to compare.",
        ),
    ],
    tiles_dirs: Annotated[
        List[Path],
        typer.Option(
            "-t",
            "--tiles-dir",
            help="List of tile directories containing the pipeline output to compare.",
        ),
    ],
    output_comparison_dir: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output-comparison-dir",
            help="Directory where the comparison results will be saved.",
        ),
    ],
    output_format: Annotated[
        str,
        typer.Option(
            "--output-format",
            help="Format to save the comparison results (e.g., 'parquet', 'csv', 'json').",
        ),
    ],
    id_column: Annotated[
        str,
        typer.Option(
            "--id-column",
            help="Name of the column containing the building IDs in the datasets.",
        ),
    ],
    spacing_m: Annotated[
        float,
        typer.Option(
            "--spacing-m",
            help="Spacing in meters to use for the comparison.",
        ),
    ],
    keep_columns: Annotated[
        Optional[List[str]],
        typer.Option(
            "-k",
            "--keep-columns",
            help=(
                "List of additional column names to keep in the output comparison results (in addition to the id_column). If not provided, only the id_column will be kept."
            ),
        ),
    ] = None,
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
            "--skip-existing",
            help="Whether to skip steps if output files already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
    num_workers: Annotated[
        Optional[int],
        typer.Option(
            "--num-workers",
            help="Maximum number of multiprocessing workers (defaults to the platform default).",
        ),
    ] = None,
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
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
        num_workers=num_workers,
    )


if __name__ == "__main__":
    app()
