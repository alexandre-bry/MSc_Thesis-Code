from pathlib import Path
from typing import Annotated, List, Optional

import typer

from ..utils import InputOutput, Verbose

app = typer.Typer(no_args_is_help=True)


@app.command("download", help="Download LiDAR HD data for a specified bounding box.")
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
            "--output-path",
            help="Path to save the downloaded files. The path can contain the values {xmin}, {ymin}, {xmax}, {ymax}, {file_name} which will be replaced with the corresponding values. The values also have their kilometre equivalents {xmin_km}, {ymin_km}, {xmax_km}, {ymax_km}.",
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
    bbox_values = bbox.split(",")
    if len(bbox_values) != 4:
        raise ValueError("Bounding box must be in the format 'xmin,ymin,xmax,ymax'.")
    xmin, ymin, xmax, ymax = map(int, bbox_values)

    from .download import download_lidar_hd_call

    download_lidar_hd_call(
        xmin=xmin,
        ymin=ymin,
        xmax=xmax,
        ymax=ymax,
        output_path_template=output_path_template,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
        concurrency=concurrency,
    )


if __name__ == "__main__":
    app()
