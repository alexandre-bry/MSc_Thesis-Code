from pathlib import Path
from typing import Optional

from ..lidar_hd import download_lidar_hd_data_implementation
from ..utils import InputOutput, LoggingContext, Verbose


def download_lidar_hd_pipeline_implementation(
    xmin: int,
    xmax: int,
    ymin: int,
    ymax: int,
    tiles_dir: Path,
    input_output: InputOutput,
    concurrency: Optional[int],
):
    output_path_template = (
        Path(tiles_dir) / "{xmin_km}_{ymin_km}/lidar_hd/lidar_hd.copc.laz"
    )
    download_lidar_hd_data_implementation(
        xmin=xmin,
        xmax=xmax,
        ymin=ymin,
        ymax=ymax,
        output_path_template=output_path_template,
        input_output=input_output,
        concurrency=concurrency,
    )


def download_lidar_hd_pipeline_call(
    xmin: int,
    xmax: int,
    ymin: int,
    ymax: int,
    tiles_dir: Path,
    input_output: InputOutput,
    verbose: Verbose,
    concurrency: Optional[int],
):
    with LoggingContext(verbose=verbose):
        download_lidar_hd_pipeline_implementation(
            xmin=xmin,
            xmax=xmax,
            ymin=ymin,
            ymax=ymax,
            tiles_dir=tiles_dir,
            input_output=input_output,
            concurrency=concurrency,
        )
