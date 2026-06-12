"""
Main entry point for the CLI application.
This module defines the main Typer app and registers all sub-apps for different functionalities.
"""

from typing import Annotated

import typer

from .bd_topo.main import app as bd_topo_app
from .lidar_hd.main import app as lidar_hd_app
from .outline.main import app as outline_app
from .pipeline.main import app as pipeline_app
from .point_cloud.main import app as point_cloud_app
from .polygon_deformation.main import app as polygon_deformation_app
from .roof.main import app as roof_app
from .validation.main import app as validation_app

main_app = typer.Typer(no_args_is_help=True)

main_app.add_typer(
    pipeline_app,
    name="pipeline",
    help="Wrappers to run the entire pipeline on the BD TOPO and LiDAR HD datasets.",
)
main_app.add_typer(
    bd_topo_app,
    name="bd_topo",
    help="Functions specific to the BD TOPO dataset.",
)
main_app.add_typer(
    lidar_hd_app,
    name="lidar_hd",
    help="Functions specific to the LiDAR HD dataset.",
)
main_app.add_typer(
    outline_app,
    name="outline",
    help="Manipulate building outlines.",
)
main_app.add_typer(
    point_cloud_app,
    name="point_cloud",
    help="Manipulate point clouds.",
)
main_app.add_typer(
    roof_app,
    name="roof",
    help="Compute the roof of a building.",
)
main_app.add_typer(
    validation_app,
    name="validation",
    help="Clean up the validation dataset and use it to compute metrics.",
)
main_app.add_typer(
    polygon_deformation_app,
    name="polygon_deformation",
    help="Play with the polygon deformation and polygon matching algorithms on toy datasets.",
)


@main_app.command(
    "test_logging",
    help="A simple command to test the logging setup with different verbosity levels.",
)
def test(verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0):
    import logging

    from .utils.custom_logging import LoggingContext

    with LoggingContext(verbose=verbose_int):
        logging.debug("This is a debug message.")
        logging.info("This is an info message.")
        logging.warning("This is a warning message.")
        logging.error("This is an error message.")


if __name__ == "__main__":
    main_app()
