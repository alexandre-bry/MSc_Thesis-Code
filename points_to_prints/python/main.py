from typing import Annotated

import typer

from .bd_topo.main import app as bd_topo_app
from .lidar_hd.main import app as lidar_hd_app
from .pipeline.main import app as pipeline_app
from .polygon_deformation.main import app as polygon_deformation_app
from .roof.main import app as roof_app
from .validation.main import app as validation_app

main_app = typer.Typer(no_args_is_help=True)

main_app.add_typer(
    pipeline_app,
    name="pipeline",
    help="Run the whole pipeline to produce roofprints and footprints in one go.",
)
main_app.add_typer(
    bd_topo_app,
    name="bd_topo",
    help="Pre-process the BD TOPO dataset.",
)
main_app.add_typer(
    lidar_hd_app,
    name="lidar_hd",
    help="Download and process the LiDAR HD dataset, and more generally manipulate LAS/LAZ point clouds.",
)
main_app.add_typer(
    roof_app,
    name="roof",
    help="Compute and manipulate the roof of a building.",
)
main_app.add_typer(
    validation_app,
    name="validation",
    help="Clean up the validation dataset and compute metrics.",
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
