import asyncio
import logging
from pathlib import Path
from typing import Annotated, List, Optional

import typer

from ..utils.custom_logging import LoggingContext
from .pipeline import run_pipeline_call

app = typer.Typer()


@app.command(
    "run_pipeline",
    help="Run the pipeline.",
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
            help="Directory containing the downloaded tile .laz files.",
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
