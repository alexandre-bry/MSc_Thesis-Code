import logging
from pathlib import Path
from typing import Annotated

import typer

from ..utils.custom_logging import Verbose
from ..utils.input_output import InputOutput

app = typer.Typer(no_args_is_help=True)


@app.command(
    "intersections",
    help="Compute the intersections between edges in the outlines using DuckDB and export the edges and intersections to Parquet files.",
)
def intersections_command(
    bd_topo_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd-topo-file",
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
            "--output-edges-file",
            help="Path to the output edges parquet file.",
        ),
    ],
    output_intersections_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--output-intersections-file",
            help="Path to the output intersections parquet file.",
        ),
    ],
    output_groups_file: Annotated[
        Path,
        typer.Option(
            "-g",
            "--output-groups-file",
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
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip-existing",
            help="Whether to skip steps if output files already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    from .intersections import intersections_call

    intersections_call(
        bd_topo_file=bd_topo_file,
        output_edges_file=output_edges_file,
        output_intersections_file=output_intersections_file,
        output_building_groups_file=output_groups_file,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


@app.command(
    "crop_intersections",
    help="Crop the Parquet files corresponding to the intersections in the BD TOPO to the bounds of a given LAS/LAZ file and export the cropped data to a parquet file.",
)
def crop_intersections_command(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las-file",
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
            "--edges-file",
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
            "--intersections-file",
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
            "--groups-file",
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
            "--output-edges-file",
            help="Path to the output cropped edges parquet file.",
        ),
    ],
    output_intersections_file: Annotated[
        Path,
        typer.Option(
            "-p",
            "--output-intersections-file",
            help="Path to the output cropped intersections parquet file.",
        ),
    ],
    output_groups_file: Annotated[
        Path,
        typer.Option(
            "-q",
            "--output-groups-file",
            help="Path to the output cropped building groups parquet file.",
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
):
    from .intersections import crop_intersections_call

    crop_intersections_call(
        input_las_file=las_file,
        input_edges_file=edges_file,
        input_intersections_file=intersections_file,
        input_building_groups_file=groups_file,
        output_edges_file=output_edges_file,
        output_intersections_file=output_intersections_file,
        output_building_groups_file=output_groups_file,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


if __name__ == "__main__":
    app()
