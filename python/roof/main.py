from pathlib import Path
from typing import Annotated

import typer

from ..utils.custom_logging import Verbose
from ..utils.input_output import InputOutput

app = typer.Typer(no_args_is_help=True)


@app.command("roofprints_to_lod22")
def roofprints_to_lod22_command(
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
    roofprints_file: Annotated[
        Path,
        typer.Option(
            "-r",
            "--roofprints-file",
            help="Path to the input roofprints file.",
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
            "--output-roof-file",
            help="Path to the output roof file (Parquet).",
            exists=False,
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
    from .roof import roofprints_to_lod22_call

    roofprints_to_lod22_call(
        point_cloud_path=las_file,
        roofprints_path=roofprints_file,
        roof_path=output_file,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


if __name__ == "__main__":
    app()
