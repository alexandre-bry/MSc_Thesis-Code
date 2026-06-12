import logging
from pathlib import Path
from typing import Annotated

import typer

from ..utils.custom_logging import Verbose
from ..utils.input_output import InputOutput

app = typer.Typer(no_args_is_help=True)


@app.command(
    "convert",
    help="Convert the BD TOPO data from its original format to a parquet file with a geometry column in WKB format.",
)
def convert_bd_topo_command(
    input_path: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input_path",
            help="Path to the input BD TOPO file containing all the layers including the buildings layer (e.g., a .gpkg file).",
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
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip steps if output files already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    from .convert import convert_bd_topo_call

    convert_bd_topo_call(
        input_path=input_path,
        output_path=output_path,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


if __name__ == "__main__":
    app()
