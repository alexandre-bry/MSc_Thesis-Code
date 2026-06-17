import asyncio
import logging
from pathlib import Path
from typing import Annotated, List, Optional

import typer

from ..utils.custom_logging import LoggingContext
from ..utils.input_output import InputOutput
from .las_manipulations import (
    classification_mapping_call,
    merge_files,
    split_point_cloud_call,
)

app = typer.Typer(no_args_is_help=True)


@app.command(
    "split_las",
    help="Split a LAS/LAZ file into one file for each value of a specified dimension.",
)
def split_point_cloud_command(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the input LAS/LAZ file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file_template: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output-file-template",
            help="Template for the output LAS/LAZ files. Use # as a placeholder for the dimension value.",
        ),
    ],
    dimension: Annotated[
        str,
        typer.Option(
            "-d",
            "--dimension",
            help="The dimension to split by (e.g., 'Classification').",
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
    split_point_cloud_call(
        input_file=input_file,
        output_file_template=output_file_template,
        dimension=dimension,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose_int=verbose_int,
    )


@app.command(
    "merge_las", help="Merge multiple LAS/LAZ files into a single LAS/LAZ file."
)
def merge_las(
    input_files: Annotated[
        List[Path],
        typer.Option(
            "-i",
            "--input",
            help="Paths to the LAS/LAZ files. Can be used multiple times.",
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
            "--output-file",
            help="Path to the output LAS/LAZ file.",
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
    with LoggingContext(verbose=verbose_int):
        output_file.parent.mkdir(parents=True, exist_ok=True)

        merge_files(
            input_files=input_files,
            output_file=output_file,
            input_output=InputOutput.from_flags(overwrite, skip_existing),
        )
        logging.info(f"Successfully merged files into {output_file}")


@app.command(
    "classification_mapping",
    help="Map the classification values in a LAS/LAZ file to new values based on a provided mapping.",
)
def classification_mapping(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the input LAS/LAZ file.",
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
            "--output",
            help="Path to the output LAS/LAZ file.",
        ),
    ],
    mapping: Annotated[
        str,
        typer.Option(
            "-m",
            "--mapping",
            help="Mapping of old classification values to new values, in the format 'old1:new1,old2:new2,...'.",
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
    with LoggingContext(verbose=verbose_int):
        mapping_dict = {}
        for pair in mapping.split(","):
            old, new = pair.split(":")
            mapping_dict[int(old)] = int(new)
        classification_mapping_call(
            input_file=input_file,
            output_file=output_file,
            mapping=mapping_dict,
            input_output=InputOutput.from_flags(overwrite, skip_existing),
            verbose_int=verbose_int,
        )


if __name__ == "__main__":
    app()
