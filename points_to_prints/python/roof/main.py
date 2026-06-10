from pathlib import Path
from typing import Annotated

import typer

app = typer.Typer(no_args_is_help=True)


@app.command("roofprints_to_lod22")
def roofprints_to_lod22_command(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
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
            "--roofprints_file",
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
            "--output_roof_file",
            help="Path to the output roof file (Parquet).",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip processing files that already exist.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
) -> None:
    from .roof import roofprints_to_lod22_call

    roofprints_to_lod22_call(
        point_cloud_path=las_file,
        roofprints_path=roofprints_file,
        roof_path=output_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
    )


if __name__ == "__main__":
    app()
