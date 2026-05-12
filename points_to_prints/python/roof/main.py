from pathlib import Path
from typing import Annotated

import typer

app = typer.Typer()


@app.command("roofprints_to_lod22")
def roofprints_to_lod22_command(
    point_cloud_path: Annotated[
        Path, typer.Argument(help="Path to the point cloud file (LAS/LAZ)")
    ],
    roofprints_path: Annotated[
        Path, typer.Argument(help="Path to the roofprints file")
    ],
    output_path: Annotated[
        Path,
        typer.Argument(help="Path where the resulting 3D roof model will be saved"),
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
        point_cloud_path=point_cloud_path,
        roofprints_path=roofprints_path,
        roof_path=output_path,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
    )


@app.command("lod22_to_roof")
def lod22_to_roof_command(
    lod22_path: Annotated[
        Path, typer.Argument(help="Path to the input LOD2.2 CityJSON file")
    ],
    output_path: Annotated[
        Path,
        typer.Argument(
            help="Path where the resulting roof-only CityJSON file will be saved"
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
    from .roof import lod22_to_roof_call

    lod22_to_roof_call(
        lod22_path=lod22_path,
        roof_path=output_path,
        overwrite=overwrite,
        skip_existing=skip_existing,
        verbose_int=verbose_int,
    )


if __name__ == "__main__":
    app()
