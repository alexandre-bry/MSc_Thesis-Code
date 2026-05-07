from pathlib import Path
from typing import Annotated

import typer

app = typer.Typer()


@app.command("roofprints_to_roof")
def roofprints_to_roof_command(
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
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
) -> None:
    from .roof import roofprints_to_roof_call

    roofprints_to_roof_call(
        point_cloud_path=point_cloud_path,
        roofprints_path=roofprints_path,
        roof_path=output_path,
        verbose_int=verbose_int,
    )


if __name__ == "__main__":
    app()
