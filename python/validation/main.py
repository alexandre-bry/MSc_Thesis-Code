from pathlib import Path
from typing import Annotated

import typer

from ..utils.custom_logging import Verbose
from ..utils.input_output import InputOutput

app = typer.Typer(no_args_is_help=True)


@app.command("clean_topology")
def clean_polygon_topology_command(
    input_path: Annotated[
        Path,
        typer.Argument(
            help="Path to the polygon dataset to clean (.parquet or .gpkg)",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_path: Annotated[
        Path,
        typer.Argument(
            help="Path where the cleaned polygon dataset will be written (.parquet or .gpkg)"
        ),
    ],
    threshold_m: Annotated[
        float,
        typer.Option(
            "--threshold-m",
            "-t",
            help="Merge threshold used to snap nearby polygon vertices, in metres.",
        ),
    ] = 1e-2,
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
    from .metrics import clean_polygon_topology_call

    clean_polygon_topology_call(
        input_path=input_path,
        output_path=output_path,
        threshold_m=threshold_m,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


@app.command("prepare_validation_dataset")
def prepare_validation_dataset_command(
    input_path: Annotated[
        Path,
        typer.Argument(
            help="Path to the raw validation ground-truth polygon dataset (.parquet or .gpkg)",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    individual_output_path: Annotated[
        Path,
        typer.Argument(
            help="Path where the individual-building validation dataset will be written (.parquet or .gpkg)"
        ),
    ],
    aggregated_output_path: Annotated[
        Path,
        typer.Argument(
            help="Path where the aggregated validation dataset will be written (.parquet or .gpkg)"
        ),
    ],
    id_column: Annotated[
        str,
        typer.Option(
            "--id-column",
            "-i",
            help="Column name containing the unique building id in the raw validation dataset.",
        ),
    ] = "cleabs",
    keep_columns: Annotated[
        list[str],
        typer.Option(
            "--keep-columns",
            "-k",
            help="Additional columns to keep in the aggregated validation dataset.",
        ),
    ] = [],
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
    from .processing import prepare_validation_dataset_call

    prepare_validation_dataset_call(
        input_ground_truth_path=input_path,
        id_column=id_column,
        individual_output_path=individual_output_path,
        aggregated_output_path=aggregated_output_path,
        keep_columns=keep_columns,
        input_output=InputOutput.from_flags(overwrite, skip_existing),
        verbose=Verbose.from_int(verbose_int),
    )


@app.command("compare")
def compare_polygon_datasets_command(
    ground_truth_path: Annotated[
        Path,
        typer.Argument(
            help="Path to the validation polygon dataset (.parquet or .gpkg)",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    scored_path: Annotated[
        Path,
        typer.Argument(
            help="Path to the scored polygon dataset (.parquet or .gpkg)",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_path: Annotated[
        Path,
        typer.Argument(
            help="Path where the per-pair metrics table will be written (.csv, .parquet or .json)"
        ),
    ],
    id_column: Annotated[
        str,
        typer.Option(
            "--id-column",
            "-i",
            help="Column name used to match scored polygons against validation source ids.",
        ),
    ] = "cleabs",
    spacing_m: Annotated[
        float,
        typer.Option(
            "--spacing-m",
            "-d",
            help="Sampling spacing along polygon boundaries, in metres.",
        ),
    ] = 1.0,
    keep_columns: Annotated[
        list[str],
        typer.Option(
            "--keep-columns",
            "-k",
            help=(
                "Additional columns from the ground-truth and scored datasets to keep in the output results."
            ),
        ),
    ] = [],
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
    from .metrics import compare_polygon_datasets_call

    compare_polygon_datasets_call(
        ground_truth_path=ground_truth_path,
        scored_path=scored_path,
        output_path=output_path,
        id_column=id_column,
        spacing_m=spacing_m,
        keep_columns=keep_columns,
        verbose=Verbose.from_int(verbose_int),
        input_output=InputOutput.from_flags(overwrite, skip_existing),
    )


if __name__ == "__main__":
    app()
