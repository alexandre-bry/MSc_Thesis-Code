from pathlib import Path
from typing import Annotated

import typer

app = typer.Typer()


def _format_summary(summary: dict[str, float | int], output_path: Path) -> str:
    return "\n".join(
        [
            f"Matched polygon pairs: {summary['matched_count']}",
            f"Ignored ground-truth polygons: {summary['ignored_ground_truth_count']}",
            f"Ignored scored polygons: {summary['ignored_scored_count']}",
            f"Mean IoU: {summary['mean_iou']:.6f}",
            (
                "Mean directed boundary distances (m): "
                f"GT->Scored {summary['mean_ground_truth_to_scored_boundary_distance_m']:.6f}, "
                f"Scored->GT {summary['mean_scored_to_ground_truth_boundary_distance_m']:.6f}, "
                f"Symmetric {summary['mean_symmetric_boundary_distance_m']:.6f}"
            ),
            f"Results written to: {output_path}",
        ]
    )


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
            "-f",
            help="Overwrite the output file if it exists",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
) -> None:
    from .metrics import clean_polygon_topology_call, write_polygon_topology_results

    cleaned_dataset = clean_polygon_topology_call(
        input_path=input_path,
        threshold_m=threshold_m,
        verbose_int=verbose_int,
    )
    write_polygon_topology_results(cleaned_dataset, output_path, overwrite)
    typer.echo(
        "\n".join(
            [
                f"Cleaned polygon features: {len(cleaned_dataset)}",
                f"Results written to: {output_path}",
            ]
        )
    )


@app.command("compare")
def compare_polygon_datasets_command(
    ground_truth_path: Annotated[
        Path,
        typer.Argument(
            help="Path to the ground-truth polygon dataset (.parquet or .gpkg)",
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
            help="Path where the per-pair metrics table will be written (.csv or .parquet)"
        ),
    ],
    id_column: Annotated[
        str,
        typer.Option(
            "--id-column",
            "-i",
            help="Column name used to pair polygons between datasets.",
        ),
    ] = "cleabs",
    keep_columns: Annotated[
        list[str],
        typer.Option(
            "--keep-column",
            "-k",
            help="Ground-truth columns to keep in the output. Can be used multiple times.",
        ),
    ] = [],
    aggregate_touching: Annotated[
        bool,
        typer.Option(
            "--aggregate-touching/--no-aggregate-touching",
            help=(
                "First dissolve touching polygons in the ground-truth dataset into "
                "aggregates before computing the metrics."
            ),
        ),
    ] = False,
    spacing_m: Annotated[
        float,
        typer.Option(
            "--spacing-m",
            "-d",
            help="Sampling spacing along polygon boundaries, in metres.",
        ),
    ] = 1.0,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
) -> None:
    from .metrics import compare_polygon_datasets_call, write_comparison_results

    result = compare_polygon_datasets_call(
        ground_truth_path=ground_truth_path,
        scored_path=scored_path,
        id_column=id_column,
        spacing_m=spacing_m,
        keep_columns=keep_columns,
        aggregate_touching=aggregate_touching,
        verbose_int=verbose_int,
    )
    write_comparison_results(result.paired_results, output_path)
    typer.echo(_format_summary(result.summary, output_path))


if __name__ == "__main__":
    app()
