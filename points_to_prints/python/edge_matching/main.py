import copy
import logging
import random
from pathlib import Path
from typing import Annotated, Optional

import matplotlib.pyplot as plt
import tqdm
import typer

from ..utils.custom_logging import LoggingContext
from .algorithm import EdgeShiftingAlgorithm
from .constants import *
from .criterion import LinearCriterion
from .geometry import Point, Vector
from .line_mover import AllLinesMoverSimple
from .plot_recorder import PlotRecorder
from .sample_data import example_circle, generate_points_circle, generate_polygon_circle
from .topology import AllLines

app = typer.Typer()


@app.command(
    "test_circle",
    help="Test the edge shifting algorithm on a circle example.",
)
def test_circle(
    optimization_iterations: Annotated[
        int,
        typer.Option(
            "--optimization-iterations",
            "-i",
            help="Number of optimization iterations to perform",
        ),
    ] = 1,
    record_steps: Annotated[
        bool,
        typer.Option(
            "--record-steps",
            "-s",
            help="Whether to record intermediate steps of the optimization (more images will be generated)",
        ),
    ] = False,
    num_points: Annotated[int, typer.Option("--num-points", "-p")] = 100,
    num_vertices: Annotated[int, typer.Option("--num-vertices", "-P")] = 20,
    noise_points: Annotated[float, typer.Option("--noise-points", "-n")] = 0.5,
    noise_polygon: Annotated[float, typer.Option("--noise-polygon", "-N")] = 1.0,
    radius_points: Annotated[float, typer.Option("--radius-points", "-r")] = 10.0,
    radius_polygon: Annotated[float, typer.Option("--radius-polygon", "-R")] = 10.0,
    shift_polygon_center_x: Annotated[
        float, typer.Option("--shift-polygon-center-x", "-x")
    ] = 0.0,
    shift_polygon_center_y: Annotated[
        float, typer.Option("--shift-polygon-center-y", "-y")
    ] = 0.0,
    random_seed: Annotated[Optional[int], typer.Option("--random-seed", "-S")] = None,
    alpha_ratio: Annotated[float, typer.Option("--alpha-ratio", "-a")] = 0.1,
    alpha_abs: Annotated[float, typer.Option("--alpha-abs", "-A")] = 0.1,
    verbose_int: Annotated[
        int,
        typer.Option(
            "--verbose",
            "-v",
            count=True,
            help="Verbosity level (0=Error, 1=Warning, 2=Info, 3=Debug)",
        ),
    ] = 0,
) -> None:
    with LoggingContext(verbose=verbose_int):
        shift_polygon_center = Vector(shift_polygon_center_x, shift_polygon_center_y)
        points, polygon = example_circle(
            num_points=num_points,
            num_vertices=num_vertices,
            noise_points=noise_points,
            noise_polygon=noise_polygon,
            radius_points=radius_points,
            radius_polygon=radius_polygon,
            shift_polygon_center=shift_polygon_center,
            random_seed=random_seed,
        )

        # Recorders
        bounds_recorders = None

        all_lines = AllLines(
            lines=polygon.lines,
            prev_lines=[
                (i - 1) % len(polygon.lines) for i in range(len(polygon.lines))
            ],
            next_lines=[
                (i + 1) % len(polygon.lines) for i in range(len(polygon.lines))
            ],
            touching_lines=[[] for _ in polygon.lines],
        )

        recorder_iterations = PlotRecorder()
        recorder_iterations.capture(
            "initial",
            points=points,
            segments=all_lines.get_segments(),
            title="Initial state",
            bounds=bounds_recorders,
        )

        if record_steps:
            recorder_steps = PlotRecorder()
        else:
            recorder_steps = None
        if recorder_steps is not None:
            recorder_steps.capture(
                "initial",
                points=points,
                segments=all_lines.get_segments(),
                title="Initial state",
                bounds=bounds_recorders,
            )

        algo = EdgeShiftingAlgorithm(
            all_lines,
            LinearCriterion(
                points=points,
                weights=[1.0] * len(points),
                max_distance=LINEAR_CRITERION_MAX_DISTANCE,
                alpha_ratio=alpha_ratio,
                alpha_abs=alpha_abs,
                initial_perimeter=sum(
                    segment.length() for segment in all_lines.get_segments()
                ),
            ),
        )

        for step in tqdm.trange(
            1, optimization_iterations + 1, desc="Optimization steps"
        ):

            if recorder_steps is not None:
                optimization_callback = lambda i: recorder_steps.capture(
                    f"optimization_step_{step:{f'0{len(str(optimization_iterations))}d'}}_line_{i}",
                    points=points,
                    segments=all_lines.get_segments(),
                    title=f"After optimization step {step} (line {i})",
                    bounds=bounds_recorders,
                )
            else:
                optimization_callback = lambda i: None

            sum_shifts = algo.optimize_all_lines(
                callback_after_optimization=optimization_callback
            )

            recorder_iterations.capture(
                f"optimization_step_{step:{f'0{len(str(optimization_iterations))}d'}}",
                points=points,
                segments=all_lines.get_segments(),
                title=f"Step {step}, shifted by {sum_shifts:.5f}",
                bounds=bounds_recorders,
            )

            if sum_shifts < EDGE_MATCHING_OFFSET_STEP:
                logging.info(
                    f"Optimization converged after {step} steps with total shift {sum_shifts:.5f} < 1e-5."
                )
                break

        # recorder_iterations.save_all_combined(
        #     Path("images/optimization_iterations.png")
        # )
        recorder_iterations.save_combined_as_video(
            Path("images/optimization_iterations.gif"), fps=2
        )
        if recorder_steps is not None:
            recorder_steps.save_all_combined(Path("images/optimization_steps.png"))


def experiment_one_move(line_idx: int, shift: Vector) -> None:
    # Points
    center_points = Point(0, 0)
    radius_points = 10
    num_points = 100
    noise_points = 0.0

    # Polygon
    center_polygon = center_points + Vector(0, 1)
    radius_polygon = radius_points - 1
    num_vertices_polygon = 20
    noise_polygon = 0.0

    # Recorders
    bounds_recorders = (
        (center_points.x - radius_points - 2, center_points.y - radius_points - 2),
        (center_points.x + radius_points + 2, center_points.y + radius_points + 2),
    )
    bounds_recorders = None

    random.seed(1)

    points = generate_points_circle(
        center=center_points,
        radius=radius_points,
        num_points=num_points,
        noise=noise_points,
    )
    polygon = generate_polygon_circle(
        center=center_polygon,
        radius=radius_polygon,
        num_vertices=num_vertices_polygon,
        noise=noise_polygon,
    )

    all_lines = AllLines(
        lines=polygon.lines,
        prev_lines=[(i - 1) % len(polygon.lines) for i in range(len(polygon.lines))],
        next_lines=[(i + 1) % len(polygon.lines) for i in range(len(polygon.lines))],
        touching_lines=[[] for _ in polygon.lines],
    )
    all_lines_copy = copy.deepcopy(all_lines)

    recorder = PlotRecorder()
    recorder.capture(
        "initial",
        points=points,
        segments=all_lines.get_segments(),
        title="Initial state",
        bounds=bounds_recorders,
    )

    shift_direction = shift.normalized()

    def callback(line_idx: int, shifted_lines: dict[int, float]) -> None:
        for i, shift in shifted_lines.items():
            line = all_lines.get_line(i).shifted(shift_direction * shift)
            all_lines_copy.update_line(i, line)
        recorder.capture(
            f"line_{line_idx}_shifted",
            points=points,
            segments=all_lines_copy.get_segments(),
            title=f"After shifting line {line_idx}",
            bounds=bounds_recorders,
        )

    lines_mover = AllLinesMoverSimple(
        all_lines=all_lines,
        line_idx=line_idx,
        shift_direction=shift_direction,
        queried_shifts=[shift.length()],
    )

    shifted_lines = lines_mover.compute_shifted_lines(callback_line=callback)[0]

    recorder.capture(
        f"final",
        points=points,
        segments=all_lines.get_segments(),
        title=f"After optimization",
        bounds=bounds_recorders,
    )

    recorder.save_all_combined(Path("images/experiment_one_move.png"))


@app.command(
    "one_move",
    help="Run a single edge shifting experiment with specified parameters.",
)
def one_move(
    line_idx: Annotated[
        int,
        typer.Option(
            "--line-idx",
            "-l",
            help="Index of the line to shift",
        ),
    ],
    shift_x: Annotated[
        float,
        typer.Option(
            "--shift-x",
            "-x",
            help="X component of the shift vector",
        ),
    ],
    shift_y: Annotated[
        float,
        typer.Option(
            "--shift-y",
            "-y",
            help="Y component of the shift vector",
        ),
    ],
    verbose_int: Annotated[
        int,
        typer.Option(
            "--verbose",
            "-v",
            count=True,
            help="Verbosity level (0=Error, 1=Warning, 2=Info, 3=Debug)",
        ),
    ] = 0,
):
    with LoggingContext(verbose=verbose_int):
        plt.set_loglevel(level="warning")
        shift = Vector(shift_x, shift_y)
        experiment_one_move(line_idx=line_idx, shift=shift)


if __name__ == "__main__":

    app()
