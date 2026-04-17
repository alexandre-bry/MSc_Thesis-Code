import copy
import itertools
import json
import logging
import math
import random
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Annotated, Dict, List, Optional

import matplotlib.pyplot as plt
import tqdm
import typer

from ..utils.custom_logging import LoggingContext
from .algorithm import EdgeShiftingAlgorithm
from .constants import *
from .criterion import LinearCriterion
from .geometry import Point, UnitVector, Vector
from .line_mover import AllLinesMoverSimple
from .plot_recorder import PlotRecorder
from .sample_data import example_circle, generate_points_circle, generate_polygon_circle
from .topology import AllLines

app = typer.Typer()


def _run_test_circle_task(task: tuple[str, str, dict]) -> tuple[str, str]:
    experiment_name, experiment_instance_name, merged_params = task
    _test_circle(**merged_params, use_tqdm=False)
    return experiment_name, experiment_instance_name


def _test_circle(
    output_dir: Path,
    output_prefix: Optional[str],
    optimization_iterations: int,
    record_steps: bool,
    num_points: int,
    num_vertices: int,
    noise_points: float,
    noise_polygon: float,
    radius_points: float,
    radius_polygon: float,
    shift_polygon_center_x: float,
    shift_polygon_center_y: float,
    random_seed: Optional[int],
    removed_segments: str,
    alpha_edge_difference: float,
    # alpha_abs: float,
    use_tqdm: bool = True,
) -> None:
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
        remove_segments=[int(x) for x in removed_segments.split(",") if x],
    )

    # Recorders
    bounds_recorders = None

    all_lines = AllLines(
        lines=polygon.lines,
        prev_lines=[(i - 1) % len(polygon.lines) for i in range(len(polygon.lines))],
        next_lines=[(i + 1) % len(polygon.lines) for i in range(len(polygon.lines))],
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
            alpha_edge_difference=alpha_edge_difference,
            # alpha_abs=alpha_abs,
            # initial_perimeter=sum(
            #     segment.length() for segment in all_lines.get_segments()
            # ),
        ),
    )

    for step in tqdm.trange(
        1, optimization_iterations + 1, desc="Optimization steps", disable=not use_tqdm
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
            callback_after_optimization=optimization_callback,
            use_tqdm=use_tqdm,
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
    #     output_dir / f"{output_prefix}iterations.png"
    # )
    output_dir.mkdir(parents=True, exist_ok=True)
    if output_prefix is not None:
        output_prefix = output_prefix.strip() + "-"
    else:
        output_prefix = ""

    recorder_iterations.save_combined_as_video(
        output_dir / f"{output_prefix}iterations.gif", show_axes=False, fps=2
    )
    recorder_iterations.save_first(
        output_dir / f"{output_prefix}initial.png", show_axes=False
    )
    recorder_iterations.save_last(
        output_dir / f"{output_prefix}final.png", show_axes=False
    )

    if recorder_steps is not None:
        recorder_steps.save_all_combined(output_dir / f"{output_prefix}steps.png")


@app.command(
    "test_circle",
    help="Test the edge shifting algorithm on a circle example.",
)
def test_circle(
    output_dir: Annotated[
        Path,
        typer.Option(
            "--output-dir",
            "-o",
            file_okay=False,
            dir_okay=True,
            help="Directory to save output images",
        ),
    ],
    output_prefix: Annotated[
        Optional[str],
        typer.Option(
            "--output-prefix",
            "-O",
            help="Prefix for output image filenames",
        ),
    ] = None,
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
    removed_segments: Annotated[str, typer.Option("--removed-segments", "-m")] = "",
    alpha_edge_difference: Annotated[
        float, typer.Option("--alpha-edge-difference", "-a")
    ] = 0.1,
    # alpha_abs: Annotated[float, typer.Option("--alpha-abs", "-A")] = 0.1,
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
        _test_circle(
            output_dir=output_dir,
            output_prefix=output_prefix,
            optimization_iterations=optimization_iterations,
            record_steps=record_steps,
            num_points=num_points,
            num_vertices=num_vertices,
            noise_points=noise_points,
            noise_polygon=noise_polygon,
            radius_points=radius_points,
            radius_polygon=radius_polygon,
            shift_polygon_center_x=shift_polygon_center_x,
            shift_polygon_center_y=shift_polygon_center_y,
            random_seed=random_seed,
            removed_segments=removed_segments,
            alpha_edge_difference=alpha_edge_difference,
            # alpha_abs=alpha_abs,
        )


def experiment_one_move(
    line_idx: int,
    final_shift_magnitude: float,
    steps: int,
    num_vertices: int,
    noise: float,
    fps: int,
    output_path: Path,
    seed: Optional[int] = None,
) -> None:
    if seed is not None:
        random.seed(seed)

    # Polygon
    center_polygon = Point(0.0, 0.0)
    radius_polygon = 10.0

    # Recorders
    # bounds_recorders = (
    #     (center_points.x - radius_points - 2, center_points.y - radius_points - 2),
    #     (center_points.x + radius_points + 2, center_points.y + radius_points + 2),
    # )
    # bounds_recorders = None

    polygon = generate_polygon_circle(
        center=center_polygon,
        radius=radius_polygon,
        num_vertices=num_vertices,
        noise=noise,
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
        # points=points,
        segments=all_lines.get_segments(),
        title="Initial state",
        # bounds=bounds_recorders,
    )

    shift_direction = all_lines.get_line(line_idx).normal_vector
    if final_shift_magnitude < 0:
        shift_direction = shift_direction.flipped()
        final_shift_magnitude = abs(final_shift_magnitude)

    shift_magnitudes = [
        final_shift_magnitude * (step + 1) / steps for step in range(steps)
    ]
    minimum_precision = -math.floor(math.log10(final_shift_magnitude / steps))

    lines_mover = AllLinesMoverSimple(
        all_lines=all_lines,
        line_idx=line_idx,
        shift_direction=shift_direction,
        queried_shifts=shift_magnitudes,
    )

    shifted_lines_all = lines_mover.compute_shifted_lines()

    for magnitude, shifted_lines in zip(shift_magnitudes, shifted_lines_all[0]):
        for i, line in shifted_lines.items():
            # line = all_lines.get_line(i).shifted(shift_direction * shift)
            all_lines_copy.update_line(i, line)
        recorder.capture(
            f"line_{line_idx}_shifted_by_{magnitude:.{minimum_precision}f}",
            # points=points,
            segments=all_lines_copy.get_segments(),
            title=f"After shifting line {line_idx} by {magnitude:.{minimum_precision}f}",
            # bounds=bounds_recorders,
        )

    recorder.save_combined_as_video(output_path, fps=fps, show_axes=False)


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
    final_shift_magnitude: Annotated[
        float,
        typer.Option(
            "--final-shift-magnitude",
            "-f",
            help="Final magnitude of the shift to apply to the line",
        ),
    ],
    steps: Annotated[
        int,
        typer.Option(
            "--steps",
            "-s",
            help="Number of steps to divide the shift into (more steps will generate more intermediate images)",
        ),
    ],
    output_path: Annotated[
        Path, typer.Option("--output-path", "-o", help="Path to save the output video")
    ],
    num_vertices: Annotated[int, typer.Option("--num-vertices", "-P")] = 20,
    noise: Annotated[float, typer.Option("--noise", "-n")] = 0.5,
    seed: Annotated[Optional[int], typer.Option("--seed", "-S")] = None,
    fps: Annotated[
        int, typer.Option("--fps", help="Frames per second for the output video")
    ] = 20,
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
        experiment_one_move(
            line_idx=line_idx,
            final_shift_magnitude=final_shift_magnitude,
            steps=steps,
            num_vertices=num_vertices,
            noise=noise,
            seed=seed,
            output_path=output_path,
            fps=fps,
        )


@app.command(
    "run_experiments",
    help="Run a series of predefined experiments to analyze the behavior of the edge shifting algorithm.",
)
def run_experiments(
    experiments_file: Annotated[
        Path,
        typer.Option(
            "--experiments-file",
            "-e",
            exists=True,
            file_okay=True,
            dir_okay=False,
            help="Path to a JSON file containing experiment configurations",
        ),
    ],
    output_dir: Annotated[
        Path,
        typer.Option(
            "--output-dir",
            "-o",
            file_okay=False,
            dir_okay=True,
            help="Directory to save output images",
        ),
    ],
    selected_experiment: Annotated[
        Optional[str],
        typer.Option(
            "--selected-experiment",
            "-s",
            help="Name of the specific experiment to run (hierarchical path like 'circle/circle')",
        ),
    ] = None,
    max_workers: Annotated[
        Optional[int],
        typer.Option(
            "--max-workers",
            help="Maximum number of multiprocessing workers (defaults to Python executor default).",
        ),
    ] = None,
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite existing output images for experiments (if --skip-existing is not set)",
        ),
    ] = False,
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

        with open(experiments_file, "r") as f:
            specifications = json.load(f)

        # Extract top-level parameters
        global_fixed = specifications.get("fixed", {})
        variable_spec: List[List[Dict]] = specifications.get("variable", [])
        specific_groups = specifications.get("specific", {})

        # Validate max_workers
        if max_workers is not None and max_workers < 1:
            raise typer.BadParameter("--max-workers must be >= 1.")

        # Create cartesian product of variable parameters
        if not variable_spec:
            # No variables, just one empty combination
            variable_combinations = [{}]
        else:
            # Build cartesian product of all variable lists
            variable_combinations = []
            for var_list in variable_spec:
                if not variable_combinations:
                    variable_combinations = [{}]
                new_combinations = []
                for var_dict in var_list:
                    for combo in variable_combinations:
                        new_combo = {**combo, **var_dict}
                        new_combinations.append(new_combo)
                variable_combinations = new_combinations

        # Build tasks from the recursively nested specific structure
        tasks: list[tuple[str, str, dict]] = []
        skipped_experiments: list[str] = []

        def process_specifics(
            specifics: dict, inherited_fixed: dict, path_parts: list
        ) -> None:
            """Recursively process nested specific structure to extract experiments."""
            for group_name, group_config in specifics.items():
                current_path = path_parts + [group_name]
                current_path_str = "/".join(current_path)

                # Skip if a specific experiment is selected and doesn't match
                if (
                    selected_experiment is not None
                    and current_path_str != selected_experiment
                ):
                    continue

                # Merge fixed parameters from this level
                group_fixed = group_config.get("fixed", {})
                merged_fixed = {**inherited_fixed, **group_fixed}

                nested_specific = group_config.get("specific")

                if nested_specific:
                    # Intermediate node: recurse deeper
                    process_specifics(nested_specific, merged_fixed, current_path)
                else:
                    # Leaf node: create experiment instances for all variable combinations
                    for var_combo in variable_combinations:
                        merged_params = {**merged_fixed, **var_combo}

                        if "output_dir" not in merged_params:
                            raise ValueError(
                                f"Experiment '{current_path_str}' is missing 'output_dir' parameter."
                            )

                        merged_params["output_dir"] = (
                            output_dir / merged_params["output_dir"]
                        )

                        if not overwrite and merged_params["output_dir"].exists():
                            skipped_experiments.append(current_path_str)
                            continue

                        # Create instance name from variable parameters
                        var_parts = [f"{k}={v}" for k, v in sorted(var_combo.items())]
                        instance_name = "_".join(var_parts) if var_parts else "default"

                        tasks.append((current_path_str, instance_name, merged_params))

        # Process all specific groups with global_fixed as base
        process_specifics(specific_groups, global_fixed, [])

        if len(skipped_experiments) > 0:
            logging.warning(
                f"Skipping {len(skipped_experiments)} experiment(s) because their output directories already exist and --overwrite is not set: {', '.join(set(skipped_experiments))}"
            )

        if not tasks:
            logging.warning("No experiment instances found.")
            return

        logging.info(
            f"Running {len(tasks)} experiment instances with multiprocessing (max_workers={max_workers})."
        )

        failed_tasks: list[tuple[str, str, Exception]] = []
        with ProcessPoolExecutor(max_workers=max_workers) as executor:
            future_to_task = {
                executor.submit(_run_test_circle_task, task): task for task in tasks
            }
            for future in tqdm.tqdm(
                as_completed(future_to_task),
                total=len(tasks),
                desc="Running experiments",
            ):
                experiment_name, experiment_instance_name, _ = future_to_task[future]
                try:
                    finished_experiment, finished_instance = future.result()
                    # logging.info(
                    #     f"Finished experiment '{finished_experiment}' instance '{finished_instance}'."
                    # )
                except Exception as exc:
                    failed_tasks.append(
                        (experiment_name, experiment_instance_name, exc)
                    )
                    logging.error(
                        f"Experiment '{experiment_name}' instance '{experiment_instance_name}' failed."
                    )

        if failed_tasks:
            failed_names = ", ".join(
                f"{experiment_name}:{instance_name}"
                for experiment_name, instance_name, _ in failed_tasks
            )
            raise RuntimeError(
                f"{len(failed_tasks)} experiment instance(s) failed: {failed_names}"
            )


if __name__ == "__main__":
    app()
