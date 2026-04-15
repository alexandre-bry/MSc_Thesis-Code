import asyncio
import logging
import os
import pty
import re
import subprocess
import sys
import threading
from enum import Enum
from pathlib import Path
from typing import Annotated, List

import typer

from ..utils.custom_logging import LoggingContext
from .bd_topo_crop import crop_parquet_from_las
from .bd_topo_intersections import (
    compute_export_intersections,
    crop_intersections_files,
)
from .download import download_lidar_hd_data
from .las_manipulations import get_las_bounds, identity_convert, merge_files, split_file

app = typer.Typer()


def run_command_with_tqdm_logging(command: list[str]) -> int:
    env = os.environ.copy()
    env.setdefault("PY_COLORS", "1")
    env.setdefault("CLICOLOR_FORCE", "1")
    env.setdefault("FORCE_COLOR", "1")
    env.setdefault("TERM", "xterm-256color")

    if os.name == "posix":
        master_fd, slave_fd = pty.openpty()
        try:
            process = subprocess.Popen(
                command,
                stdout=slave_fd,
                stderr=slave_fd,
                text=False,
                env=env,
            )
        finally:
            os.close(slave_fd)

        try:
            while True:
                try:
                    chunk = os.read(master_fd, 4096)
                except OSError:
                    break

                if not chunk:
                    break

                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
        finally:
            os.close(master_fd)

        return process.wait()

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=False,
        bufsize=0,
        env=env,
    )

    def _forward_stream(stream, target_buffer):
        if stream is None:
            return

        try:
            while True:
                chunk = stream.read(4096)
                if not chunk:
                    break
                target_buffer.write(chunk)
                target_buffer.flush()
        finally:
            stream.close()

    stdout_thread = threading.Thread(
        target=_forward_stream, args=(process.stdout, sys.stdout.buffer), daemon=True
    )
    stderr_thread = threading.Thread(
        target=_forward_stream,
        args=(process.stderr, sys.stderr.buffer),
        daemon=True,
    )

    stdout_thread.start()
    stderr_thread.start()

    return_code = process.wait()
    stdout_thread.join()
    stderr_thread.join()
    return return_code


@app.command(
    "split_las",
    help="Split a .laz file into one file for each value of a specified dimension.",
)
def split_las(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the .laz file.",
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
            "--output_file_template",
            help="Template for the output .laz files. Use # as a placeholder for the dimension value.",
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
    # use_value_in_filename: Annotated[
    #     bool,
    #     typer.Option(
    #         "-n",
    #         "--name_with_value",
    #         help="Whether to include the dimension value in the output filename.",
    #     ),
    # ] = False,
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
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
):
    with LoggingContext(verbose=verbose_int):
        output_file_template.parent.mkdir(parents=True, exist_ok=True)

        all_output_files = split_file(
            input_file=input_file,
            output_file_template=output_file_template,
            dimension=dimension,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )

    return all_output_files


@app.command("merge_las", help="Merge multiple .laz files into a single .laz file.")
def merge_las(
    input_files: Annotated[
        List[Path],
        typer.Option(
            "-i",
            "--input",
            help="Paths to the .laz files. Can be used multiple times.",
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
            "--output_file",
            help="Path to the output .laz file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
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
):
    with LoggingContext(verbose=verbose_int):
        output_file.parent.mkdir(parents=True, exist_ok=True)

        merge_files(
            input_files=input_files,
            output_file=output_file,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )
        logging.info(f"Successfully merged files into {output_file}")


@app.command(
    "download_lidar_hd", help="Download LiDAR HD data for a specified bounding box."
)
def download_lidar_hd(
    bbox: Annotated[
        str,
        typer.Option(
            "-b",
            "--bbox",
            help="Bounding box to download data for, in the format 'xmin,ymin,xmax,ymax'.",
        ),
    ],
    output_path_template: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_path",
            help="Path to save the downloaded files. The path can contain the values {xmin}, {ymin}, {xmax}, {ymax}, {file_name} which will be replaced with the corresponding values. The values also have their kilometre equivalents {xmin_km}, {ymin_km}, {xmax_km}, {ymax_km}.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        bbox_values = bbox.split(",")
        if len(bbox_values) != 4:
            raise ValueError(
                "Bounding box must be in the format 'xmin,ymin,xmax,ymax'."
            )
        xmin, ymin, xmax, ymax = map(int, bbox_values)

        asyncio.run(
            download_lidar_hd_data(
                xmin=xmin,
                ymin=ymin,
                xmax=xmax,
                ymax=ymax,
                output_path_template=output_path_template,
                overwrite=overwrite,
            )
        )


@app.command(
    "run_pipeline",
    help="Run the pipeline.",
)
def run_pipeline(
    other_data_dir: Annotated[
        Path,
        typer.Option(
            "-d",
            "--other_data_dir",
            help="Directory containing the other data files (BD TOPO parquet file, etc.) needed for the pipeline.",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    tile_dir: Annotated[
        Path,
        typer.Option(
            "-t",
            "--tile_dir",
            help="Directory containing the downloaded tile .laz files.",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    bd_topo_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo_file",
            help="Path to the BD_TOPO parquet file.",
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
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    skip_existing: Annotated[
        bool,
        typer.Option(
            "--skip_existing",
            help="Whether to skip processing files that already have output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        initial_laz_file = tile_dir / "lidarhd.copc.laz"

        # Build the C++ tools using pixi
        command_build = ["pixi", "run", "--quiet", "cpp", "build", "release"]
        logging.info(f"Building the C++ tools: {' '.join(command_build)}")
        return_code = run_command_with_tqdm_logging(command_build)
        if return_code != 0:
            logging.error("C++ build failed.")
        else:
            logging.info("C++ tools built successfully.")

        # Add the inward direction information to the original .laz file
        source_laz_file = tile_dir / "lidarhd-with_inwards.laz"
        if source_laz_file.exists() and skip_existing:
            logging.info(
                f"{source_laz_file} already exists. Skipping creation of source .laz file with inward directions due to --skip_existing flag."
            )
        elif source_laz_file.exists() and not overwrite:
            raise FileExistsError(
                f"{source_laz_file} already exists. Use --overwrite to overwrite it or --skip_existing to skip creating it."
            )
        else:
            command_inwards = [
                "pixi",
                "run",
                "cpp",
                "run-only",
                "release",
                "add_inward_directions",
                "-i",
                str(initial_laz_file),
                "-o",
                str(source_laz_file),
            ]

            if overwrite:
                command_inwards.append("--overwrite")

            logging.debug(
                f"Creating {source_laz_file} with command: {' '.join(command_inwards)}"
            )
            return_code = run_command_with_tqdm_logging(command_inwards)
            logging.debug(f"Return code for {source_laz_file}: {return_code}")
            if return_code != 0:
                logging.error(f"Failed to create {source_laz_file}.")
            else:
                logging.info(f"Successfully created {source_laz_file}.")

        # Split the source .laz file into multiple files based on the "PointSourceId" attribute
        all_strip_files = split_las(
            input_file=source_laz_file,
            output_file_template=tile_dir / "axis_#.laz",
            dimension="PointSourceId",
            overwrite=overwrite,
            skip_existing=skip_existing,
            verbose_int=verbose_int,
        )
        all_strip_files = sorted(all_strip_files)

        # Compute the trajectory for each split file
        for strip_file in all_strip_files:
            final_trajectory_file = (
                strip_file.parent / f"{strip_file.stem}-trajectory.txt"
            )
            if final_trajectory_file.exists() and skip_existing:
                logging.info(
                    f"Trajectory file already exists for {strip_file.name}: {final_trajectory_file}. Skipping trajectory computation due to --skip_existing flag."
                )
                continue
            other_output_file = (
                strip_file.parent / f"{strip_file.stem}_center_refine.txt"
            )
            command_trajectory_file_1 = strip_file.parent / f"{strip_file.stem}_1.txt"
            command_trajectory_file_2 = strip_file.parent / f"{strip_file.stem}_2.txt"
            command = ["pixi", "run", "traj-estimation", str(strip_file)]
            logging.debug(
                f"Computing trajectory for {strip_file.name} with command:\n{' '.join(command)}"
            )
            return_code = run_command_with_tqdm_logging(command)
            logging.debug(
                f"Return code for trajectory computation of {strip_file.name}: {return_code}"
            )
            if return_code != 0:
                logging.error(f"Failed to compute trajectory for {strip_file.name}.")
            else:
                logging.info(f"Successfully computed trajectory for {strip_file.name}.")

            # Check if the trajectory file was created
            if not command_trajectory_file_1.exists():
                logging.error(
                    f"Trajectory file not found for {strip_file.name}: {command_trajectory_file_1}"
                )

            # Rename it to match the expected format for the next steps
            command_trajectory_file_1.rename(final_trajectory_file)

            # Check if there was a second file created
            if command_trajectory_file_2.exists():
                logging.error(
                    f"Second trajectory file found for {strip_file.name}: {command_trajectory_file_2}. This may indicate an issue with the trajectory computation."
                )

            # Remove the other output file if it exists, since it is not needed
            if other_output_file.exists():
                other_output_file.unlink()

        # Check that the intersections of the BD TOPO are available
        bd_topo_dir = other_data_dir / "bd_topo"
        edges_file = bd_topo_dir / "edges.parquet"
        intersections_file = bd_topo_dir / "intersections.parquet"
        groups_file = bd_topo_dir / "building_groups.parquet"
        fail = False
        for file in [edges_file, intersections_file, groups_file]:
            if not file.exists():
                logging.error(f"Required BD TOPO file not found: {file}")
                fail = True
            else:
                logging.info(f"Found required BD TOPO file: {file}")
        if fail:
            raise FileNotFoundError(
                "One or more required BD TOPO files are missing. Please ensure they are present in the specified directory."
            )

        # Crop the BD TOPO data to the bounds of the source .laz file
        cropped_edges_file = tile_dir / "bd_topo-edges.parquet"
        cropped_intersections_file = tile_dir / "bd_topo-intersections.parquet"
        cropped_groups_file = tile_dir / "bd_topo-building_groups.parquet"
        bounding_box = get_las_bounds(source_laz_file)
        crop_intersections_files(
            input_edges_file=edges_file,
            input_intersections_file=intersections_file,
            input_building_groups_file=groups_file,
            output_edges_file=cropped_edges_file,
            output_intersections_file=cropped_intersections_file,
            output_building_groups_file=cropped_groups_file,
            bounding_box=bounding_box,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )

        # Process each .laz file with the C++ pipeline
        total_files = len(all_strip_files)
        distances_files = []
        edges_files = []
        for index, laz_file in enumerate(all_strip_files, start=1):
            logging.info(
                f"\n\nO----- [{index}/{total_files}] Processing tile: {laz_file.name} -----O\n"
            )
            distance_file = tile_dir / f"{laz_file.stem}-distances.laz"
            edge_file = tile_dir / f"{laz_file.stem}-edges.laz"
            distances_files.append(distance_file)
            edges_files.append(edge_file)

            if distance_file.exists() and edge_file.exists():
                if skip_existing:
                    logging.info(
                        f"Output files for {laz_file.name} already exist. Skipping processing."
                    )
                    continue
                elif not overwrite:
                    raise FileExistsError(
                        f"Output files for {laz_file.name} already exist. Use --overwrite to overwrite them or --skip_existing to skip processing."
                    )

            command = [
                "pixi",
                "run",
                "--quiet",
                "cpp",
                "run-only",
                "release",
                "distances_in_order",
                "-i",
                str(laz_file),
                "-t",
                str(tile_dir / f"{laz_file.stem}-trajectory.txt"),
                "-d",
                str(distance_file),
                "-e",
                str(edge_file),
            ]

            if overwrite:
                command.append("--overwrite")

            logging.debug(
                f"Processing {laz_file.name} with command: {' '.join(command)}"
            )
            return_code = run_command_with_tqdm_logging(command)
            logging.debug(f"Return code for {laz_file.name}: {return_code}")
            if return_code != 0:
                logging.error(f"Failed to process {laz_file.name}.")
            else:
                logging.info(f"Successfully processed {laz_file.name}.")

    # After processing all files, merge the distances and edges files into single files
    logging.info("\n\nO----- Merging files -----O\n")

    merged_distances_file = tile_dir / "merged_distances.laz"
    merged_edges_file = tile_dir / "merged_edges.laz"

    merge_files(
        input_files=distances_files,
        output_file=merged_distances_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )
    merge_files(
        input_files=edges_files,
        output_file=merged_edges_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    # Exit here to skip the roofprint computation for now, since it is not working correctly yet
    exit(0)

    # Then, compute the roofprints using the merged edges
    logging.info("\n\nO----- Computing roofprints -----O\n")
    roofprints_file = tile_dir / "roofprints.parquet"
    if roofprints_file.exists() and not overwrite:
        logging.info(
            f"Roofprints file already exists: {roofprints_file}. Skipping roofprint computation."
        )
    else:
        command = [
            "pixi",
            "run",
            "--quiet",
            "cpp",
            "run-only",
            "release",
            "compute_roofprints",
            "-i",
            str(merged_edges_file),
            "-b",
            str(bd_topo_file),
            "-o",
            roofprints_file,
        ]

        if overwrite:
            command.append("--overwrite")

        logging.debug(
            f"Running roofprint computation with command: {' '.join(command)}"
        )
        return_code = run_command_with_tqdm_logging(command)
        if return_code != 0:
            logging.error("Failed to compute roofprints.")
        else:
            logging.info("Successfully computed roofprints.")

    logging.info("\n\nPipeline completed.")


@app.command(
    "bd_topo_intersections",
    help="Compute the intersections between edges in the BD TOPO and export the edges and intersections to parquet files.",
)
def bd_topo_intersections(
    bd_topo_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo_file",
            help="Path to the BD_TOPO parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_edges_file: Annotated[
        Path,
        typer.Option(
            "-e",
            "--output_edges_file",
            help="Path to the output edges parquet file.",
        ),
    ],
    output_intersections_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--output_intersections_file",
            help="Path to the output intersections parquet file.",
        ),
    ],
    output_groups_file: Annotated[
        Path,
        typer.Option(
            "-g",
            "--output_groups_file",
            help="Path to the output building groups parquet file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        compute_export_intersections(
            bd_topo_file=bd_topo_file,
            output_edges_file=output_edges_file,
            output_intersections_file=output_intersections_file,
            output_building_groups_file=output_groups_file,
            overwrite=overwrite,
        )


@app.command(
    "bd_topo_crop_intersections",
    help="Crop the BD TOPO data to the bounds of a given LAS file and export the cropped data to a parquet file.",
)
def bd_topo_crop_intersections(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
            help="Path to the input LAS file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    edges_file: Annotated[
        Path,
        typer.Option(
            "-e",
            "--edges_file",
            help="Path to the edges parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    intersections_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--intersections_file",
            help="Path to the intersections parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    groups_file: Annotated[
        Path,
        typer.Option(
            "-g",
            "--groups_file",
            help="Path to the building groups parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_edges_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_edges_file",
            help="Path to the output cropped edges parquet file.",
        ),
    ],
    output_intersections_file: Annotated[
        Path,
        typer.Option(
            "-p",
            "--output_intersections_file",
            help="Path to the output cropped intersections parquet file.",
        ),
    ],
    output_groups_file: Annotated[
        Path,
        typer.Option(
            "-q",
            "--output_groups_file",
            help="Path to the output cropped building groups parquet file.",
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
):
    with LoggingContext(verbose=verbose_int):
        bounding_box = get_las_bounds(las_file)
        crop_intersections_files(
            input_edges_file=edges_file,
            input_intersections_file=intersections_file,
            input_building_groups_file=groups_file,
            output_edges_file=output_edges_file,
            output_intersections_file=output_intersections_file,
            output_building_groups_file=output_groups_file,
            bounding_box=bounding_box,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )


@app.command(
    "identity",
    help="A simple command to test the pipeline by copying an input point cloud to an output file without any modifications.",
)
def identity(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the input point cloud file.",
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
            "--output_file",
            help="Path to the output point cloud file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        identity_convert(
            input_file=input_file, output_file=output_file, overwrite=overwrite
        )


@app.command(
    "bd_topo_crop_from_las",
    help="Crop the BD TOPO data to the bounds of a given LAS file and export the cropped data to a parquet file.",
)
def bd_topo_crop_from_las(
    las_file: Annotated[
        Path,
        typer.Option(
            "-l",
            "--las_file",
            help="Path to the input LAS file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    input_parquet_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input_parquet_file",
            help="Path to the input parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_parquet_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_parquet_file",
            help="Path to the output cropped parquet file.",
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
):
    with LoggingContext(verbose=verbose_int):
        crop_parquet_from_las(
            input_las_file=las_file,
            input_parquet_file=input_parquet_file,
            output_parquet_file=output_parquet_file,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )


@app.command(
    "test_logging",
    help="A simple command to test the logging setup with different verbosity levels.",
)
def test(verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0):
    with LoggingContext(verbose=verbose_int):
        logging.debug("This is a debug message.")
        logging.info("This is an info message.")
        logging.warning("This is a warning message.")
        logging.error("This is an error message.")


if __name__ == "__main__":
    app()
