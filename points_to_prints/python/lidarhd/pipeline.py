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

from ..utils.custom_logging import LoggingContext, run_command_with_tqdm_logging
from .bd_topo_crop import crop_parquet_from_las
from .bd_topo_intersections import (
    compute_export_intersections,
    crop_intersections_files,
)
from .download import download_lidar_hd_data
from .las_manipulations import (
    get_las_bounds,
    identity_convert,
    merge_files,
    split_point_cloud_implementation,
)


def _build_cpp_tool():
    """Builds the C++ program to be able to use it in the rest of the pipeline."""
    logging.info(f"Building the C++ tools...")
    command_build = ["pixi", "run", "--quiet", "cpp", "build", "release"]
    return_code = run_command_with_tqdm_logging(command_build)
    if return_code != 0:
        logging.error("C++ build failed.")
    else:
        logging.info("C++ tools built successfully.")


def _compute_inward_direction(
    input_las_path: Path, output_las_path: Path, overwrite: bool, skip_existing: bool
):
    """Compute the inward direction to prepare the computation of the roofprints.

    Args:
        input_las_path (Path): Path to the input point cloud.
        output_las_path (Path): Path to the output point cloud with the new attributes.
        overwrite (bool): _description_
        skip_existing (bool): _description_

    Raises:
        FileExistsError: _description_
    """
    logging.info("Computing the inward direction...")
    if output_las_path.exists():
        if skip_existing:
            logging.info(f"{output_las_path} already exists. Skipping.")
            return
        if overwrite:
            logging.info(f"Overwriting {output_las_path}.")
        else:
            raise FileExistsError(
                f"{output_las_path} already exists. Use --overwrite to overwrite it or --skip_existing to skip creating it."
            )

    command_inwards = [
        "pixi",
        "run",
        "cpp",
        "run-only",
        "release",
        "--",
        "add_inward_directions",
        "-i",
        str(input_las_path),
        "-o",
        str(output_las_path),
        "-t",
        "roof",
    ]

    if overwrite:
        command_inwards.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command_inwards)
    if return_code != 0:
        logging.error(f"Failed to create {output_las_path}.")
    else:
        logging.info(f"Successfully created {output_las_path}.")


def _split_source_point_cloud(
    input_las_path: Path,
    output_template_path: Path,
    overwrite: bool,
    skip_existing: bool,
) -> List[Path]:
    all_strip_files = split_point_cloud_implementation(
        input_file=input_las_path,
        output_file_template=output_template_path,
        dimension="PointSourceId",
        overwrite=overwrite,
        skip_existing=skip_existing,
    )
    return all_strip_files


def _compute_trajectory(
    input_las_path: Path,
    output_trajectory_path: Path,
    overwrite: bool,
    skip_existing: bool,
):
    logging.info(f"Computing the trajectory for {input_las_path}...")
    if output_trajectory_path.exists():
        if skip_existing:
            logging.info(f"{output_trajectory_path} already exists. Skipping.")
            return
        if overwrite:
            logging.info(f"Overwriting {output_trajectory_path}.")
        else:
            raise FileExistsError(
                f"{output_trajectory_path} already exists. Use --overwrite to overwrite it or --skip_existing to skip creating it."
            )

    command_trajectory_file_1 = input_las_path.parent / f"{input_las_path.stem}_1.txt"
    command_trajectory_file_2 = input_las_path.parent / f"{input_las_path.stem}_2.txt"

    command = ["pixi", "run", "traj-estimation", str(input_las_path)]
    return_code = run_command_with_tqdm_logging(command)
    if return_code != 0:
        logging.error(f"Failed to compute trajectory for {input_las_path.name}.")
    else:
        logging.info(f"Successfully computed trajectory for {input_las_path.name}.")

    # Check if the trajectory file was created
    if not command_trajectory_file_1.exists():
        logging.error(
            f"Could not find the expected output ({command_trajectory_file_1}) for the trajectory of {input_las_path.name}."
        )

    # Rename it to match the expected format for the next steps
    command_trajectory_file_1.rename(output_trajectory_path)

    # Check if there was a second file created
    if command_trajectory_file_2.exists():
        logging.error(
            f"Second trajectory file found for {input_las_path.name}: {command_trajectory_file_2}. This means that the initial point cloud was not split properly between the different flight axes."
        )

    # Remove the other output file if it exists, since it is not needed
    other_output_file = (
        input_las_path.parent / f"{input_las_path.stem}_center_refine.txt"
    )
    if other_output_file.exists():
        other_output_file.unlink()


def _check_file_exist(file_path: Path) -> bool:
    file_exists = file_path.exists()
    if not file_exists:
        logging.error("This file is expected to exist but it is missing.")
    return file_exists


def run_pipeline(
    other_data_dir: Path,
    tile_dir: Path,
    bd_topo_file: Path,
    overwrite: bool,
    skip_existing: bool,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    with LoggingContext(verbose=verbose_int):
        # Build the C++ tools
        _build_cpp_tool()

        # Compute the inward directions
        initial_laz_file = tile_dir / "lidarhd.copc.laz"
        source_laz_file = tile_dir / "lidarhd-with_inwards_roof.laz"
        _compute_inward_direction(
            input_las_path=initial_laz_file,
            output_las_path=source_laz_file,
            overwrite=overwrite,
            skip_existing=skip_existing,
        )

        # Split the source .laz file into multiple files based on the "PointSourceId" attribute
        all_flight_strip_files = _split_source_point_cloud(
            input_las_path=source_laz_file,
            output_template_path=tile_dir / "axis_#.laz",
            overwrite=overwrite,
            skip_existing=skip_existing,
        )
        all_flight_strip_files = sorted(all_flight_strip_files)

        # Compute the trajectory for each split file
        for flight_strip_file in all_flight_strip_files:
            # We cannot chose the path of the output with the code we use, so this cannot be changed
            final_trajectory_file = (
                flight_strip_file.parent / f"{flight_strip_file.stem}-trajectory.txt"
            )
            _compute_trajectory(
                input_las_path=flight_strip_file,
                output_trajectory_path=final_trajectory_file,
                overwrite=overwrite,
                skip_existing=skip_existing,
            )

        # Check that the intersections of the BD TOPO are available
        bd_topo_dir = other_data_dir / "bd_topo"
        edges_file = bd_topo_dir / "edges.parquet"
        intersections_file = bd_topo_dir / "intersections.parquet"
        groups_file = bd_topo_dir / "building_groups.parquet"
        fail = False
        for file_path in [edges_file, intersections_file, groups_file]:
            file_exists = _check_file_exist(file_path=file_path)
            fail = fail or not file_exists
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
        total_files = len(all_flight_strip_files)
        distances_files = []
        edges_files = []
        for index, laz_file in enumerate(all_flight_strip_files, start=1):
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
                "--",
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
            "--",
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
