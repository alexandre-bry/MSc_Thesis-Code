import logging
import os
import pty
import re
import subprocess
import sys
import threading
from enum import Enum
from multiprocessing import Pool
from pathlib import Path
from typing import Annotated, List, Optional, Tuple

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


def _compute_single_trajectory(args: Tuple[Path, Path, bool, bool]) -> None:
    """Helper function for multiprocessing trajectory computation."""
    input_las_path, output_trajectory_path, overwrite, skip_existing = args
    _compute_trajectory(
        input_las_path=input_las_path,
        output_trajectory_path=output_trajectory_path,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )


def _process_bd_topo_data(
    initial_laz_file: Path,
    input_edges_file: Path,
    input_intersections_file: Path,
    input_building_groups_file: Path,
    output_edges_file: Path,
    output_intersections_file: Path,
    output_building_groups_file: Path,
    overwrite: bool,
    skip_existing: bool,
) -> None:
    """Process and crop BD TOPO data to match the LiDAR HD bounds.

    Args:
        other_data_dir: Directory containing BD TOPO data
        tile_dir: Output directory for cropped BD TOPO files
        initial_laz_file: LiDAR HD LAZ file to match bounds
        overwrite: Whether to overwrite existing files
        skip_existing: Whether to skip if files exist

    Returns:
        Tuple of (edges_file, intersections_file, groups_file) paths
    """
    logging.info("\n\nProcessing BD TOPO data...")

    # Check that the intersections of the BD TOPO are available
    fail = False
    for file_path in [
        input_edges_file,
        input_intersections_file,
        input_building_groups_file,
    ]:
        file_exists = _check_file_exist(file_path=file_path)
        fail = fail or not file_exists
    if fail:
        raise FileNotFoundError(
            "One or more required BD TOPO files are missing. Please ensure they are present in the specified directory."
        )

    # Crop the BD TOPO data to the bounds of the source .laz file
    bounding_box = get_las_bounds(initial_laz_file)
    crop_intersections_files(
        input_edges_file=input_edges_file,
        input_intersections_file=input_intersections_file,
        input_building_groups_file=input_building_groups_file,
        output_edges_file=output_edges_file,
        output_intersections_file=output_intersections_file,
        output_building_groups_file=output_building_groups_file,
        bounding_box=bounding_box,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )


def _process_lidar_hd_data(
    initial_laz_file: Path,
    laz_with_inwards_roof_file: Path,
    template_flight_strip_file: Path,
    overwrite: bool,
    skip_existing: bool,
) -> List[Path]:
    """Process LiDAR HD data: compute inward directions and split by flight strips.

    Args:
        initial_laz_file: Input LiDAR HD LAZ file
        laz_with_inwards_roof_file: Output file for LAZ with inward directions
        template_flight_strip_file: Template file for flight strip splitting
        overwrite: Whether to overwrite existing files
        skip_existing: Whether to skip if files exist

    Returns:
        List of sorted flight strip LAZ file paths
    """
    logging.info("Processing LiDAR HD data...")

    # Compute the inward directions
    _compute_inward_direction(
        input_las_path=initial_laz_file,
        output_las_path=laz_with_inwards_roof_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    # Split the source .laz file into multiple files based on the "PointSourceId" attribute
    all_flight_strip_files = _split_source_point_cloud(
        input_las_path=laz_with_inwards_roof_file,
        output_template_path=template_flight_strip_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    return sorted(all_flight_strip_files)


def _compute_trajectories_parallel(
    flight_strip_files: List[Path],
    trajectory_files: List[Path],
    overwrite: bool,
    skip_existing: bool,
    num_workers: Optional[int] = None,
) -> None:
    """Compute trajectories for all flight strips in parallel.

    Args:
        flight_strip_files: List of flight strip LAZ file paths
        trajectory_files: List of trajectory file paths
        overwrite: Whether to overwrite existing files
        skip_existing: Whether to skip if files exist
        num_workers: Number of worker processes (None = CPU count)
    """
    logging.info(
        f"Computing trajectories for {len(flight_strip_files)} flight strips in parallel..."
    )

    # Prepare arguments for multiprocessing
    trajectory_args = [
        (
            laz_file,
            trajectory_file,
            overwrite,
            skip_existing,
        )
        for laz_file, trajectory_file in zip(flight_strip_files, trajectory_files)
    ]

    # Use multiprocessing pool to compute trajectories in parallel
    with Pool(processes=num_workers) as pool:
        pool.map(_compute_single_trajectory, trajectory_args)

    logging.info("Trajectory computation completed.")


def _compute_distances_and_edges(
    laz_file: Path,
    trajectory_file: Path,
    distance_file: Path,
    edge_file: Path,
    overwrite: bool,
    skip_existing: bool,
) -> None:
    """
    _summary_

    Parameters
    ----------
    laz_file : Path
        _description_
    trajectory_file : Path
        _description_
    distance_file : Path
        _description_
    edge_file : Path
        _description_
    overwrite : bool
        _description_
    skip_existing : bool
        _description_
    """

    output_files = [distance_file, edge_file]
    existing_files = []
    for file_path in output_files:
        if file_path.exists():
            existing_files.append(file_path)
    if skip_existing:
        if len(existing_files) == len(output_files):
            logging.info(
                f"Output files for {laz_file.name} already exist. Skipping processing."
            )
            return
        if len(existing_files) > 0:
            logging.error(
                f"Only some of the expected output files already exist for {laz_file.name}: {', '.join(str(f) for f in existing_files)}. This likely means that a previous run of the pipeline was interrupted. Please check the existing files and either remove them or use --overwrite to overwrite them."
            )
    if overwrite:
        if len(existing_files) > 0:
            logging.info(
                f"Overwriting existing files for {laz_file.name}: {', '.join(str(f) for f in existing_files)}."
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
        str(trajectory_file),
        "-d",
        str(distance_file),
        "-e",
        str(edge_file),
    ]

    if overwrite:
        command.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command)
    if return_code != 0:
        logging.error(f"Failed to process {laz_file.name}.")
    else:
        logging.info(f"Successfully processed {laz_file.name}.")


def _compute_distances_and_edges_single(
    args: Tuple[Path, Path, Path, Path, int, int, bool, bool],
) -> Tuple[Path, Path]:
    """Helper function for processing a single file with C++ pipeline.

    Parameters
    ----------
    args : Tuple[Path, Path, Path, Path, int, int, bool, bool]
        laz_file, trajectory_file, distance_file, edge_file, index, total_files, overwrite, skip_existing
    """
    (
        laz_file,
        trajectory_file,
        distance_file,
        edge_file,
        index,
        total_files,
        overwrite,
        skip_existing,
    ) = args

    logging.info(
        f"\n\nO----- [{index}/{total_files}] Processing tile: {laz_file.name} -----O\n"
    )

    _compute_distances_and_edges(
        laz_file=laz_file,
        trajectory_file=trajectory_file,
        distance_file=distance_file,
        edge_file=edge_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    return distance_file, edge_file


def _compute_distances_and_edges_parallel(
    flight_strip_files: List[Path],
    trajectory_files: List[Path],
    distances_files: List[Path],
    edges_files: List[Path],
    overwrite: bool,
    skip_existing: bool,
    num_workers: Optional[int] = None,
) -> Tuple[List[Path], List[Path]]:
    """Process all flight strips with C++ pipeline in parallel.

    Args:
        flight_strip_files: List of flight strip LAZ file paths
        tile_dir: Output directory
        overwrite: Whether to overwrite existing files
        skip_existing: Whether to skip if files exist
        num_workers: Number of worker processes (None = CPU count)

    Returns:
        Tuple of (distances_files, edges_files) lists
    """
    logging.info(
        f"Processing {len(flight_strip_files)} flight strips with C++ pipeline in parallel..."
    )

    total_files = len(flight_strip_files)

    # Prepare arguments for multiprocessing
    cpp_args = [
        (
            laz_file,
            trajectory_file,
            distance_file,
            edge_file,
            index,
            total_files,
            overwrite,
            skip_existing,
        )
        for index, (laz_file, trajectory_file, distance_file, edge_file) in enumerate(
            zip(flight_strip_files, trajectory_files, distances_files, edges_files),
            start=1,
        )
    ]

    # Use multiprocessing pool to process files in parallel
    with Pool(processes=num_workers) as pool:
        results = pool.map(_compute_distances_and_edges_single, cpp_args)

    # Separate the results into distances and edges files
    distances_files = [result[0] for result in results]
    edges_files = [result[1] for result in results]

    logging.info("C++ pipeline processing completed.")
    return distances_files, edges_files


def _merge_output_files(
    distances_files: List[Path],
    edges_files: List[Path],
    merged_distances_file: Path,
    merged_edges_file: Path,
    overwrite: bool,
    skip_existing: bool,
) -> None:
    """Merge distances and edges files from all flight strips.

    Parameters
    ----------
    distances_files : List[Path]
        List of distances LAZ files to merge.
    edges_files : List[Path]
        List of edges LAZ files to merge.
    merged_distances_file : Path
        Output path for merged distances LAZ file.
    merged_edges_file : Path
        Output path for merged edges LAZ file.
    overwrite : bool
        Whether to overwrite existing merged files.
    skip_existing : bool
        Whether to skip merging if merged files already exist.
    """
    logging.info("\n\nO----- Merging files -----O\n")

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

    logging.info("File merging completed.")


def _compute_roofprints(
    merged_edges_file: Path,
    bd_topo_edges_file: Path,
    bd_topo_intersections_file: Path,
    output_roofprints_file: Path,
    max_iterations: int,
    overwrite: bool,
    skip_existing: bool,
) -> None:
    """Compute roofprints from merged edges and BD TOPO data.

    Parameters
    ----------
    merged_edges_file : Path
        Path to merged edges LAZ file.
    bd_topo_edges_file : Path
        Path to cropped BD TOPO edges Parquet file.
    bd_topo_intersections_file : Path
        Path to cropped BD TOPO intersections Parquet file.
    output_roofprints_file : Path
        Output path for roofprints file.
    max_iterations : int
        Maximum number of iterations for roofprint computation.
    overwrite : bool
        Whether to overwrite existing roofprints file.
    skip_existing : bool
        Whether to skip computation if roofprints file already exists.
    """
    logging.info("\n\nO----- Computing roofprints -----O\n")

    if output_roofprints_file.exists():
        if skip_existing:
            logging.info(f"{output_roofprints_file} already exists. Skipping.")
            return
        if overwrite:
            logging.info(f"Overwriting {output_roofprints_file}.")
        else:
            raise FileExistsError(
                f"{output_roofprints_file} already exists. Use --overwrite to overwrite it or --skip_existing to skip creating it."
            )

    command = [
        "pixi",
        "run",
        "--quiet",
        "cpp",
        "run-only",
        "release",
        "--",
        "compute_roofprints",
        "-l",
        str(merged_edges_file),
        "-e",
        str(bd_topo_edges_file),
        "-i",
        str(bd_topo_intersections_file),
        "-n",
        str(max_iterations),
        "-o",
        str(output_roofprints_file),
    ]

    if overwrite:
        command.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command)
    if return_code != 0:
        logging.error("Failed to compute roofprints.")
    else:
        logging.info("Successfully computed roofprints.")


def run_pipeline_implementation(
    other_data_dir: Path,
    tile_dir: Path,
    overwrite: bool,
    skip_existing: bool,
    num_workers: Optional[int],
):
    """Execute the complete pipeline to compute roofprints from LiDAR HD data.

    Args:
        other_data_dir: Directory containing BD TOPO data
        tile_dir: Working directory for intermediate and output files
        bd_topo_file: Path to BD TOPO shapefile
        overwrite: Whether to overwrite existing files
        skip_existing: Whether to skip processing if output files exist
        num_workers: Number of worker processes for multiprocessing (None = CPU count)
    """
    initial_laz_file = tile_dir / "lidarhd.copc.laz"

    # Build the C++ tools
    _build_cpp_tool()

    # -------------------------------------------------------------------- #
    #                                BD TOPO                               #
    # -------------------------------------------------------------------- #

    bd_topo_dir = other_data_dir / "bd_topo"
    edges_file = bd_topo_dir / "edges.parquet"
    intersections_file = bd_topo_dir / "intersections.parquet"
    groups_file = bd_topo_dir / "building_groups.parquet"
    cropped_edges_file = tile_dir / "bd_topo-edges.parquet"
    cropped_intersections_file = tile_dir / "bd_topo-intersections.parquet"
    cropped_groups_file = tile_dir / "bd_topo-building_groups.parquet"

    _process_bd_topo_data(
        initial_laz_file=initial_laz_file,
        input_edges_file=edges_file,
        input_intersections_file=intersections_file,
        input_building_groups_file=groups_file,
        output_edges_file=cropped_edges_file,
        output_intersections_file=cropped_intersections_file,
        output_building_groups_file=cropped_groups_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    # -------------------------------------------------------------------- #
    #                               LiDAR HD                               #
    # -------------------------------------------------------------------- #

    # Process LiDAR HD: compute inward directions and split by flight strips
    las_with_inwards_roof_file = tile_dir / "lidarhd-with_inwards_roof.laz"
    template_flight_strip_file = tile_dir / "axis_#.laz"

    all_flight_strip_files = _process_lidar_hd_data(
        initial_laz_file=initial_laz_file,
        laz_with_inwards_roof_file=las_with_inwards_roof_file,
        template_flight_strip_file=template_flight_strip_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    # Compute trajectories in parallel
    trajectory_files = [
        tile_dir / f"{laz_file.stem}-trajectory.txt"
        for laz_file in all_flight_strip_files
    ]
    _compute_trajectories_parallel(
        flight_strip_files=all_flight_strip_files,
        trajectory_files=trajectory_files,
        overwrite=overwrite,
        skip_existing=skip_existing,
        num_workers=num_workers,
    )

    # Process flight strips with C++ pipeline in parallel
    distances_files = [
        tile_dir / f"{laz_file.stem}-distances.laz"
        for laz_file in all_flight_strip_files
    ]
    edges_files = [
        tile_dir / f"{laz_file.stem}-edges.laz" for laz_file in all_flight_strip_files
    ]
    _compute_distances_and_edges_parallel(
        flight_strip_files=all_flight_strip_files,
        trajectory_files=trajectory_files,
        distances_files=distances_files,
        edges_files=edges_files,
        overwrite=overwrite,
        skip_existing=skip_existing,
        num_workers=num_workers,
    )

    # Merge output files
    merged_distances_file = tile_dir / "merged_distances.laz"
    merged_edges_file = tile_dir / "merged_edges.laz"
    _merge_output_files(
        distances_files=distances_files,
        edges_files=edges_files,
        merged_distances_file=merged_distances_file,
        merged_edges_file=merged_edges_file,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    # -------------------------------------------------------------------- #
    #                              Roofprints                              #
    # -------------------------------------------------------------------- #

    roofprints_file = tile_dir / "roofprints.parquet"
    _compute_roofprints(
        merged_edges_file=merged_edges_file,
        bd_topo_edges_file=cropped_edges_file,
        bd_topo_intersections_file=cropped_intersections_file,
        output_roofprints_file=roofprints_file,
        max_iterations=1,
        overwrite=overwrite,
        skip_existing=skip_existing,
    )

    logging.info("\n\nPipeline completed successfully.")


def run_pipeline_call(
    other_data_dir: Path,
    tile_dir: Path,
    overwrite: bool,
    skip_existing: bool,
    verbose_int: int,
    num_workers: Optional[int],
):
    """Execute the complete pipeline to compute roofprints from LiDAR HD data.

    Args:
        other_data_dir: Directory containing BD TOPO data
        tile_dir: Working directory for intermediate and output files
        bd_topo_file: Path to BD TOPO shapefile
        overwrite: Whether to overwrite existing files
        skip_existing: Whether to skip processing if output files exist
        verbose_int: Verbosity level (0-4)
        num_workers: Number of worker processes for multiprocessing (None = CPU count)
    """
    with LoggingContext(verbose=verbose_int):
        run_pipeline_implementation(
            other_data_dir=other_data_dir,
            tile_dir=tile_dir,
            overwrite=overwrite,
            skip_existing=skip_existing,
            num_workers=num_workers,
        )
