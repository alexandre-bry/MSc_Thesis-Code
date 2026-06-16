import logging
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import Pool
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import List, Optional, Tuple

from tqdm import tqdm

from ..outline.intersections import (
    crop_intersections_implementation,
)
from ..point_cloud.las_manipulations import (
    classification_mapping_implementation,
    merge_files,
    split_point_cloud_implementation,
)
from ..roof.roof import roofprints_to_lod22_implementation
from ..utils.custom_logging import (
    LoggingContext,
    Verbose,
    run_command_with_tqdm_logging,
)
from ..utils.input_output import InputOutput, OutputActionEnum, OutputBehaviour


def _build_cpp_tool():
    """Builds the C++ program to be able to use it in the rest of the pipeline."""
    logging.info(f"Building the C++ tools...")
    command_build = ["pixi", "run", "--quiet", "just", "build", "release"]
    return_code = run_command_with_tqdm_logging(command_build)
    if return_code != 0:
        logging.error("C++ build failed.")
        raise RuntimeError("C++ build failed.")
    else:
        logging.info("C++ tools built successfully.")


def _compute_inward_direction(
    input_las_path: Path, output_las_path: Path, input_output: InputOutput
):
    """Compute the inward direction to prepare the computation of the roofprints.

    Parameters
    ----------
    input_las_path : Path
        Path to the input point cloud.
    output_las_path : Path
        Path to the output point cloud with the new attributes.
    input_output: InputOutput
        The handler for input and output file issues.

    Raises
    ------
    FileExistsError
        _description_
    RuntimeError
        _description_
    """
    logging.info("Computing the inward direction...")
    message_prefix = "Inward direction computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[input_las_path],
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_las_path]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    command_inwards = [
        "pixi",
        "run",
        "just",
        "run-only",
        "release",
        "inward_directions",
        "-i",
        str(input_las_path),
        "-o",
        str(output_las_path),
        "-t",
        "roof",
    ]

    if input_output.is_overwrite():
        command_inwards.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command_inwards)
    if return_code != 0:
        logging.error(f"Failed to create {output_las_path}.")
        raise RuntimeError(f"Failed to create {output_las_path}.")
    else:
        logging.info(f"Successfully created {output_las_path}.")


def _split_source_point_cloud(
    input_las_path: Path,
    output_template_path: Path,
    input_output: InputOutput,
) -> List[Path]:
    all_strip_files = split_point_cloud_implementation(
        input_file=input_las_path,
        output_file_template=output_template_path,
        dimension="PointSourceId",
        input_output=input_output,
    )
    return all_strip_files


def _compute_trajectory(
    input_las_path: Path,
    output_trajectory_path: Path,
    input_output: InputOutput,
    display: bool,
) -> bool:
    """
    Compute the trajectory for a given flight strip using the traj-estimation command.

    Parameters
    ----------
    input_las_path : Path
        Path to the input flight strip LAS/LAZ file.
    output_trajectory_path : Path
        Path to the output trajectory file.
    input_output: InputOutput
        The handler for input and output file issues.
    display : bool
        Whether to display the command output.

    Returns
    -------
    bool
        True if the trajectory was computed successfully, False otherwise.

    Raises
    ------
    FileExistsError
        If the output file already exists and overwrite is False.
    """
    logging.info(f"Computing the trajectory for {input_las_path}...")
    message_prefix = "Trajectory computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[input_las_path],
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_trajectory_path]],
    )
    if output_action == OutputActionEnum.SKIP:
        return True

    command_trajectory_file_1 = input_las_path.parent / f"{input_las_path.stem}_1.txt"
    command_trajectory_file_2 = input_las_path.parent / f"{input_las_path.stem}_2.txt"

    command = ["pixi", "run", "traj-estimation", str(input_las_path)]
    return_code = run_command_with_tqdm_logging(command, display=display)
    if return_code != 0:
        logging.error(f"Failed to compute trajectory for {input_las_path.name}.")
        raise RuntimeError(f"Failed to compute trajectory for {input_las_path.name}.")
    else:
        logging.info(f"Successfully computed trajectory for {input_las_path.name}.")

    # Check if the trajectory file was created
    if not command_trajectory_file_1.exists():
        logging.error(
            f"Could not find the expected output ({command_trajectory_file_1}) for the trajectory of {input_las_path.name}."
        )
        raise RuntimeError(
            f"Could not find the expected output ({command_trajectory_file_1}) for the trajectory of {input_las_path.name}."
        )

    # Rename it to match the expected format for the next steps
    command_trajectory_file_1.rename(output_trajectory_path)

    # Check if there was a second file created
    if command_trajectory_file_2.exists():
        logging.error(
            f"Second trajectory file found for {input_las_path.name}: {command_trajectory_file_2}. This means that the initial point cloud was not split properly between the different flight axes."
        )
        raise RuntimeError(
            f"Second trajectory file found for {input_las_path.name}: {command_trajectory_file_2}. This means that the initial point cloud was not split properly between the different flight axes."
        )
    # Remove the other output file if it exists, since it is not needed
    other_output_file = (
        input_las_path.parent / f"{input_las_path.stem}_center_refine.txt"
    )
    if other_output_file.exists():
        other_output_file.unlink()

    return True


def _compute_single_trajectory(args: Tuple[Path, Path, InputOutput, bool]) -> bool:
    """Helper function for multiprocessing trajectory computation."""
    input_las_path, output_trajectory_path, input_output, display = args
    return _compute_trajectory(
        input_las_path=input_las_path,
        output_trajectory_path=output_trajectory_path,
        input_output=input_output,
        display=display,
    )


def _process_bd_topo_data(
    initial_laz_file: Path,
    input_edges_file: Path,
    input_intersections_file: Path,
    input_building_groups_file: Path,
    output_edges_file: Path,
    output_intersections_file: Path,
    output_building_groups_file: Path,
    input_output: InputOutput,
) -> None:
    """Process and crop BD TOPO data to match the LiDAR HD bounds.

    Parameters
    ----------
    initial_laz_file : Path
        Path to the initial LiDAR HD LAS/LAZ file, used to get the bounding box for cropping the BD TOPO data.
    input_edges_file : Path
        Path to the input BD TOPO edges Parquet file.
    input_intersections_file : Path
        Path to the input BD TOPO intersections Parquet file.
    input_building_groups_file : Path
        Path to the input BD TOPO building groups Parquet file.
    output_edges_file : Path
        Path to the output cropped BD TOPO edges Parquet file.
    output_intersections_file : Path
        Path to the output cropped BD TOPO intersections Parquet file.
    output_building_groups_file : Path
        Path to the output cropped BD TOPO building groups Parquet file.
    input_output : InputOutput
        The handler for input and output file issues.

    Raises
    ------
    FileNotFoundError
        If any of the required files are missing.
    """

    logging.info("Processing BD TOPO data...")
    message_prefix = "BD TOPO processing"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[
            initial_laz_file,
            input_edges_file,
            input_intersections_file,
            input_building_groups_file,
        ],
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[
            [
                output_edges_file,
                output_intersections_file,
                output_building_groups_file,
            ]
        ],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    # Crop the BD TOPO data to the bounds of the source LAS/LAZ file
    crop_intersections_implementation(
        input_las_file=initial_laz_file,
        input_edges_file=input_edges_file,
        input_intersections_file=input_intersections_file,
        input_building_groups_file=input_building_groups_file,
        output_edges_file=output_edges_file,
        output_intersections_file=output_intersections_file,
        output_building_groups_file=output_building_groups_file,
        input_output=input_output,
    )


def _process_lidar_hd_data(
    initial_laz_file: Path,
    laz_with_inwards_roof_file: Path,
    template_flight_strip_file: Path,
    input_output: InputOutput,
) -> List[Path]:
    """Process LiDAR HD data: compute inward directions and split by flight strips.

    Parameters
    ----------
    initial_laz_file : Path
        Input LiDAR HD LAS/LAZ file
    laz_with_inwards_roof_file : Path
        Output file for LAS/LAZ with inward directions
    template_flight_strip_file : Path
        Template file for flight strip splitting (should contain a "#" character to be replaced by the strip number)
    input_output : InputOutput
        The handler for input and output file issues.

    Returns
    -------
    List[Path]
        List of flight strip LAS/LAZ file paths
    """
    logging.info("Processing LiDAR HD data...")

    # Compute the inward directions
    _compute_inward_direction(
        input_las_path=initial_laz_file,
        output_las_path=laz_with_inwards_roof_file,
        input_output=input_output,
    )

    # Split the source LAS/LAZ file into multiple files based on the "PointSourceId" attribute
    all_flight_strip_files = _split_source_point_cloud(
        input_las_path=laz_with_inwards_roof_file,
        output_template_path=template_flight_strip_file,
        input_output=input_output,
    )

    return sorted(all_flight_strip_files)


def _compute_trajectories_parallel(
    flight_strip_files: List[Path],
    trajectory_files: List[Path],
    input_output: InputOutput,
    num_workers: Optional[int] = None,
) -> List[bool]:
    """Compute trajectories for all flight strips in parallel.

    Parameters
    ----------
    flight_strip_files : List[Path]
        List of flight strip LAS/LAZ file paths.
    trajectory_files : List[Path]
        List of output trajectory file paths corresponding to each flight strip.
    input_output : InputOutput
        The handler for input and output file issues.
    num_workers : Optional[int], optional
        Number of worker processes (None = CPU count), by default None

    Returns
    -------
    List[bool]
        List of boolean values indicating success or failure for each flight strip.
    """

    logging.info(
        f"Computing trajectories for {len(flight_strip_files)} flight strips in parallel..."
    )

    # Prepare arguments for multiprocessing
    trajectory_args = [
        (
            laz_file,
            trajectory_file,
            input_output,
            False,  # display
        )
        for laz_file, trajectory_file in zip(flight_strip_files, trajectory_files)
    ]

    # Use multiprocessing pool to compute trajectories in parallel
    with Pool(processes=num_workers) as pool:
        results = pool.map(_compute_single_trajectory, trajectory_args)

    logging.info("Trajectory computation completed.")
    return results


def _compute_distances_and_edges(
    laz_file: Path,
    trajectory_file: Path,
    distance_file: Path,
    edge_file: Path,
    input_output: InputOutput,
    display: bool,
) -> None:
    """
    Compute distances and edges for a single flight strip using the C++ pipeline.

    Parameters
    ----------
    laz_file : Path
        Path to the input flight strip LAS/LAZ file.
    trajectory_file : Path
        Path to the trajectory file for the flight strip.
    distance_file : Path
        Path to the output distances LAS/LAZ file.
    edge_file : Path
        Path to the output edges LAS/LAZ file.
    input_output : InputOutput
        The handler for input and output file issues.
    display : bool
        Whether to display the command output.
    """
    message_prefix = "Distances and edges computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[laz_file, trajectory_file],
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[distance_file, edge_file]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    command = [
        "pixi",
        "run",
        "--quiet",
        "just",
        "run-only",
        "release",
        "roof_edge_points",
        "-i",
        str(laz_file),
        "-t",
        str(trajectory_file),
        "-d",
        str(distance_file),
        "-e",
        str(edge_file),
    ]

    if input_output.is_overwrite():
        command.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command, display=display)
    if return_code != 0:
        logging.error(f"Failed to process {laz_file.name}.")
        raise RuntimeError(f"Failed to process {laz_file.name}.")
    else:
        logging.info(f"Successfully processed {laz_file.name}.")


def _compute_distances_and_edges_single(
    args: Tuple[Path, Path, Path, Path, int, int, InputOutput, bool],
) -> Tuple[Path, Path]:
    """Helper function for processing a single file with C++ pipeline.

    Parameters
    ----------
    args : Tuple[Path, Path, Path, Path, int, int, InputOutput, bool]
        laz_file, trajectory_file, distance_file, edge_file, index, total_files, input_output, display
    """
    (
        laz_file,
        trajectory_file,
        distance_file,
        edge_file,
        index,
        total_files,
        input_output,
        display,
    ) = args

    logging.info(
        f"\n\nO----- [{index}/{total_files}] Processing tile: {laz_file.name} -----O\n"
    )

    _compute_distances_and_edges(
        laz_file=laz_file,
        trajectory_file=trajectory_file,
        distance_file=distance_file,
        edge_file=edge_file,
        input_output=input_output,
        display=display,
    )

    return distance_file, edge_file


def _compute_distances_and_edges_parallel(
    flight_strip_files: List[Path],
    trajectory_files: List[Path],
    distances_files: List[Path],
    edges_files: List[Path],
    input_output: InputOutput,
    num_workers: Optional[int] = None,
) -> Tuple[List[Path], List[Path]]:
    """Process all flight strips with C++ pipeline in parallel.

    Parameters
    ----------
    flight_strip_files : List[Path]
        List of flight strip LAS/LAZ file paths.
    trajectory_files : List[Path]
        List of trajectory file paths.
    distances_files : List[Path]
        List of distances file paths.
    edges_files : List[Path]
        List of edges file paths.
    input_output : InputOutput
        The handler for input and output file issues.
    num_workers : Optional[int], optional
        The number of worker processes to use (None = CPU count).
        By default None.

    Returns
    -------
    Tuple[List[Path], List[Path]]
        A tuple containing the list of distances file paths and the list of edges file paths.
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
            input_output,
            False,  # display
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
    input_output: InputOutput,
) -> None:
    """Merge distances and edges files from all flight strips.

    Parameters
    ----------
    distances_files : List[Path]
        List of distances LAS/LAZ LAS/LAZ files to merge.
    edges_files : List[Path]
        List of edges LAS/LAZ files to merge.
    merged_distances_file : Path
        Output path for merged distances LAS/LAZ file.
    merged_edges_file : Path
        Output path for merged edges LAS/LAZ file.
    input_output : InputOutput
        The handler for input and output file issues.
    """
    logging.info("Merging files")

    merge_files(
        input_files=distances_files,
        output_file=merged_distances_file,
        input_output=input_output,
    )
    merge_files(
        input_files=edges_files,
        output_file=merged_edges_file,
        input_output=input_output,
    )

    logging.info("File merging completed.")


def _compute_roofprints(
    merged_edges_file: Path,
    bd_topo_edges_file: Path,
    bd_topo_intersections_file: Path,
    output_roofprints_template_file: Path,
    n_iterations: int,
    input_output: InputOutput,
) -> List[Path]:
    """Compute roofprints from merged edges and BD TOPO data.

    Parameters
    ----------
    merged_edges_file : Path
        Path to merged edges LAS/LAZ file.
    bd_topo_edges_file : Path
        Path to cropped BD TOPO edges Parquet file.
    bd_topo_intersections_file : Path
        Path to cropped BD TOPO intersections Parquet file.
    output_roofprints_template_file : Path
        Output path for roofprints template file, with one '{iteration}' placeholder which will be replaced with the n_iterations value.
    n_iterations : int
        Number of iterations for roofprint computation.
    input_output : InputOutput
        The handler for input and output file issues.

    Raises
    ------
    FileExistsError
        If the output roofprints file already exists and overwrite is False.
    """
    logging.info("\n\nO----- Computing roofprints -----O\n")

    message_prefix = "Roofprints computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[merged_edges_file, bd_topo_edges_file, bd_topo_intersections_file],
    )

    # Check the template
    if str(output_roofprints_template_file).count("{iteration}") != 1:
        logging.error(
            f"The output roofprints template file name must contain exactly one '{{iteration}}' placeholder to be replaced with the iteration number. Provided file: {output_roofprints_template_file}"
        )
        return []

    iterations = range(1, n_iterations + 1)
    output_roofprints_files = [
        Path(str(output_roofprints_template_file).replace("{iteration}", str(n)))
        for n in iterations
    ]

    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[output_roofprints_files],
    )
    if output_action == OutputActionEnum.SKIP:
        return output_roofprints_files

    logging.info(f"Computing roofprints with n_iterations={n_iterations}...")

    command = [
        "pixi",
        "run",
        "--quiet",
        "just",
        "run-only",
        "release",
        "roofprints",
        "-l",
        str(merged_edges_file),
        "-e",
        str(bd_topo_edges_file),
        "-i",
        str(bd_topo_intersections_file),
        "-n",
        str(n_iterations),
        "-o",
        str(output_roofprints_template_file),
    ]

    if input_output.is_overwrite():
        command.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command)
    if return_code != 0:
        logging.error("Failed to compute roofprints.")
        raise RuntimeError("Failed to compute roofprints.")
    else:
        logging.info("Successfully computed roofprints.")

    return output_roofprints_files


def _compute_lod22(
    lidar_hd_reclassified_file: Path,
    roofprints_file: Path,
    output_lod22_cj_file: Path,
    input_output: InputOutput,
):
    # Compute the roof using roofer
    roofprints_to_lod22_implementation(
        point_cloud_path=lidar_hd_reclassified_file,
        roofprints_path=roofprints_file,
        roof_path=output_lod22_cj_file,
        input_output=input_output,
    )


def _compute_footprints(
    lidar_hd_file: Path,
    lod22_file: Path,
    roofprints_file: Path,
    output_footprints_template_file: Path,
    n_iterations: int,
    input_output: InputOutput,
) -> List[Path]:
    """
    Compute the footprints using the C++ pipeline.

    Parameters
    ----------
    lidar_hd_file : Path
        Path to the LiDAR HD file.
    lod22_file : Path
        Path to the LoD22 file.
    roofprints_file : Path
        Path to the roofprints file.
    output_footprints_template_file : Path
        Path to the output footprints template file, with one '{iteration}' placeholder which will be replaced with the iteration number.
    n_iterations : int
        Number of iterations for footprint computation.
    input_output : InputOutput
        The handler for input and output file issues.

    Raises
    ------
    FileExistsError
        If the output footprint file already exists and overwrite is False.
    """
    logging.info("\n\nO----- Computing footprints -----O\n")

    message_prefix = "Footprints computation"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[lidar_hd_file, lod22_file, roofprints_file],
    )

    # Check the template
    if str(output_footprints_template_file).count("{iteration}") != 1:
        logging.error(
            f"The output footprints template file name must contain exactly one '{{iteration}}' placeholder to be replaced with the iteration number. Provided file: {output_footprints_template_file}"
        )
        return []

    iterations = range(1, n_iterations + 1)
    output_footprints_files = [
        Path(str(output_footprints_template_file).replace("{iteration}", str(n)))
        for n in iterations
    ]

    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[output_footprints_files],
    )
    if output_action == OutputActionEnum.SKIP:
        return output_footprints_files

    logging.info(f"Computing footprints with n_iterations={n_iterations}...")

    command = [
        "pixi",
        "run",
        "--quiet",
        "just",
        "run-only",
        "release",
        "footprints",
        "-p",
        str(lidar_hd_file),
        "-l",
        str(lod22_file),
        "-r",
        str(roofprints_file),
        "-o",
        str(output_footprints_template_file),
        "-n",
        str(n_iterations),
    ]

    if input_output.is_overwrite():
        command.append("--overwrite")

    return_code = run_command_with_tqdm_logging(command)
    if return_code != 0:
        logging.error("Failed to compute footprints.")
        raise RuntimeError("Failed to compute footprints.")
    else:
        logging.info("Successfully computed footprints.")

    return output_footprints_files


def run_pipeline_implementation(
    bd_topo_dir: Path,
    tile_dir: Path,
    stop_after_roofprints: bool,
    stop_after_lod22: bool,
    input_output: InputOutput,
    num_workers: Optional[int],
):
    """Execute the complete pipeline to compute roofprints from LiDAR HD data.

    Parameters
    ----------
    bd_topo_dir : Path
        Directory containing BD TOPO data
    tile_dir : Path
        Working directory for intermediate and output files
    stop_after_roofprints : bool
        Whether to stop after computing roofprints
    stop_after_lod22 : bool
        Whether to stop after computing LoD2.2 models
    input_output: InputOutput
        The handler for input and output file issues.
    num_workers : Optional[int]
        Number of worker processes for multiprocessing (None = CPU count)
    """
    tile_bd_topo_dir = tile_dir / "bd_topo"
    tile_bd_topo_dir.mkdir(exist_ok=True)
    tile_lidar_hd_dir = tile_dir / "lidar_hd"
    tile_lidar_hd_dir.mkdir(exist_ok=True)
    tile_axes_dir = tile_lidar_hd_dir / "axes"
    tile_axes_dir.mkdir(exist_ok=True)
    tile_roofprints_dir = tile_dir / "roofprints"
    tile_roofprints_dir.mkdir(exist_ok=True)
    tile_roof_dir = tile_dir / "roof"
    tile_roof_dir.mkdir(exist_ok=True)
    tile_footprints_dir = tile_dir / "footprints"
    tile_footprints_dir.mkdir(exist_ok=True)

    initial_laz_file = tile_lidar_hd_dir / "lidar_hd.copc.laz"
    if not initial_laz_file.exists():
        initial_laz_file = tile_lidar_hd_dir / "lidar_hd.laz"
    input_output.handle_input(
        message_prefix="Initial LiDAR HD file",
        input_files=[initial_laz_file],
    )

    # Build the C++ tools
    _build_cpp_tool()

    # -------------------------------------------------------------------- #
    #                                BD TOPO                               #
    # -------------------------------------------------------------------- #

    edges_file = bd_topo_dir / "edges.parquet"
    intersections_file = bd_topo_dir / "intersections.parquet"
    groups_file = bd_topo_dir / "building_groups.parquet"
    cropped_edges_file = tile_bd_topo_dir / "edges.parquet"
    cropped_intersections_file = tile_bd_topo_dir / "intersections.parquet"
    cropped_groups_file = tile_bd_topo_dir / "building_groups.parquet"

    _process_bd_topo_data(
        initial_laz_file=initial_laz_file,
        input_edges_file=edges_file,
        input_intersections_file=intersections_file,
        input_building_groups_file=groups_file,
        output_edges_file=cropped_edges_file,
        output_intersections_file=cropped_intersections_file,
        output_building_groups_file=cropped_groups_file,
        input_output=input_output,
    )

    # -------------------------------------------------------------------- #
    #                               LiDAR HD                               #
    # -------------------------------------------------------------------- #

    # Process LiDAR HD: compute inward directions and split by flight strips
    las_with_inwards_roof_file = tile_lidar_hd_dir / "lidar_hd-with_inwards_roof.laz"
    template_flight_strip_file = tile_axes_dir / "axis_#.laz"

    all_flight_strip_files = _process_lidar_hd_data(
        initial_laz_file=initial_laz_file,
        laz_with_inwards_roof_file=las_with_inwards_roof_file,
        template_flight_strip_file=template_flight_strip_file,
        input_output=input_output,
    )

    # Compute trajectories in parallel
    trajectory_files = [
        tile_axes_dir / f"{laz_file.stem}-trajectory.txt"
        for laz_file in all_flight_strip_files
    ]
    successes = _compute_trajectories_parallel(
        flight_strip_files=all_flight_strip_files,
        trajectory_files=trajectory_files,
        input_output=input_output,
        num_workers=num_workers,
    )

    # Only keep files for which the trajectory computation was successful
    all_flight_strip_files = [
        laz for laz, success in zip(all_flight_strip_files, successes) if success
    ]
    trajectory_files = [
        traj for traj, success in zip(trajectory_files, successes) if success
    ]

    # Process flight strips with C++ pipeline in parallel
    distances_files = [
        tile_axes_dir / f"{laz_file.stem}-distances.laz"
        for laz_file in all_flight_strip_files
    ]
    edges_files = [
        tile_axes_dir / f"{laz_file.stem}-edges.laz"
        for laz_file in all_flight_strip_files
    ]
    _compute_distances_and_edges_parallel(
        flight_strip_files=all_flight_strip_files,
        trajectory_files=trajectory_files,
        distances_files=distances_files,
        edges_files=edges_files,
        input_output=input_output,
        num_workers=num_workers,
    )

    # Merge output files
    merged_distances_file = tile_lidar_hd_dir / "merged_distances.laz"
    merged_edges_file = tile_lidar_hd_dir / "merged_edges.laz"
    _merge_output_files(
        distances_files=distances_files,
        edges_files=edges_files,
        merged_distances_file=merged_distances_file,
        merged_edges_file=merged_edges_file,
        input_output=input_output,
    )

    # -------------------------------------------------------------------- #
    #                              Roofprints                              #
    # -------------------------------------------------------------------- #

    n_iterations_roofprints = 3
    roofprints_template_file = tile_roofprints_dir / f"roofprints-{{iteration}}.parquet"

    roofprints_files = _compute_roofprints(
        merged_edges_file=merged_edges_file,
        bd_topo_edges_file=cropped_edges_file,
        bd_topo_intersections_file=cropped_intersections_file,
        output_roofprints_template_file=roofprints_template_file,
        n_iterations=n_iterations_roofprints,
        input_output=input_output,
    )

    if stop_after_roofprints:
        logging.info("Stopping pipeline after roofprints as requested.")
        return

    # ------------------------------------------------------------------------ #
    #                                   Roof                                   #
    # ------------------------------------------------------------------------ #

    # Reclassify the input LiDAR HD file to classify as building all the points which are potentially building points, as roofer only uses those
    reclassified_lidar_hd_file = tile_lidar_hd_dir / "lidar_hd-reclassified.laz"
    classification_mapping_implementation(
        input_file=initial_laz_file,
        output_file=reclassified_lidar_hd_file,
        mapping={1: 6, 64: 6, 65: 6, 67: 6},
        input_output=input_output,
    )

    # Compute the LoD22 models
    lod22_files = []
    for roofprints_file in roofprints_files:
        lod22_file = tile_roof_dir / f"{roofprints_file.stem}-roof.city.json"
        _compute_lod22(
            lidar_hd_reclassified_file=reclassified_lidar_hd_file,
            roofprints_file=roofprints_file,
            output_lod22_cj_file=lod22_file,
            input_output=input_output,
        )
        lod22_files.append(lod22_file)

    if stop_after_lod22:
        logging.info("Stopping pipeline after LoD22 as requested.")
        return

    # ------------------------------------------------------------------------ #
    #                                Footprints                                #
    # ------------------------------------------------------------------------ #

    n_iterations_footprints = 3
    for roofprints_file, lod22_file in zip(roofprints_files, lod22_files):
        output_footprints_template_file = (
            tile_footprints_dir
            / f"{roofprints_file.stem.replace('roofprints', 'footprints')}-{{iteration}}.parquet"
        )
        _compute_footprints(
            lidar_hd_file=initial_laz_file,
            lod22_file=lod22_file,
            roofprints_file=roofprints_file,
            output_footprints_template_file=output_footprints_template_file,
            n_iterations=n_iterations_footprints,
            input_output=input_output,
        )

    logging.info("Pipeline completed successfully.")


def run_pipeline_call(
    bd_topo_dir: Path,
    tile_dir: Path,
    stop_after_roofprints: bool,
    stop_after_lod22: bool,
    input_output: InputOutput,
    verbose: Verbose,
    num_workers: Optional[int],
):
    """Execute the complete pipeline to compute roofprints from LiDAR HD data.

    Parameters
    ----------
    bd_topo_dir: Path
        Directory containing BD TOPO data
    tile_dir: Path
        Working directory for intermediate and output files
    stop_after_roofprints: bool
        Whether to stop after computing roofprints
    stop_after_lod22: bool
        Whether to stop after computing LoD2.2 models
    input_output: InputOutput
        The handler for input and output file issues.
    verbose: Verbose
        The verbosity level for logging.
    num_workers: Optional[int]
        Number of worker processes for multiprocessing (None = CPU count)
    """
    with LoggingContext(verbose=verbose):
        run_pipeline_implementation(
            bd_topo_dir=bd_topo_dir,
            tile_dir=tile_dir,
            stop_after_roofprints=stop_after_roofprints,
            stop_after_lod22=stop_after_lod22,
            input_output=input_output,
            num_workers=num_workers,
        )


def _compare_polygon_datasets_single_job(
    args: Tuple[
        Path, Path, Path, Path, Path, str, float, Optional[List[str]], InputOutput
    ],
) -> Tuple[Path, Path, Path]:
    """Run one validation comparison job in a worker process."""
    (
        scored_file,
        output_indiv_file,
        output_aggreg_file,
        validation_dataset_indiv_file,
        validation_dataset_aggreg_file,
        id_column,
        spacing_m,
        keep_columns,
        input_output,
    ) = args

    from ..validation.metrics import compare_polygon_datasets_implementation

    with TemporaryDirectory() as tmp_dir:
        valid_scored_file = Path(tmp_dir) / scored_file.name
        try:
            command = [
                "pixi",
                "run",
                "--quiet",
                "gdal",
                "vector",
                "make-valid",
                "-i",
                str(scored_file),
                "-o",
                str(valid_scored_file),
            ]

            return_code = run_command_with_tqdm_logging(command, display=False)
            if return_code != 0:
                raise RuntimeError(f"Failed to make valid {scored_file}.")

            compare_polygon_datasets_implementation(
                ground_truth_path=validation_dataset_indiv_file,
                scored_path=valid_scored_file,
                output_path=output_indiv_file,
                id_column=id_column,
                spacing_m=spacing_m,
                keep_columns=keep_columns,
                input_output=input_output,
            )
            compare_polygon_datasets_implementation(
                ground_truth_path=validation_dataset_aggreg_file,
                scored_path=valid_scored_file,
                output_path=output_aggreg_file,
                id_column=id_column,
                spacing_m=spacing_m,
                keep_columns=keep_columns,
                input_output=input_output,
            )
        except Exception:
            logging.exception("Worker failed for %s.", scored_file)
            raise

    return scored_file, output_indiv_file, output_aggreg_file


def compute_metrics_implementation(
    validation_dataset_indiv_file: Path,
    validation_dataset_aggreg_file: Path,
    bd_topo_file: Path,
    tiles_dirs: List[Path],
    output_comparison_dir: Path,
    output_format: str,
    id_column: str,
    spacing_m: float,
    keep_columns: Optional[List[str]],
    input_output: InputOutput,
    num_workers: Optional[int],
):
    """Compare the pipeline output to a validation dataset.

    Parameters
    ----------
    validation_dataset_indiv_file: Path
        Path to the individual building validation dataset (Parquet file).
    validation_dataset_aggreg_file: Path
        Path to the aggregated building validation dataset (Parquet file).
    bd_topo_file: Path
        Path to the BD TOPO polygon dataset to compare.
    tiles_dirs: List[Path]
        List of tile directories containing the pipeline output to compare.
    output_comparison_dir: Path
        Directory where the comparison results will be saved.
    output_format: str
        Format to save the comparison results (e.g., 'parquet', 'csv', 'json').
    id_column: str
        Name of the column containing the building IDs in the datasets.
    spacing_m: float
        Spacing in meters to use for the comparison.
    keep_columns: Optional[List[str]]
         List of additional column names to keep in the output comparison results (in addition to the id_column). If None, only the id_column will be kept.
    input_output: InputOutput
        The handler for input and output file issues.
    num_workers: Optional[int]
        Number of worker processes for multiprocessing (None = CPU count)
    """

    message_prefix = "Computing metrics for pipeline output"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[
            validation_dataset_indiv_file,
            validation_dataset_aggreg_file,
            bd_topo_file,
        ],
    )

    # Check the output format
    valid_formats = ["parquet", "csv", "json"]
    if output_format not in valid_formats:
        logging.error(
            f"Invalid output format: {output_format}. Supported formats are: {', '.join(valid_formats)}."
        )
        raise ValueError(
            f"Invalid output format: {output_format}. Supported formats are: {', '.join(valid_formats)}."
        )

    comparison_jobs: List[
        Tuple[
            Path, Path, Path, Path, Path, str, float, Optional[List[str]], InputOutput
        ]
    ] = []

    scored_files: List[Path] = [bd_topo_file]
    output_indiv_files: List[Path] = [
        output_comparison_dir / f"bd_topo-indiv.{output_format}"
    ]
    output_aggreg_files: List[Path] = [
        output_comparison_dir / f"bd_topo-aggreg.{output_format}"
    ]

    for tile_dir in tiles_dirs:
        tile_name = tile_dir.name
        for roofprints_file in (tile_dir / "roofprints").glob("*.parquet"):
            roofprint_name = roofprints_file.stem
            scored_files.append(roofprints_file)
            output_indiv_files.append(
                output_comparison_dir
                / f"{tile_name}-{roofprint_name}-indiv.{output_format}"
            )
            output_aggreg_files.append(
                output_comparison_dir
                / f"{tile_name}-{roofprint_name}-aggreg.{output_format}"
            )

    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=scored_files,
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=list(zip(output_indiv_files, output_aggreg_files)),
    )
    if output_action == OutputActionEnum.SKIP:
        return

    for scored_file, output_indiv_file, output_aggreg_file in zip(
        scored_files, output_indiv_files, output_aggreg_files
    ):
        comparison_jobs.append(
            (
                scored_file,
                output_indiv_file,
                output_aggreg_file,
                validation_dataset_indiv_file,
                validation_dataset_aggreg_file,
                id_column,
                spacing_m,
                keep_columns,
                input_output,
            )
        )

    if not comparison_jobs:
        logging.info("No comparison jobs to run.")
        return

    logging.info(f"Comparing {len(comparison_jobs)} polygon datasets in parallel...")

    failed_jobs: List[Tuple[Path, Exception]] = []
    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        future_to_job = {
            executor.submit(_compare_polygon_datasets_single_job, job): job
            for job in comparison_jobs
        }
        for future in tqdm(
            as_completed(future_to_job),
            total=len(future_to_job),
            desc="Comparing polygon datasets",
        ):
            job = future_to_job[future]
            scored_file = job[0]
            try:
                future.result()
            except Exception as exc:
                failed_jobs.append((scored_file, exc))
                logging.exception("Comparison failed for %s.", scored_file)

    if failed_jobs:
        failed_names = ", ".join(scored_file.name for scored_file, _ in failed_jobs)
        raise RuntimeError(
            f"{len(failed_jobs)} comparison job(s) failed: {failed_names}"
        )

    logging.info("Comparison jobs completed successfully.")


def compute_metrics_call(
    validation_dataset_indiv_file: Path,
    validation_dataset_aggreg_file: Path,
    bd_topo_file: Path,
    tiles_dirs: List[Path],
    output_comparison_dir: Path,
    output_format: str,
    id_column: str,
    spacing_m: float,
    keep_columns: Optional[List[str]],
    input_output: InputOutput,
    verbose: Verbose,
    num_workers: Optional[int],
):
    """Compare the pipeline output to a validation dataset.

    Parameters
    ----------
    validation_dataset_indiv_file: Path
        Path to the individual building validation dataset (Parquet file).
    validation_dataset_aggreg_file: Path
        Path to the aggregated building validation dataset (Parquet file).
    bd_topo_file: Path
        Path to the BD TOPO polygon dataset to compare.
    tiles_dirs: List[Path]
        List of tile directories containing the pipeline output to compare.
    output_comparison_dir: Path
        Directory where the comparison results will be saved.
    output_format: str
        Format to save the comparison results (e.g., 'parquet', 'csv', 'json').
    id_column: str
        Name of the column containing the building IDs in the datasets.
    spacing_m: float
        Spacing in meters to use for the comparison.
    keep_columns: Optional[List[str]]
        List of additional column names to keep in the output comparison results (in addition to the id_column). If None, only the id_column will be kept.
    input_output: InputOutput
        The handler for input and output file issues.
    verbose: Verbose
        The verbosity level for logging.
    num_workers: Optional[int]
        Number of worker processes for multiprocessing (None = CPU count)
    """

    with LoggingContext(verbose=verbose):
        compute_metrics_implementation(
            validation_dataset_indiv_file=validation_dataset_indiv_file,
            validation_dataset_aggreg_file=validation_dataset_aggreg_file,
            bd_topo_file=bd_topo_file,
            tiles_dirs=tiles_dirs,
            output_comparison_dir=output_comparison_dir,
            output_format=output_format,
            id_column=id_column,
            spacing_m=spacing_m,
            keep_columns=keep_columns,
            input_output=input_output,
            num_workers=num_workers,
        )
