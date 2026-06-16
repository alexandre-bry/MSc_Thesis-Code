"""
Merge multiple point cloud files into a single output file while preserving all attributes.
Uses PDAL to perform the merge operation.
"""

import json
import logging
from pathlib import Path
from typing import Dict, List

from pdal import Filter, Pipeline, Reader, Writer
from tqdm import tqdm

from ..utils.custom_logging import LoggingContext
from ..utils.geom import Box2154, Point2154
from ..utils.input_output import InputOutput, OutputActionEnum, OutputBehaviour


def get_las_bounds(las_file: Path) -> Box2154:
    # Open the LAS/LAZ file and read the header to get the bounds
    reader = Reader(str(las_file))
    pipeline = Pipeline([reader])
    pipeline.execute()
    metadata = pipeline.metadata

    las_metadata = metadata["metadata"]
    proj_metadata = None
    if "readers.las" in las_metadata:
        proj_metadata = las_metadata["readers.las"]
    elif "readers.copc" in las_metadata:
        proj_metadata = las_metadata["readers.copc"]
    else:
        raise ValueError(
            f"Could not find LAS/LAZ metadata for file '{las_file}' in PDAL metadata output"
        )

    if (
        "minx" not in proj_metadata
        or "miny" not in proj_metadata
        or "maxx" not in proj_metadata
        or "maxy" not in proj_metadata
    ):
        raise ValueError(
            f"Could not retrieve bounds from LAS/LAZ metadata for file '{las_file}'"
        )
    minx = int(float(proj_metadata["minx"]))
    miny = int(float(proj_metadata["miny"]))
    maxx = int(float(proj_metadata["maxx"]))
    maxy = int(float(proj_metadata["maxy"]))
    return Box2154(Point2154(minx, miny), Point2154(maxx, maxy))


def create_merge_pipeline(input_files: List[Path], output_file: Path) -> Dict:
    """
    Create a PDAL pipeline that merges multiple point cloud files.

    Args:
        input_files: List of input point cloud file paths
        output_file: Output file path

    Returns:
        Dictionary representing the PDAL pipeline
    """
    input_files_str = [str(f) for f in input_files]
    output_file_str = str(output_file)

    # Create pipeline stages
    pipeline = []

    # Add additional files as merge stages (if there are multiple files)
    for input_file_str in input_files_str:
        pipeline.append({"type": "readers.las", "filename": input_file_str})

    pipeline.append(
        {
            "type": "filters.merge",
        }
    )

    # Add writer at the end
    pipeline.append(
        {"type": "writers.las", "filename": output_file_str, "extra_dims": "all"}
    )

    return {"pipeline": pipeline}


def merge_files(
    input_files: List[Path], output_file: Path, input_output: InputOutput
) -> None:
    """
    Merge multiple point cloud files into a single output file.

    Parameters
    ----------
    input_files : List[Path]
        List of input point cloud file paths.
    output_file : Path
        Output point cloud file path.
    input_output : InputOutput
        The handler for input and output file issues.
    """
    message_prefix = "Merge point clouds"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=input_files,
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_file]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    # Create pipeline
    pipeline = create_merge_pipeline(input_files, output_file)
    logging.debug(f"PDAL Pipeline: {json.dumps(pipeline, indent=2)}")

    # Execute pipeline
    pipeline_json = json.dumps(pipeline)
    pdal_pipeline = Pipeline(pipeline_json)
    pdal_pipeline.execute()


def split_point_cloud_implementation(
    input_file: Path,
    output_file_template: Path,
    dimension: str,
    input_output: InputOutput,
) -> List[Path]:
    """
    Split a point cloud file into multiple files based on a specified dimension.

    Parameters
    ----------
    input_file : Path
        Input LAS/LAZ file path.
    output_file_template : Path
        Template for output file paths (should include # placeholder).
    dimension : str
        Dimension to split on (e.g., "Classification").
    input_output: InputOutput
        The handler for input and output file issues.

    Returns
    -------
    List[Path]
        List of paths to the created output files.

    Raises
    ------
    ValueError
        If the output file template doesn't contain a # placeholder.
    """
    message_prefix = "Splitting point cloud"
    input_output.handle_input(message_prefix=message_prefix, input_files=[input_file])

    # Check if the output file template contains the # placeholder
    if "#" not in output_file_template.name:
        raise ValueError(
            "Output file template must contain a # placeholder for the dimension value"
        )

    # Ensure output directory exists
    output_file_template.parent.mkdir(parents=True, exist_ok=True)

    # Create the pipeline
    reader = Reader(str(input_file))
    filter_groupby = Filter("filters.groupby", dimension=dimension)

    processing_pipeline = Pipeline([reader, filter_groupby])

    # Run the pipeline
    logging.info(
        f"Processing input file '{input_file}' and splitting by dimension '{dimension}'"
    )
    logging.debug(
        f"PDAL Pipeline: {json.dumps(processing_pipeline.pipeline, indent=2)}"
    )
    processing_pipeline.execute()
    logging.debug(
        f"Found {len(processing_pipeline.arrays)} different values for dimension '{dimension}'"
    )

    # Find the unique values for the dimension of interest
    dimension_unique_values = [arr[dimension][0] for arr in processing_pipeline.arrays]
    output_files = [
        Path(str(output_file_template).replace("#", f"{dimension_value}"))
        for dimension_value in dimension_unique_values
    ]

    # Check if the output files already exist
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[output_files],
    )
    if output_action == OutputActionEnum.SKIP:
        return output_files

    # Write every output to a separate file
    tqdm_iterable = tqdm(
        zip(processing_pipeline.arrays, dimension_unique_values, output_files),
        total=len(dimension_unique_values),
        desc="Writing output files",
    )
    for arr, dimension_value, output_file in tqdm_iterable:
        tqdm_iterable.set_postfix({dimension: dimension_value})
        tqdm_iterable.refresh()

        # Write the array to the output file
        writer = Writer(
            str(output_file),
            extra_dims="all",
        )
        pipeline_writer = Pipeline([writer], arrays=[arr])
        pipeline_writer.execute()

    return output_files


def split_point_cloud_call(
    input_file: Path,
    output_file_template: Path,
    dimension: str,
    input_output: InputOutput,
    verbose_int: int,
):
    with LoggingContext(verbose=verbose_int):
        output_file_template.parent.mkdir(parents=True, exist_ok=True)

        all_output_files = split_point_cloud_implementation(
            input_file=input_file,
            output_file_template=output_file_template,
            dimension=dimension,
            input_output=input_output,
        )

    return all_output_files


def classification_mapping_implementation(
    input_file: Path,
    output_file: Path,
    mapping: Dict[int, int],
    input_output: InputOutput,
):
    """
    Map the classification values in a LAS/LAZ file to a new set of values based on a provided mapping.

    Parameters
    ----------
    input_file : Path
        Input LAS/LAZ file.
    output_file : Path
        Output LAS/LAZ file.
    mapping : Dict[int, int]
        Mapping of old classification values to new classification values.
    input_output: InputOutput
        The handler for input and output file issues.
    """
    message_prefix = "Reclassifying point cloud"
    input_output.handle_input(message_prefix=message_prefix, input_files=[input_file])
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_file]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    # Create pipeline
    reader = Reader(str(input_file))
    filter_reclassify = Filter(
        "filters.assign",
        value=[
            f"Classification = {new} WHERE Classification == {old}"
            for old, new in mapping.items()
        ],
    )
    writer = Writer(str(output_file), extra_dims="all")

    pipeline = Pipeline([reader, filter_reclassify, writer])
    pipeline.execute()


def classification_mapping_call(
    input_file: Path,
    output_file: Path,
    mapping: Dict[int, int],
    input_output: InputOutput,
    verbose_int: int,
):
    with LoggingContext(verbose=verbose_int):
        classification_mapping_implementation(
            input_file=input_file,
            output_file=output_file,
            mapping=mapping,
            input_output=input_output,
        )
