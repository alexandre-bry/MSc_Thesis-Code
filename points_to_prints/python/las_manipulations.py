"""
Merge multiple point cloud files into a single output file while preserving all attributes.
Uses PDAL to perform the merge operation.
"""

import json
from pathlib import Path
from typing import List, Dict
from pdal import Pipeline, Reader, Filter, Writer
import logging

from tqdm import tqdm


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


def merge_files(input_files: List[Path], output_file: Path) -> None:
    """
    Merge multiple point cloud files into a single output file.

    Args:
        input_files: List of input point cloud file paths
        output_file: Output file path

    Raises:
        FileNotFoundError: If any input file doesn't exist
        ValueError: If input_files is empty
    """
    if not input_files:
        raise ValueError("At least one input file must be provided")

    # Validate input files exist
    input_paths = [Path(f) for f in input_files]
    for p in input_paths:
        if not p.exists():
            raise FileNotFoundError(f"Input file not found: {p}")

    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Create pipeline
    pipeline = create_merge_pipeline(input_files, output_file)
    logging.debug(f"PDAL Pipeline: {json.dumps(pipeline, indent=2)}")

    # Execute pipeline
    pipeline_json = json.dumps(pipeline)
    pdal_pipeline = Pipeline(pipeline_json)
    pdal_pipeline.execute()


def create_split_pipeline(
    input_file: Path, output_file_template: Path, dimension: str
) -> Pipeline:
    """
    Create a PDAL pipeline that splits a point cloud file based on a specified dimension.

    Args:
        input_file: Input point cloud file path
        output_file_template: Template for output file paths (should include # placeholder)
        dimension: Dimension to split on (e.g., "Classification")

    Returns:
        Dictionary representing the PDAL pipeline
    """
    input_file_str = str(input_file)
    output_file_template_str = str(output_file_template)

    reader = Reader(input_file_str)
    filter_groupby = Filter("filters.groupby", dimension=dimension)

    pipeline = Pipeline([reader, filter_groupby])

    # pipeline = {"pipeline": [
    #     {"type": "readers.las", "filename": input_file_str},
    #     {
    #         "type": "filters.groupby",
    #         "dimension": dimension,
    #     },
    #     {
    #         "type": "writers.las",
    #         "filename": output_file_template_str,
    #         "extra_dims": "all",
    #     },
    # ]
    # }

    return pipeline


def split_file(
    input_file: Path,
    output_file_template: Path,
    dimension: str,
    use_value_in_filename: bool,
    overwrite: bool,
) -> None:
    """
    Split a point cloud file into multiple files based on a specified dimension.

    Args:
        input_file: Input point cloud file path
        output_file_template: Template for output file paths (should include # placeholder)
        dimension: Dimension to split on (e.g., "Classification")
        use_value_in_filename: Whether to use the dimension value in the filename

    Raises:
        FileNotFoundError: If the input file doesn't exist
    """
    if not input_file.exists():
        raise FileNotFoundError(f"Input file not found: {input_file}")

    # Check if the output file template contains the # placeholder
    if "#" not in output_file_template.name:
        raise ValueError(
            "Output file template must contain a # placeholder for the dimension value"
        )

    # Check if the output directory already contains files that match the output file template pattern
    if not overwrite:
        # Find the values for the dimension in the input file to determine the expected output files
        reader = Reader(str(input_file))
        reader_pipeline = Pipeline([reader])
        reader_pipeline.execute()
        dimension_values = set(reader_pipeline.arrays[0][dimension])

        # Check if any of the expected output files already exist
        already_existing_files = []
        for value in dimension_values:
            expected_output_file = output_file_template.with_name(
                output_file_template.name.replace("#", f"{value}")
            )
            if expected_output_file.exists():
                already_existing_files.append(expected_output_file)

        if len(already_existing_files) > 0:
            raise FileExistsError(
                f"Output files already exist for the following dimension values: {list(map(str, already_existing_files))}.\nUse --overwrite to overwrite them."
            )

    # Ensure output directory exists
    output_file_template.parent.mkdir(parents=True, exist_ok=True)

    # Create the pipeline
    reader = Reader(str(input_file))
    filter_groupby = Filter("filters.groupby", dimension=dimension)

    processing_pipeline = Pipeline([reader, filter_groupby])

    # Run the pipeline
    processing_pipeline.execute()
    logging.debug(
        f"Found {len(processing_pipeline.arrays)} different values for dimension '{dimension}'"
    )

    # Write every output to a separate file
    tqdm_iterable = tqdm(
        processing_pipeline.arrays,
        total=len(processing_pipeline.arrays),
        desc="Writing output files",
    )
    for arr in tqdm_iterable:
        dimension_value = arr[dimension][0]
        tqdm_iterable.set_postfix({dimension: dimension_value})
        tqdm_iterable.refresh()
        writer = Writer(
            str(output_file_template).replace("#", f"{dimension_value}"),
            extra_dims="all",
        )
        pipeline_writer = Pipeline([writer], arrays=[arr])
        pipeline_writer.execute()
