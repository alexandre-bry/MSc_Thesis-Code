import logging
import subprocess
from glob import glob
from pathlib import Path
from tempfile import TemporaryDirectory

from ..utils import (
    InputOutput,
    LoggingContext,
    OutputActionEnum,
    OutputBehaviour,
    Verbose,
    run_command_with_tqdm_logging,
)


def roofprints_to_lod22_implementation(
    point_cloud_path: Path,
    roofprints_path: Path,
    roof_path: Path,
    input_output: InputOutput,
) -> None:
    """
    Creates a 3D roof model from roofprints and a point cloud.
    This simply calls roofer with the appropriate arguments, and then converts the CityJSONSeq output to CityJSON.

    Parameters
    ----------
    point_cloud_path : Path
        Path to the point cloud file (LAS/LAZ).
    roofprints_path : Path
        Path to the roofprints file (Parquet, GeoPackage, Shapefile, ...).
    roof_path : Path
        Path where the resulting 3D roof model will be saved (CityJSON).
    input_output: InputOutput
        The handler for input and output file issues.
    """
    message_prefix = "Roofprints to LoD2.2"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[point_cloud_path, roofprints_path],
    )
    output_action = input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[roof_path]],
    )
    if output_action == OutputActionEnum.SKIP:
        return

    with TemporaryDirectory() as temp_dir:
        # ----------------- Convert roofprints to GeoPackage ----------------- #
        if roofprints_path.suffix.lower() != ".gpkg":
            logging.info(
                f"Converting roofprints from {roofprints_path.suffix} to GeoPackage format..."
            )
            roofprints_gpkg_path = Path(temp_dir) / "roofprints.gpkg"
            command_convert = [
                "gdal",
                "convert",
                "-i",
                str(roofprints_path),
                "-o",
                str(roofprints_gpkg_path),
            ]
            return_code = run_command_with_tqdm_logging(command_convert, display=True)
            if return_code != 0:
                logging.error(f"Failed to convert roofprints to GeoPackage format.")
                return
            else:
                logging.info(f"Successfully converted roofprints to GeoPackage format.")
        else:
            roofprints_gpkg_path = roofprints_path

        # ------------------- Create the 3D building models ------------------ #

        roofer_output_path = Path(temp_dir) / "roofer_output"
        command_roofer = [
            "roofer",
            str(point_cloud_path),
            str(roofprints_gpkg_path),
            str(roofer_output_path),
            "--no-clip-terrain",
            "--id-attribute",
            "cleabs",
        ]

        return_code = run_command_with_tqdm_logging(command_roofer, display=True)
        if return_code != 0:
            logging.error(f"Failed to create 3D roof model.")
        else:
            logging.info(f"Successfully created 3D roof model.")

        # ------------ Convert the CityJSONSeq outputs to CityJSON ----------- #

        command_cat = ["cat", *glob(str(roofer_output_path / "*.city.jsonl"))]
        command_cjseq = ["cjseq", "collect"]

        logging.info(
            f"Running command: {' '.join(command_cat)} | {' '.join(command_cjseq)} > {roof_path}"
        )
        with roof_path.open("wb") as roof_file:
            ps = subprocess.Popen(command_cat, stdout=subprocess.PIPE)
            try:
                result = subprocess.run(
                    command_cjseq, stdin=ps.stdout, stdout=roof_file
                )
                return_code = result.returncode
            finally:
                if ps.stdout is not None:
                    ps.stdout.close()
                ps.wait()

        if return_code != 0:
            logging.error("Failed to convert CityJSONSeq to CityJSON.")
        else:
            logging.info("Successfully converted CityJSONSeq to CityJSON.")


def roofprints_to_lod22_call(
    point_cloud_path: Path,
    roofprints_path: Path,
    roof_path: Path,
    input_output: InputOutput,
    verbose: Verbose,
) -> None:
    """
    Creates a 3D roof model from roofprints and a point cloud.
    This simply calls roofer with the appropriate arguments, and then converts the CityJSONSeq output to CityJSON.

    Parameters
    ----------
    point_cloud_path : Path
        Path to the point cloud file (LAS/LAZ).
    roofprints_path : Path
        Path to the roofprints file (Parquet, GeoPackage, Shapefile, ...).
    roof_path : Path
        Path where the resulting 3D roof model will be saved (CityJSON).
    input_output: InputOutput
        The handler for input and output file issues.
    verbose: Verbose
        The verbosity level for logging.
    """
    with LoggingContext(verbose=verbose):
        roof_path.parent.mkdir(parents=True, exist_ok=True)

        roofprints_to_lod22_implementation(
            point_cloud_path=point_cloud_path,
            roofprints_path=roofprints_path,
            roof_path=roof_path,
            input_output=input_output,
        )
