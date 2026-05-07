from glob import glob
import logging
from pathlib import Path
import subprocess

from ..utils.custom_logging import LoggingContext, run_command_with_tqdm_logging


def roofprints_to_roof_implementation(
    point_cloud_path: Path,
    roofprints_path: Path,
    roof_path: Path,
) -> None:
    """
    Creates a 3D roof model from roofprints and a point cloud.
    This simply calls roofer with the appropriate arguments, and then converts the CityJSONSeq output to CityJSON.

    Args:
        point_cloud_path (Path): path to the point cloud file (LAS/LAZ)
        roofprints_path (Path): path to the roofprints file
        roof_path (Path): CityJSON path where the resulting 3D roof model will be saved
    """

    # --------------------- Create the 3D building models -------------------- #

    roofer_output_path = roof_path.parent / f"{roof_path.stem}_roofer_output"
    # command_roofer = [
    #     "roofer",
    #     str(point_cloud_path),
    #     str(roofprints_path),
    #     str(roofer_output_path),
    #     "--no-clip-terrain",
    # ]

    # return_code = run_command_with_tqdm_logging(command_roofer, display=True)
    # if return_code != 0:
    #     logging.error(f"Failed to create 3D roof model.")
    # else:
    #     logging.info(f"Successfully created 3D roof model.")

    # -------------- Convert the CityJSONSeq outputs to CityJSON ------------- #

    command_cat = ["cat", *glob(str(roofer_output_path / "*.city.jsonl"))]
    command_cjseq = [
        "cjseq",
        "collect",
        ">",
        str(roof_path),
    ]

    ps = subprocess.Popen(command_cat, stdout=subprocess.PIPE)
    output = subprocess.check_output(command_cjseq, stdin=ps.stdout)
    ps.wait()

    # return_code = run_command_with_tqdm_logging(command, display=True)
    # if return_code != 0:
    #     logging.error(f"Failed to convert CityJSONSeq to CityJSON.")
    # else:
    #     logging.info(f"Successfully converted CityJSONSeq to CityJSON.")


def roofprints_to_roof_call(
    point_cloud_path: Path,
    roofprints_path: Path,
    roof_path: Path,
    verbose_int: int = 0,
) -> None:
    with LoggingContext(verbose=verbose_int):
        roof_path.parent.mkdir(parents=True, exist_ok=True)

        roofprints_to_roof_implementation(
            point_cloud_path=point_cloud_path,
            roofprints_path=roofprints_path,
            roof_path=roof_path,
        )
