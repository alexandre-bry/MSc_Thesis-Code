import asyncio
import json
import logging
import subprocess
from enum import Enum
from pathlib import Path
from typing import Annotated, List, Literal, Tuple

import laspy
import numpy as np
import pdal
import typer
from geopandas import GeoDataFrame
from tqdm import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm


class Verbose(Enum):
    Error = logging.ERROR
    Warning = logging.WARNING
    Info = logging.INFO
    Debug = logging.DEBUG

    @classmethod
    def from_int(cls, verbose_int: int):
        match verbose_int:
            case 0:
                return cls.Error
            case 1:
                return cls.Warning
            case 2:
                return cls.Info
            case 3:
                return cls.Debug
            case _:
                raise RuntimeError("Verbose has only 4 possible values.")


def setup_logging(verbose: Verbose):
    logging.basicConfig(
        level=verbose.value,
        format="%(asctime)s - %(levelname)s - %(message)s",
    )
    return verbose != Verbose.Error


app = typer.Typer()


def iterate_features(file_path: Path):
    gdf = GeoDataFrame.from_file(file_path)
    for _, feature in gdf.iterrows():
        yield feature


def clip_point_cloud(
    laz_file: Path,
    output_file: Path,
    polygon_wkt: str,
):
    pipeline_json = {
        "pipeline": [
            str(laz_file),
            {
                "type": "filters.crop",
                "polygon": polygon_wkt,
            },
            {
                "type": "writers.las",
                "filename": str(output_file),
                "extra_dims": "all",
            },
        ]
    }
    logging.debug(f"PDAL Pipeline: {json.dumps(pipeline_json, indent=2)}")
    pipeline = pdal.Pipeline(json.dumps(pipeline_json))
    pipeline.execute()


@app.command("extract_buildings")
def extract_buildings(
    laz_file: Annotated[
        Path,
        typer.Option("-l", "--laz_file", help="Path to the .laz file", exists=True),
    ],
    buildings_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--buildings_file",
            help="Path to the buildings file.",
            exists=True,
        ),
    ],
    output_dir: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_dir",
            help="Directory to save the output files.",
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
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        output_dir.mkdir(parents=True, exist_ok=True)

        for feature in tqdm(
            iterate_features(buildings_file),
            desc="Processing buildings",
            total=sum(1 for _ in iterate_features(buildings_file)),
        ):
            f_id = feature["ID"]

            # Extract the geometry and add a buffer around it
            f_wkt_buffered = (
                feature["geometry"].buffer(2, join_style="mitre", mitre_limit=5).wkt
            )
            logging.debug(f"{f_wkt_buffered = }")

            output_file = output_dir / f"building_{f_id}.laz"
            if output_file.exists() and not overwrite:
                logging.info(f"File {output_file} already exists. Skipping...")
                continue
            logging.info(f"Clipping building {f_id} to {output_file}...")
            clip_point_cloud(
                laz_file=laz_file,
                output_file=output_file,
                polygon_wkt=f_wkt_buffered,
            )
            logging.info(f"Finished clipping building {f_id}.")

            exit(0)


SELECTED_CLASSIFICATIONS = [
    0,  # Never Classified
    1,  # Unclassified
    2,  # Ground
    6,  # Building
]


@app.command("distances_in_order")
def distances_in_order(
    laz_file: Annotated[
        Path,
        typer.Option("-l", "--laz_file", help="Path to the .laz file.", exists=True),
    ],
    output_dir: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_dir",
            help="Directory to save the output files.",
        ),
    ],
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        output_dir.mkdir(parents=True, exist_ok=True)

        las = laspy.read(laz_file)

        logging.info(f"Total number of points: {len(las):_}")

        logging.info(f"Filtering points by classification...")
        classifications: np.ndarray = las.classification  # type: ignore
        classification_mask = np.isin(classifications, SELECTED_CLASSIFICATIONS)
        las = las[classification_mask]
        logging.info(f"Number of points after classification filter: {len(las):_}")

        logging.info(f"Sorting points by GPS time...")
        gps_times: np.ndarray = las.gps_time  # type: ignore
        gps_time_order = np.argsort(gps_times)
        las = las[gps_time_order]

        logging.info(f"Computing distances between successive points...")
        # Get all the point source ids
        point_source_ids: np.ndarray = las.point_source_id  # type: ignore
        unique_ids = np.unique(point_source_ids)

        for unique_id in unique_ids:
            mask = point_source_ids == unique_id
            las_subset = las[mask]
            points_subset = las_subset.xyz

            # Compute the distance between successive points
            logging.debug(f"Points subset shape: {points_subset.shape}")
            distances_successive_subset = np.linalg.norm(
                points_subset[1:] - points_subset[:-1], axis=1
            )

            # Remove distances when GPS time difference is too large
            gps_times_subset: np.ndarray = las_subset.gps_time  # type: ignore
            gps_time_diffs_subset = gps_times_subset[1:] - gps_times_subset[:-1]
            large_time_diff_indices_subset = np.where(gps_time_diffs_subset > 10e-4)[0]
            distances_successive_subset[large_time_diff_indices_subset] = 0

            # Remove distances higher than 50 meters
            large_distance_indices_subset = np.where(distances_successive_subset > 50)[
                0
            ]
            distances_successive_subset[large_distance_indices_subset] = 0

            # Create arrays for distances to next and previous points
            distances_to_next_subset = np.hstack((distances_successive_subset, [0]))
            distances_to_prev_subset = np.hstack(([0], distances_successive_subset))

            new_las_subset = laspy.create(
                point_format=las_subset.point_format,
                file_version=str(las_subset.header.version),
            )

            for dim in las_subset.point_format.dimensions:
                new_las_subset[dim.name] = las_subset[dim.name]

            new_las_subset.add_extra_dim(
                laspy.ExtraBytesParams(
                    name="DistanceToNext",
                    type="float64",
                    description="Distance to the next point.",
                )
            )
            new_las_subset.add_extra_dim(
                laspy.ExtraBytesParams(
                    name="DistanceToPrev",
                    type="float64",
                    description="Distance to the previous point.",
                )
            )
            new_las_subset.DistanceToNext = distances_to_next_subset
            new_las_subset.DistanceToPrev = distances_to_prev_subset

            new_las_subset.write(str(output_dir / f"distances_psid_{unique_id}.laz"))


if __name__ == "__main__":
    app()
