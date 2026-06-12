from __future__ import annotations

import hashlib
import logging
from pathlib import Path
from typing import Iterable

import geopandas as gpd
import pandas as pd
from shapely.ops import unary_union

from ..utils.custom_logging import LoggingContext, Verbose
from ..utils.input_output import InputOutput, OutputBehaviour
from .metrics import (
    _normalize_keep_columns,
    _read_polygon_dataset,
    _validate_input_columns,
)


def _deterministic_aggregate_id(source_ids: Iterable[object]) -> str:
    sorted_ids = sorted(str(source_id) for source_id in source_ids)
    digest = hashlib.sha1(",".join(sorted_ids).encode("utf-8")).hexdigest()[:12]
    return f"agg_{digest}"


def _build_touching_components(dataset: gpd.GeoDataFrame) -> list[list[int]]:
    if len(dataset) == 0:
        return []

    parent = list(range(len(dataset)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left_index: int, right_index: int) -> None:
        left_root = find(left_index)
        right_root = find(right_index)
        if left_root != right_root:
            parent[right_root] = left_root

    spatial_index = dataset.sindex
    geometries = list(dataset.geometry)

    for left_index, left_geometry in enumerate(geometries):
        try:
            candidate_indices = spatial_index.query(left_geometry, predicate="touches")
        except TypeError:
            candidate_indices = spatial_index.query(left_geometry)
        except AttributeError:
            candidate_indices = range(len(dataset))

        for right_index in candidate_indices:
            if right_index <= left_index:
                continue
            right_geometry = geometries[right_index]
            if left_geometry.touches(right_geometry):
                union(left_index, right_index)

    components: dict[int, list[int]] = {}
    for index in range(len(dataset)):
        root = find(index)
        components.setdefault(root, []).append(index)
    return list(components.values())


def _build_aggregated_ground_truth_rows(
    ground_truth_dataset: gpd.GeoDataFrame,
    id_column: str,
    keep_columns: list[str],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for component_indices in _build_touching_components(ground_truth_dataset):
        subset = ground_truth_dataset.iloc[component_indices]
        dissolved_geometry = unary_union(list(subset.geometry))
        source_ids = subset[id_column].tolist()
        rows.append(
            {
                "ground_truth_aggregate_id": _deterministic_aggregate_id(source_ids),
                "ground_truth_ids": source_ids,
                "geometry": dissolved_geometry,
                "ground_truth_area_m2": float(dissolved_geometry.area),
            }
        )
        for column_name in keep_columns:
            rows[-1][column_name] = subset[column_name].tolist()
    return rows


def _build_aggregated_ground_truth_dataset(
    ground_truth_dataset: gpd.GeoDataFrame,
    id_column: str,
    keep_columns: list[str],
) -> gpd.GeoDataFrame:
    ordered_columns = [
        "ground_truth_aggregate_id",
        "ground_truth_ids",
        *keep_columns,
        "geometry",
        "ground_truth_area_m2",
    ]
    aggregated = pd.DataFrame(
        _build_aggregated_ground_truth_rows(
            ground_truth_dataset, id_column, keep_columns
        )
    )
    for column_name in ordered_columns:
        if column_name not in aggregated.columns:
            aggregated[column_name] = pd.Series(dtype="object")
    aggregated = aggregated.reindex(columns=ordered_columns)
    return gpd.GeoDataFrame(
        aggregated, geometry="geometry", crs=ground_truth_dataset.crs
    )


def _ensure_supported_output_path(output_path: Path) -> None:
    if output_path.suffix.lower() not in {".parquet", ".gpkg"}:
        raise ValueError("Unsupported output format. Use a .parquet or .gpkg file.")


def _write_polygon_dataset(
    dataset: gpd.GeoDataFrame,
    output_path: Path,
    input_output: InputOutput,
) -> None:
    message_prefix = "Writing polygon dataset"
    input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[output_path]],
    )

    _ensure_supported_output_path(output_path)

    if output_path.suffix.lower() == ".parquet":
        dataset.to_parquet(
            output_path,
            index=False,
            write_covering_bbox=True,
            schema_version="1.1.0",
        )
        return
    dataset.to_file(output_path, driver="GPKG", index=False)


def prepare_validation_dataset_implementation(
    input_ground_truth_path: Path,
    id_column: str,
    individual_output_path: Path,
    aggregated_output_path: Path,
    keep_columns: list[str] | None,
    input_output: InputOutput,
) -> None:
    """Persist both the raw validation dataset and the dissolved aggregate dataset."""
    if individual_output_path == aggregated_output_path:
        raise ValueError("The individual and aggregated output paths must differ.")

    message_prefix = "Preparing validation datasets"
    input_output.handle_input(
        message_prefix=message_prefix,
        input_files=[input_ground_truth_path],
    )
    input_output.handle_output(
        message_prefix=message_prefix,
        behaviour=OutputBehaviour.ALL_OR_NOTHING,
        output_files=[[individual_output_path, aggregated_output_path]],
    )

    ground_truth = _read_polygon_dataset(input_ground_truth_path)
    _validate_input_columns(ground_truth, id_column, "ground-truth")
    if ground_truth.crs is None:
        raise ValueError("The ground-truth dataset is missing CRS information.")
    normalized_keep_columns = _normalize_keep_columns(
        ground_truth, id_column, keep_columns or []
    )

    individual_dataset = ground_truth.copy()
    aggregated_dataset = _build_aggregated_ground_truth_dataset(
        ground_truth,
        id_column,
        normalized_keep_columns,
    )

    _write_polygon_dataset(individual_dataset, individual_output_path, input_output)
    _write_polygon_dataset(aggregated_dataset, aggregated_output_path, input_output)

    logging.info(
        "\n".join(
            [
                f"Validation dataset written to: {individual_output_path}",
                f"Aggregated validation dataset written to: {aggregated_output_path}",
            ]
        )
    )


def prepare_validation_dataset_call(
    input_ground_truth_path: Path,
    id_column: str,
    individual_output_path: Path,
    aggregated_output_path: Path,
    keep_columns: list[str] | None,
    input_output: InputOutput,
    verbose: Verbose,
):
    with LoggingContext(verbose=verbose):
        prepare_validation_dataset_implementation(
            input_ground_truth_path=input_ground_truth_path,
            id_column=id_column,
            individual_output_path=individual_output_path,
            aggregated_output_path=aggregated_output_path,
            keep_columns=keep_columns,
            input_output=input_output,
        )
