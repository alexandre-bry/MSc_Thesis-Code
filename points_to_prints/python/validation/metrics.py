from __future__ import annotations

import json
import logging
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path

import geopandas as gpd
import pandas as pd
from shapely import make_valid
from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiLineString,
    MultiPolygon,
    Point,
    Polygon,
)
from shapely.ops import unary_union
from shapely.strtree import STRtree

from ..utils.custom_logging import LoggingContext


@dataclass(slots=True)
class PolygonMetricsResult:
    """Container for the per-pair metrics and global summary.

    Parameters
    ----------
    paired_results : pandas.DataFrame
        GeoDataFrame with per-pair (or per-aggregate) metrics and geometries.
    summary : dict
        Dictionary with aggregated summary statistics (counts, means).
    """

    paired_results: pd.DataFrame
    summary: dict[str, float | int]


def _read_polygon_dataset(input_path: Path) -> gpd.GeoDataFrame:
    suffix = input_path.suffix.lower()
    if suffix == ".parquet":
        return gpd.read_parquet(input_path)
    if suffix in {".gpkg", ".geojson", ".json", ".shp"}:
        return gpd.read_file(input_path)
    raise ValueError(
        f"Unsupported polygon dataset format for {input_path}. Expected .parquet or .gpkg."
    )


def _collect_ground_truth_source_ids(
    ground_truth_dataset: gpd.GeoDataFrame,
    id_column: str,
) -> list:
    def _flatten_source_ids(value):
        if value is None:
            return
        if isinstance(value, str):
            yield value
            return
        if isinstance(value, (list, tuple, set, pd.Series, pd.Index)):
            for item in value:
                yield from _flatten_source_ids(item)
            return
        if hasattr(value, "tolist"):
            try:
                converted_value = value.tolist()
            except Exception:
                converted_value = value
            if converted_value is not value:
                yield from _flatten_source_ids(converted_value)
                return
        yield value

    if _is_aggregated_ground_truth_dataset(ground_truth_dataset):
        source_ids: list = []
        for ground_truth_ids in ground_truth_dataset["ground_truth_ids"]:
            source_ids.extend(_flatten_source_ids(ground_truth_ids))
        return pd.Index(source_ids).dropna().unique().tolist()

    return pd.Index(ground_truth_dataset[id_column]).dropna().unique().tolist()


def _read_filtered_scored_dataset(
    scored_path: Path,
    id_column: str,
    source_ids: list,
) -> gpd.GeoDataFrame:
    suffix = scored_path.suffix.lower()
    if suffix == ".parquet":
        try:
            return gpd.read_parquet(
                scored_path,
                columns=[id_column, "geometry"],
                filters=[(id_column, "in", source_ids)],
            )
        except TypeError:
            scored_dataset = gpd.read_parquet(
                scored_path,
                columns=[id_column, "geometry"],
            )
            return scored_dataset[scored_dataset[id_column].isin(source_ids)]

    scored_dataset = _read_polygon_dataset(scored_path)
    return scored_dataset[scored_dataset[id_column].isin(source_ids)].copy()


def _validate_input_columns(
    dataset: gpd.GeoDataFrame, id_column: str, label: str
) -> None:
    if id_column not in dataset.columns:
        raise ValueError(f"Column '{id_column}' is missing from the {label} dataset.")
    if dataset[id_column].isna().any():
        raise ValueError(
            f"Column '{id_column}' contains missing values in the {label} dataset."
        )
    if dataset[id_column].duplicated().any():
        raise ValueError(
            f"Column '{id_column}' must contain unique IDs in the {label} dataset."
        )
    if dataset.geometry.isna().any():
        raise ValueError(f"The {label} dataset contains missing geometries.")


def _normalize_keep_columns(
    dataset: gpd.GeoDataFrame,
    id_column: str,
    keep_columns: list[str],
) -> list[str]:
    normalized_keep_columns: list[str] = []
    for column_name in keep_columns:
        if column_name == id_column or column_name == dataset.geometry.name:
            continue
        if column_name not in dataset.columns:
            raise ValueError(
                f"Column '{column_name}' is missing from the ground-truth dataset."
            )
        if column_name not in normalized_keep_columns:
            normalized_keep_columns.append(column_name)
    return normalized_keep_columns


def _validate_matching_crs(
    ground_truth_dataset: gpd.GeoDataFrame,
    scored_dataset: gpd.GeoDataFrame,
) -> None:
    if ground_truth_dataset.crs is None:
        raise ValueError("The ground-truth dataset is missing CRS information.")
    if scored_dataset.crs is None:
        raise ValueError("The scored dataset is missing CRS information.")
    if ground_truth_dataset.crs != scored_dataset.crs:
        raise ValueError(
            "The ground-truth and scored datasets must use the same CRS to compare polygons."
        )


def _build_scored_aggregate(
    scored_dataset: gpd.GeoDataFrame,
    source_ids: list,
    id_column: str,
) -> tuple[list, object] | None:
    scored_subset = scored_dataset[scored_dataset[id_column].isin(source_ids)]
    if scored_subset.empty:
        return None
    scored_geometry = unary_union(list(scored_subset.geometry))
    return scored_subset[id_column].tolist(), scored_geometry


def _validate_aggregated_ground_truth_dataset(
    dataset: gpd.GeoDataFrame,
) -> None:
    if "ground_truth_aggregate_id" not in dataset.columns:
        raise ValueError(
            "Column 'ground_truth_aggregate_id' is missing from the aggregated ground-truth dataset."
        )
    if "ground_truth_ids" not in dataset.columns:
        raise ValueError(
            "Column 'ground_truth_ids' is missing from the aggregated ground-truth dataset."
        )
    if dataset["ground_truth_aggregate_id"].isna().any():
        raise ValueError(
            "Column 'ground_truth_aggregate_id' contains missing values in the aggregated ground-truth dataset."
        )
    if dataset["ground_truth_aggregate_id"].duplicated().any():
        raise ValueError(
            "Column 'ground_truth_aggregate_id' must contain unique IDs in the aggregated ground-truth dataset."
        )
    if dataset["ground_truth_ids"].isna().any():
        raise ValueError(
            "Column 'ground_truth_ids' contains missing values in the aggregated ground-truth dataset."
        )
    if dataset.geometry.isna().any():
        raise ValueError(
            "The aggregated ground-truth dataset contains missing geometries."
        )


def _is_aggregated_ground_truth_dataset(dataset: gpd.GeoDataFrame) -> bool:
    return {
        "ground_truth_aggregate_id",
        "ground_truth_ids",
    }.issubset(dataset.columns)


def _ensure_polygonal_geometry(geometry, label: str, row_id) -> None:
    if geometry is None or geometry.is_empty:
        raise ValueError(f"Polygon {row_id!r} in the {label} dataset is empty.")
    if geometry.geom_type not in {"Polygon", "MultiPolygon"}:
        raise ValueError(
            f"Polygon {row_id!r} in the {label} dataset must be a Polygon or MultiPolygon, got {geometry.geom_type}."
        )


def _sample_line_string(line: LineString, spacing_m: float) -> list[Point]:
    if spacing_m <= 0:
        raise ValueError("The sampling distance must be strictly positive.")

    sampled_points: list[Point] = []
    current_distance = 0.0
    line_length = line.length
    if line_length == 0:
        return [Point(line.coords[0])]

    while current_distance < line_length:
        sampled_points.append(line.interpolate(current_distance))
        current_distance += spacing_m

    return sampled_points


def _sample_boundary_points(geometry, spacing_m: float) -> list[Point]:
    boundary = geometry.boundary
    if boundary.is_empty:
        return []
    if isinstance(boundary, Point):
        return [boundary]
    if isinstance(boundary, LineString):
        return _sample_line_string(boundary, spacing_m)
    if isinstance(boundary, MultiLineString):
        sampled_points: list[Point] = []
        for line in boundary.geoms:
            sampled_points.extend(_sample_line_string(line, spacing_m))
        return sampled_points
    if isinstance(boundary, GeometryCollection):
        sampled_points = []
        for geometry_part in boundary.geoms:
            if isinstance(geometry_part, Point):
                sampled_points.append(geometry_part)
            elif isinstance(geometry_part, LineString):
                sampled_points.extend(_sample_line_string(geometry_part, spacing_m))
            elif isinstance(geometry_part, MultiLineString):
                for line in geometry_part.geoms:
                    sampled_points.extend(_sample_line_string(line, spacing_m))
        return sampled_points
    return []


def _directed_boundary_distance(
    source_geometry, target_geometry, spacing_m: float
) -> float:
    sampled_points = _sample_boundary_points(source_geometry, spacing_m)
    if not sampled_points:
        return 0.0

    target_boundary = target_geometry.boundary
    distances = [point.distance(target_boundary) for point in sampled_points]
    return float(sum(distances) / len(distances))


def _intersection_over_union(first_geometry, second_geometry) -> float:
    union_area = first_geometry.union(second_geometry).area
    if union_area == 0:
        return 0.0
    return float(first_geometry.intersection(second_geometry).area / union_area)


def _compare_polygon_pair(
    ground_truth_geometry,
    scored_geometry,
    spacing_m: float,
) -> dict[str, float]:
    ground_truth_to_scored_distance_m = _directed_boundary_distance(
        ground_truth_geometry, scored_geometry, spacing_m
    )
    scored_to_ground_truth_distance_m = _directed_boundary_distance(
        scored_geometry, ground_truth_geometry, spacing_m
    )
    # Centroid distance (in CRS units, typically metres when CRS is metric)
    try:
        gt_centroid = ground_truth_geometry.centroid
        scored_centroid = scored_geometry.centroid
        centroid_distance_m = float(gt_centroid.distance(scored_centroid))
    except Exception:
        centroid_distance_m = 0.0
    return {
        "iou": _intersection_over_union(ground_truth_geometry, scored_geometry),
        "ground_truth_to_scored_boundary_distance_m": ground_truth_to_scored_distance_m,
        "scored_to_ground_truth_boundary_distance_m": scored_to_ground_truth_distance_m,
        "symmetric_boundary_distance_m": (
            ground_truth_to_scored_distance_m + scored_to_ground_truth_distance_m
        )
        / 2.0,
        "centroid_distance_m": centroid_distance_m,
    }


def _coerce_keep_column_values(value) -> list:
    if hasattr(value, "tolist") and not isinstance(value, str):
        try:
            value = value.tolist()
        except Exception:
            pass
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    return [value]


def _polygon_parts(geometry) -> list[Polygon]:
    if geometry is None or geometry.is_empty:
        return []
    if isinstance(geometry, Polygon):
        return [geometry]
    if isinstance(geometry, MultiPolygon):
        return list(geometry.geoms)
    if isinstance(geometry, GeometryCollection):
        polygon_parts: list[Polygon] = []
        for geometry_part in geometry.geoms:
            polygon_parts.extend(_polygon_parts(geometry_part))
        return polygon_parts
    return []


def _unique_closed_ring_coordinates(
    ring_coordinates: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    unique_coordinates: list[tuple[float, float]] = []
    for coordinate in ring_coordinates:
        if not unique_coordinates or coordinate != unique_coordinates[-1]:
            unique_coordinates.append(coordinate)
    if unique_coordinates and unique_coordinates[0] != unique_coordinates[-1]:
        unique_coordinates.append(unique_coordinates[0])
    return unique_coordinates


def _coerce_polygonal_geometry(geometry, label: str, row_id):
    polygon_parts = _polygon_parts(geometry)
    if not polygon_parts:
        raise ValueError(
            f"Polygon {row_id!r} in the {label} dataset could not be repaired into a polygonal geometry."
        )
    if len(polygon_parts) == 1:
        return polygon_parts[0]
    return MultiPolygon(polygon_parts)


def _build_vertex_representatives(
    coordinates: list[tuple[float, float]], threshold_m: float
) -> list[tuple[float, float]]:
    points = [Point(xy) for xy in coordinates]
    spatial_index = STRtree(points)
    threshold_sq = threshold_m * threshold_m

    parent = list(range(len(coordinates)))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left_index: int, right_index: int) -> None:
        left_root = find(left_index)
        right_root = find(right_index)
        if left_root != right_root:
            if left_root < right_root:
                parent[right_root] = left_root
            else:
                parent[left_root] = right_root

    for left_index, left_point in enumerate(points):
        candidate_indices = spatial_index.query(left_point.buffer(threshold_m))
        for candidate_index in candidate_indices:
            right_index = int(candidate_index)
            if right_index <= left_index:
                continue
            right_x, right_y = coordinates[right_index]
            dx = left_point.x - right_x
            dy = left_point.y - right_y
            if dx * dx + dy * dy <= threshold_sq:
                union(left_index, right_index)

    representative_coordinates = list(coordinates)
    cluster_members: dict[int, list[int]] = defaultdict(list)
    for index in range(len(coordinates)):
        cluster_members[find(index)].append(index)

    for members in cluster_members.values():
        representative_index = min(members)
        representative_coordinate = coordinates[representative_index]
        for member_index in members:
            representative_coordinates[member_index] = representative_coordinate

    return representative_coordinates


def _rebuild_polygon_from_vertex_representatives(
    polygon: Polygon,
    representatives: list[tuple[float, float]],
    coordinate_indices: list[int],
    label: str,
    row_id,
):
    exterior_coordinates = [
        representatives[index]
        for index in coordinate_indices[: len(polygon.exterior.coords)]
    ]
    exterior_coordinates = _unique_closed_ring_coordinates(exterior_coordinates)
    if len(exterior_coordinates) < 4:
        raise ValueError(
            f"Polygon {row_id!r} in the {label} dataset collapsed while cleaning topology."
        )

    hole_coordinates: list[list[tuple[float, float]]] = []
    cursor = len(polygon.exterior.coords)
    for interior in polygon.interiors:
        ring_indices = coordinate_indices[cursor : cursor + len(interior.coords)]
        cursor += len(interior.coords)
        ring_coordinates = _unique_closed_ring_coordinates(
            [representatives[index] for index in ring_indices]
        )
        if len(ring_coordinates) >= 4:
            hole_coordinates.append(ring_coordinates)

    rebuilt_polygon = Polygon(exterior_coordinates, hole_coordinates)
    if rebuilt_polygon.is_valid and not rebuilt_polygon.is_empty:
        return rebuilt_polygon

    repaired_polygon = make_valid(rebuilt_polygon)
    repaired_polygon = _coerce_polygonal_geometry(repaired_polygon, label, row_id)
    if not repaired_polygon.is_valid:
        repaired_polygon = _coerce_polygonal_geometry(
            make_valid(repaired_polygon.buffer(0)), label, row_id
        )
    return repaired_polygon


def clean_polygon_topology_implementation(
    input_path: Path,
    threshold_m: float,
) -> gpd.GeoDataFrame:
    """Clean polygon topology by snapping nearby vertices and rebuilding rings.

    Parameters
    ----------
    input_path : Path
        Path to the input polygon dataset (.parquet or .gpkg).
    threshold_m : float
        Distance threshold in metres used to merge nearby vertices.

    Returns
    -------
    geopandas.GeoDataFrame
        A new GeoDataFrame with cleaned polygon geometries.
    """
    if threshold_m <= 0:
        raise ValueError("The merge threshold must be strictly positive.")

    dataset = _read_polygon_dataset(input_path)
    if dataset.crs is None:
        raise ValueError("The input dataset is missing CRS information.")

    polygons_by_feature: list[list[Polygon]] = []
    coordinate_values: list[tuple[float, float]] = []
    coordinate_slices: list[list[int]] = []

    for row_id, geometry in enumerate(dataset.geometry):
        if geometry is None or geometry.is_empty:
            raise ValueError(f"Polygon {row_id!r} in the input dataset is empty.")
        if geometry.geom_type not in {"Polygon", "MultiPolygon"}:
            raise ValueError(
                f"Polygon {row_id!r} in the input dataset must be a Polygon or MultiPolygon, got {geometry.geom_type}."
            )

        feature_polygons = _polygon_parts(geometry)
        if not feature_polygons:
            raise ValueError(
                f"Polygon {row_id!r} in the input dataset could not be read as a polygonal geometry."
            )

        feature_coordinate_indices: list[int] = []
        for polygon in feature_polygons:
            for coordinate in polygon.exterior.coords:
                coordinate_values.append((float(coordinate[0]), float(coordinate[1])))
                feature_coordinate_indices.append(len(coordinate_values) - 1)
            for interior in polygon.interiors:
                for coordinate in interior.coords:
                    coordinate_values.append(
                        (float(coordinate[0]), float(coordinate[1]))
                    )
                    feature_coordinate_indices.append(len(coordinate_values) - 1)

        polygons_by_feature.append(feature_polygons)
        coordinate_slices.append(feature_coordinate_indices)

    if not coordinate_values:
        return dataset.copy()

    representatives = _build_vertex_representatives(coordinate_values, threshold_m)

    cleaned_geometries: list[Polygon | MultiPolygon] = []
    for row_id, feature_polygons in enumerate(polygons_by_feature):
        cleaned_polygons: list[Polygon] = []
        coordinate_cursor = 0
        feature_indices = coordinate_slices[row_id]

        for polygon in feature_polygons:
            ring_coordinate_count = len(polygon.exterior.coords)
            polygon_coordinate_indices = feature_indices[
                coordinate_cursor : coordinate_cursor + ring_coordinate_count
            ]
            coordinate_cursor += ring_coordinate_count

            for interior in polygon.interiors:
                polygon_coordinate_indices.extend(
                    feature_indices[
                        coordinate_cursor : coordinate_cursor + len(interior.coords)
                    ]
                )
                coordinate_cursor += len(interior.coords)

            cleaned_polygon = _rebuild_polygon_from_vertex_representatives(
                polygon,
                representatives,
                polygon_coordinate_indices,
                "input",
                row_id,
            )
            if isinstance(cleaned_polygon, Polygon):
                cleaned_polygons.append(cleaned_polygon)
            else:
                cleaned_polygons.extend(list(cleaned_polygon.geoms))

        if len(cleaned_polygons) == 1:
            cleaned_geometries.append(cleaned_polygons[0])
        else:
            cleaned_geometries.append(MultiPolygon(cleaned_polygons))

    cleaned_dataset = dataset.copy()
    cleaned_dataset.geometry = cleaned_geometries
    return cleaned_dataset


def clean_polygon_topology_call(
    input_path: Path,
    threshold_m: float,
    verbose_int: int,
) -> gpd.GeoDataFrame:
    """Call wrapper for :func:`clean_polygon_topology_implementation`.

    Parameters
    ----------
    input_path : Path
        Path to the input polygon dataset.
    threshold_m : float
        Vertex merge threshold in metres.
    verbose_int : int
        Verbosity level forwarded to the logging context.

    Returns
    -------
    geopandas.GeoDataFrame
        The cleaned polygon dataset.
    """
    with LoggingContext(verbose=verbose_int):
        return clean_polygon_topology_implementation(
            input_path=input_path,
            threshold_m=threshold_m,
        )


def write_polygon_topology_results(
    results: gpd.GeoDataFrame,
    output_path: Path,
    overwrite: bool,
) -> None:
    """Persist cleaned polygon topology results to disk.

    Parameters
    ----------
    results : geopandas.GeoDataFrame
        Cleaned geometries to persist.
    output_path : Path
        Destination path (.parquet or .gpkg supported).
    overwrite : bool
        If True, overwrite existing file.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    suffix = output_path.suffix.lower()
    if output_path.exists() and not overwrite:
        raise FileExistsError(f"Output file already exists: {output_path}")
    if suffix == ".parquet":
        results.to_parquet(
            output_path,
            index=False,
            write_covering_bbox=True,
            schema_version="1.1.0",
        )
        return
    if suffix == ".gpkg":
        results.to_file(output_path, driver="GPKG", index=False)
        return
    raise ValueError("Unsupported output format. Use a .parquet or .gpkg file.")


def _format_summary(summary: dict[str, float | int], output_path: Path) -> str:
    return "\n".join(
        [
            f"Matched polygon pairs: {summary['matched_count']}",
            f"Ignored ground-truth polygons: {summary['ignored_ground_truth_count']}",
            f"Ignored scored polygons: {summary['ignored_scored_count']}",
            f"Mean IoU: {summary['mean_iou']:.6f}",
            (
                "Mean directed boundary distances (m): "
                f"GT->Scored {summary['mean_ground_truth_to_scored_boundary_distance_m']:.6f}, "
                f"Scored->GT {summary['mean_scored_to_ground_truth_boundary_distance_m']:.6f}, "
                f"Symmetric {summary['mean_symmetric_boundary_distance_m']:.6f}"
            ),
            f"Results written to: {output_path}",
        ]
    )


def compare_polygon_datasets_implementation(
    ground_truth_path: Path,
    scored_path: Path,
    output_path: Path,
    id_column: str,
    spacing_m: float,
    keep_columns: list[str] | None = None,
) -> None:
    """Compute metrics against a prebuilt aggregated ground-truth dataset.

    Parameters
    ----------
    ground_truth_path : Path
        Path to the aggregated ground-truth polygon dataset.
    scored_path : Path
        Path to the scored polygon dataset.
    output_path : Path
        Path where the per-pair metrics table will be written (.csv, .parquet or .json).
    id_column : str
        Column name of unique IDs in the scored dataset.
    spacing_m : float
        Sampling spacing in metres used for boundary distances.
    keep_columns : list[str] | None
        Additional columns from the ground-truth dataset to keep in the output.
    """
    ground_truth_dataset = _read_polygon_dataset(ground_truth_path)

    if _is_aggregated_ground_truth_dataset(ground_truth_dataset):
        _validate_aggregated_ground_truth_dataset(ground_truth_dataset)
    else:
        _validate_input_columns(ground_truth_dataset, id_column, "ground-truth")

    source_ids = _collect_ground_truth_source_ids(ground_truth_dataset, id_column)
    scored_dataset = _read_filtered_scored_dataset(scored_path, id_column, source_ids)

    normalized_keep_columns = _normalize_keep_columns(
        ground_truth_dataset, id_column, keep_columns or []
    )

    _validate_input_columns(scored_dataset, id_column, "scored")
    _validate_matching_crs(ground_truth_dataset, scored_dataset)

    rows: list[dict[str, object]] = []
    for _, ground_truth_row in ground_truth_dataset.iterrows():
        if _is_aggregated_ground_truth_dataset(ground_truth_dataset):
            source_ids = ground_truth_row["ground_truth_ids"]
            aggregate_id = ground_truth_row["ground_truth_aggregate_id"]
            ground_truth_area_m2 = float(ground_truth_row["ground_truth_area_m2"])
        else:
            aggregate_id = ground_truth_row[id_column]
            source_ids = [aggregate_id]
            ground_truth_area_m2 = float(ground_truth_row.geometry.area)

        scored_aggregate = _build_scored_aggregate(
            scored_dataset, source_ids, id_column
        )
        if scored_aggregate is None:
            continue

        scored_ids, scored_geometry = scored_aggregate
        metrics = _compare_polygon_pair(
            ground_truth_row.geometry,
            scored_geometry,
            spacing_m,
        )
        row: dict[str, object] = {
            "ground_truth_aggregate_id": aggregate_id,
            "ground_truth_ids": source_ids,
            "scored_ids": scored_ids,
        }
        for column_name in normalized_keep_columns:
            row[column_name] = _coerce_keep_column_values(ground_truth_row[column_name])
        row.update(
            {
                "geometry": scored_geometry,
                "ground_truth_area_m2": ground_truth_area_m2,
                **metrics,
            }
        )
        rows.append(row)

    ordered_columns = [
        "ground_truth_aggregate_id",
        "ground_truth_ids",
        *normalized_keep_columns,
        "scored_ids",
        "geometry",
        "ground_truth_area_m2",
        "iou",
        "ground_truth_to_scored_boundary_distance_m",
        "scored_to_ground_truth_boundary_distance_m",
        "symmetric_boundary_distance_m",
        "centroid_distance_m",
    ]
    paired_results = pd.DataFrame(rows)
    for column_name in ordered_columns:
        if column_name not in paired_results.columns:
            paired_results[column_name] = pd.Series(dtype="object")
    paired_results = paired_results.reindex(columns=ordered_columns)
    paired_results = gpd.GeoDataFrame(
        paired_results,
        geometry="geometry",
        crs=scored_dataset.crs,
    )

    matched_scored_ids = pd.Index(
        [
            scored_id
            for scored_ids in paired_results.get("scored_ids", [])
            for scored_id in (scored_ids if isinstance(scored_ids, list) else [])
        ]
    )

    summary: dict[str, float | int] = {
        "ground_truth_count": len(ground_truth_dataset),
        "scored_count": len(scored_dataset),
        "matched_count": len(paired_results),
        "ignored_ground_truth_count": len(ground_truth_dataset) - len(paired_results),
        "ignored_scored_count": len(scored_dataset)
        - len(pd.Index(scored_dataset[id_column]).intersection(matched_scored_ids)),
    }

    if not paired_results.empty:
        summary.update(
            {
                "mean_iou": float(paired_results["iou"].mean()),
                "mean_ground_truth_to_scored_boundary_distance_m": float(
                    paired_results["ground_truth_to_scored_boundary_distance_m"].mean()
                ),
                "mean_scored_to_ground_truth_boundary_distance_m": float(
                    paired_results["scored_to_ground_truth_boundary_distance_m"].mean()
                ),
                "mean_symmetric_boundary_distance_m": float(
                    paired_results["symmetric_boundary_distance_m"].mean()
                ),
                "mean_centroid_distance_m": float(
                    paired_results["centroid_distance_m"].mean()
                ),
            }
        )
    else:
        summary.update(
            {
                "mean_iou": 0.0,
                "mean_ground_truth_to_scored_boundary_distance_m": 0.0,
                "mean_scored_to_ground_truth_boundary_distance_m": 0.0,
                "mean_symmetric_boundary_distance_m": 0.0,
                "mean_centroid_distance_m": 0.0,
            }
        )

    metrics_result = PolygonMetricsResult(
        paired_results=paired_results, summary=summary
    )

    write_comparison_results(metrics_result.paired_results, output_path)
    logging.info(_format_summary(metrics_result.summary, output_path))


def compare_polygon_datasets_call(
    ground_truth_path: Path,
    scored_path: Path,
    output_path: Path,
    id_column: str,
    spacing_m: float,
    keep_columns: list[str] | None = None,
    verbose_int: int = 0,
) -> None:
    """Entry-point wrapper for dataset comparison with logging context.

    Parameters
    ----------
    ground_truth_path : Path
        Path to the aggregated ground-truth dataset.
    scored_path : Path
        Path to the scored dataset.
    output_path : Path
        Path where the per-pair metrics table will be written (.csv, .parquet or .json).
    id_column : str
        Column name used for matching IDs.
    spacing_m : float
        Sampling spacing in metres.
    keep_columns : list[str] | None
        Additional columns from the ground-truth dataset to keep in the output.
    verbose_int : int
        Verbosity level for logging.
    """
    with LoggingContext(verbose=verbose_int):
        compare_polygon_datasets_implementation(
            ground_truth_path=ground_truth_path,
            scored_path=scored_path,
            output_path=output_path,
            id_column=id_column,
            spacing_m=spacing_m,
            keep_columns=keep_columns,
        )


def write_comparison_results(results: pd.DataFrame, output_path: Path) -> None:
    """Write comparison results to CSV, GeoParquet or JSON.

    Parameters
    ----------
    results : pandas.DataFrame
        DataFrame or GeoDataFrame containing results. If a GeoDataFrame is
        supplied or a `geometry` column exists it will be written as GeoParquet
        when the output extension is ``.parquet``.
    output_path : Path
        Destination path. Supported extensions: ``.csv``, ``.parquet`` and ``.json``.
    """
    output_path.parent.mkdir(parents=True, exist_ok=True)
    suffix = output_path.suffix.lower()
    if suffix == ".csv":
        csv_results = pd.DataFrame(results.copy())
        for column_name in csv_results.columns:
            csv_results[column_name] = csv_results[column_name].apply(
                lambda value: (
                    json.dumps(value)
                    if isinstance(value, (list, tuple, dict))
                    else (value.wkt if hasattr(value, "wkt") else value)
                )
            )
        csv_results.to_csv(output_path, index=False)
        return
    if suffix == ".json":
        json_results = pd.DataFrame(results.copy())

        def _serialize_value(v):
            try:
                if v is None or (isinstance(v, float) and pd.isna(v)):
                    return None
            except Exception:
                pass
            if hasattr(v, "wkt"):
                return v.wkt
            # numpy arrays and pandas arrays -> convert to list
            if hasattr(v, "tolist") and not isinstance(v, str):
                try:
                    lst = v.tolist()
                except Exception:
                    lst = None
                if lst is not None:
                    return [_serialize_value(x) for x in lst]
            if isinstance(v, (list, tuple)):
                return [_serialize_value(x) for x in v]
            if isinstance(v, dict):
                return {k: _serialize_value(val) for k, val in v.items()}
            # numpy scalar -> python native
            if hasattr(v, "item") and not isinstance(v, str):
                try:
                    return v.item()
                except Exception:
                    pass
            return v

        records: list[dict] = []
        for _, row in json_results.iterrows():
            record: dict = {}
            for column_name in json_results.columns:
                record[column_name] = _serialize_value(row[column_name])
            records.append(record)

        with output_path.open("w", encoding="utf-8") as fh:
            json.dump(records, fh, ensure_ascii=False, indent=2)
        return
    if suffix == ".parquet":
        if isinstance(results, gpd.GeoDataFrame):
            geo_results = results
        elif "geometry" in results.columns:
            geo_results = gpd.GeoDataFrame(results, geometry="geometry")
        else:
            results.to_parquet(output_path, index=False)
            return

        geo_results.to_parquet(
            output_path,
            index=False,
            write_covering_bbox=True,
            schema_version="1.1.0",
        )
        return
    raise ValueError("Unsupported output format. Use a .csv or .parquet file.")
