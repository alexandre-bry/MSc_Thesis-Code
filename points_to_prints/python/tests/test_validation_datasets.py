from __future__ import annotations

import unittest
from pathlib import Path
from tempfile import TemporaryDirectory

import geopandas as gpd
from shapely.geometry import Polygon

from python.validation import (
    compare_polygon_datasets_call,
    prepare_validation_dataset_implementation,
)


class ValidationDatasetTests(unittest.TestCase):
    def _sample_polygons(self) -> gpd.GeoDataFrame:
        return gpd.GeoDataFrame(
            {
                "cleabs": ["a", "b", "c"],
                "name": ["left", "middle", "right"],
                "geometry": [
                    Polygon([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]),
                    Polygon([(1.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0)]),
                    Polygon([(10.0, 0.0), (11.0, 0.0), (11.0, 1.0), (10.0, 1.0)]),
                ],
            },
            geometry="geometry",
            crs="EPSG:2154",
        )

    def _normalize_sequence_values(self, values):
        return [
            list(value) if hasattr(value, "tolist") else list(value) for value in values
        ]

    def test_build_validation_datasets_persists_individual_and_aggregated_outputs(
        self,
    ) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir:
            temp_dir_path = Path(temp_dir)
            input_path = temp_dir_path / "input.parquet"
            individual_path = temp_dir_path / "individual.parquet"
            aggregated_path = temp_dir_path / "aggregated.parquet"
            sample_polygons.to_parquet(input_path, index=False)

            prepare_validation_dataset_implementation(
                input_ground_truth_path=input_path,
                id_column="cleabs",
                individual_output_path=individual_path,
                aggregated_output_path=aggregated_path,
                keep_columns=["name"],
                overwrite=False,
            )

            individual_dataset = gpd.read_parquet(individual_path)
            aggregated_dataset = gpd.read_parquet(aggregated_path)

            self.assertEqual(
                list(individual_dataset.columns), list(sample_polygons.columns)
            )
            self.assertEqual(individual_dataset.crs, sample_polygons.crs)
            self.assertEqual(list(individual_dataset["cleabs"]), ["a", "b", "c"])

            self.assertEqual(
                list(aggregated_dataset.columns),
                [
                    "ground_truth_aggregate_id",
                    "ground_truth_ids",
                    "name",
                    "geometry",
                    "ground_truth_area_m2",
                ],
            )
            self.assertEqual(aggregated_dataset.crs, sample_polygons.crs)
            self.assertEqual(len(aggregated_dataset), 2)

            self.assertEqual(
                self._normalize_sequence_values(aggregated_dataset["name"]),
                [["left", "middle"], ["right"]],
            )

            aggregate_id_by_source_ids = {
                tuple(row.ground_truth_ids): row.ground_truth_aggregate_id
                for row in aggregated_dataset.itertuples(index=False)
            }
            self.assertIn(("a", "b"), aggregate_id_by_source_ids)
            self.assertIn(("c",), aggregate_id_by_source_ids)
            self.assertTrue(
                all(
                    value.startswith("agg_")
                    for value in aggregate_id_by_source_ids.values()
                )
            )

    def test_build_validation_datasets_is_deterministic_across_runs(self) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir_one, TemporaryDirectory() as temp_dir_two:
            first_dir = Path(temp_dir_one)
            second_dir = Path(temp_dir_two)
            first_input = first_dir / "input.parquet"
            second_input = second_dir / "input.parquet"
            first_individual = first_dir / "individual.parquet"
            first_aggregated = first_dir / "aggregated.parquet"
            second_individual = second_dir / "individual.parquet"
            second_aggregated = second_dir / "aggregated.parquet"

            sample_polygons.to_parquet(first_input, index=False)
            sample_polygons.to_parquet(second_input, index=False)

            prepare_validation_dataset_implementation(
                input_ground_truth_path=first_input,
                id_column="cleabs",
                individual_output_path=first_individual,
                aggregated_output_path=first_aggregated,
            )
            prepare_validation_dataset_implementation(
                input_ground_truth_path=second_input,
                id_column="cleabs",
                individual_output_path=second_individual,
                aggregated_output_path=second_aggregated,
            )

            first_aggregated_dataset = gpd.read_parquet(first_aggregated)
            second_aggregated_dataset = gpd.read_parquet(second_aggregated)

            first_mapping = {
                tuple(row.ground_truth_ids): row.ground_truth_aggregate_id
                for row in first_aggregated_dataset.itertuples(index=False)
            }
            second_mapping = {
                tuple(row.ground_truth_ids): row.ground_truth_aggregate_id
                for row in second_aggregated_dataset.itertuples(index=False)
            }

            self.assertEqual(first_mapping, second_mapping)

    def test_compare_polygon_datasets_uses_persisted_aggregated_validation_dataset(
        self,
    ) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir:
            temp_dir_path = Path(temp_dir)
            input_path = temp_dir_path / "input.parquet"
            individual_path = temp_dir_path / "individual.parquet"
            aggregated_path = temp_dir_path / "aggregated.parquet"
            scored_path = temp_dir_path / "scored.parquet"
            sample_polygons.to_parquet(input_path, index=False)

            prepare_validation_dataset_implementation(
                input_ground_truth_path=input_path,
                id_column="cleabs",
                individual_output_path=individual_path,
                aggregated_output_path=aggregated_path,
                keep_columns=["name"],
            )
            sample_polygons.to_parquet(scored_path, index=False)

            result = compare_polygon_datasets_call(
                ground_truth_path=aggregated_path,
                scored_path=scored_path,
                id_column="cleabs",
                spacing_m=1.0,
                keep_columns=["name"],
                verbose_int=0,
            )

            self.assertEqual(result.summary["ground_truth_count"], 2)
            self.assertEqual(result.summary["scored_count"], 3)
            self.assertEqual(result.summary["matched_count"], 2)
            self.assertEqual(result.summary["ignored_ground_truth_count"], 0)
            self.assertEqual(result.summary["ignored_scored_count"], 0)
            self.assertTrue((result.paired_results["iou"] == 1.0).all())
            self.assertEqual(
                list(result.paired_results["ground_truth_aggregate_id"]),
                list(gpd.read_parquet(aggregated_path)["ground_truth_aggregate_id"]),
            )
            self.assertEqual(
                self._normalize_sequence_values(result.paired_results["name"]),
                [["left", "middle"], ["right"]],
            )

    def test_compare_polygon_datasets_uses_persisted_individual_validation_dataset(
        self,
    ) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir:
            temp_dir_path = Path(temp_dir)
            input_path = temp_dir_path / "input.parquet"
            individual_path = temp_dir_path / "individual.parquet"
            aggregated_path = temp_dir_path / "aggregated.parquet"
            scored_path = temp_dir_path / "scored.parquet"
            sample_polygons.to_parquet(input_path, index=False)

            prepare_validation_dataset_implementation(
                input_ground_truth_path=input_path,
                id_column="cleabs",
                individual_output_path=individual_path,
                aggregated_output_path=aggregated_path,
            )
            sample_polygons.to_parquet(scored_path, index=False)

            result = compare_polygon_datasets_call(
                ground_truth_path=individual_path,
                scored_path=scored_path,
                id_column="cleabs",
                spacing_m=1.0,
                keep_columns=["name"],
                verbose_int=0,
            )

            self.assertEqual(result.summary["ground_truth_count"], 3)
            self.assertEqual(result.summary["scored_count"], 3)
            self.assertEqual(result.summary["matched_count"], 3)
            self.assertEqual(result.summary["ignored_ground_truth_count"], 0)
            self.assertEqual(result.summary["ignored_scored_count"], 0)
            self.assertTrue((result.paired_results["iou"] == 1.0).all())
            self.assertEqual(
                list(result.paired_results["ground_truth_ids"]),
                [["a"], ["b"], ["c"]],
            )
            self.assertEqual(
                list(result.paired_results["name"]),
                [["left"], ["middle"], ["right"]],
            )


if __name__ == "__main__":
    unittest.main()
