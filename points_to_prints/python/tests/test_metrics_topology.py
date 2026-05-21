from __future__ import annotations

from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

import geopandas as gpd
from shapely.geometry import Polygon

from python.metrics.metrics import (
    clean_polygon_topology_implementation,
    write_polygon_topology_results,
)


class MetricsTopologyTests(unittest.TestCase):
    def _sample_polygons(self) -> gpd.GeoDataFrame:
        return gpd.GeoDataFrame(
            {
                "cleabs": ["a", "b"],
                "name": ["left", "right"],
                "geometry": [
                    Polygon([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]),
                    Polygon([(1.04, 0.0), (2.04, 0.0), (2.04, 1.0), (1.04, 1.0)]),
                ],
            },
            geometry="geometry",
            crs="EPSG:2154",
        )

    def test_clean_polygon_topology_merges_nearby_vertices_and_preserves_columns(
        self,
    ) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir:
            input_path = Path(temp_dir) / "input.parquet"
            sample_polygons.to_parquet(input_path, index=False)

            cleaned = clean_polygon_topology_implementation(input_path, threshold_m=0.1)

            self.assertEqual(list(cleaned.columns), list(sample_polygons.columns))
            self.assertEqual(cleaned.crs, sample_polygons.crs)
            self.assertTrue(cleaned.geometry.is_valid.all())
            self.assertEqual(cleaned.loc[0, "name"], "left")
            self.assertEqual(cleaned.loc[1, "name"], "right")
            self.assertAlmostEqual(
                cleaned.geometry.iloc[0].distance(cleaned.geometry.iloc[1]),
                0.0,
                places=9,
            )

    def test_write_polygon_topology_results_supports_parquet_and_gpkg(self) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir:
            temp_dir_path = Path(temp_dir)
            for output_suffix in (".parquet", ".gpkg"):
                with self.subTest(output_suffix=output_suffix):
                    output_path = temp_dir_path / f"cleaned{output_suffix}"

                    write_polygon_topology_results(
                        sample_polygons, output_path, overwrite=False
                    )

                    self.assertTrue(output_path.exists())
                    if output_suffix == ".parquet":
                        reloaded = gpd.read_parquet(output_path)
                    else:
                        reloaded = gpd.read_file(output_path)

                    self.assertEqual(
                        list(reloaded.columns), list(sample_polygons.columns)
                    )
                    self.assertEqual(reloaded.crs, sample_polygons.crs)
                    self.assertTrue(reloaded.geometry.is_valid.all())

    def test_write_polygon_topology_results_rejects_unsupported_suffix(self) -> None:
        sample_polygons = self._sample_polygons()
        with TemporaryDirectory() as temp_dir:
            output_path = Path(temp_dir) / "cleaned.geojson"

            with self.assertRaisesRegex(ValueError, "Unsupported output format"):
                write_polygon_topology_results(
                    sample_polygons, output_path, overwrite=False
                )


if __name__ == "__main__":
    unittest.main()
