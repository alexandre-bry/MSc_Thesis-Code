# Scripts

This file contains useful scripts in command line.

## Convert the BD TOPO to Parquet

1. Rename the geometry layer from `geometrie` to `geom`

    ```bash
    pixi run -e cpp ogr2ogr -overwrite ../data/BD_TOPO/2025-12-15-renamed.gpkg ../data/BD_TOPO/2025-12-15.gpkg -nln batiment -nlt GEOMETRY -sql "SELECT geometrie AS geom, * FROM batiment"
    ```

2. Convert to GeoParquet with geoparquet-io:

    ```bash
    gpio convert ../data/BD_TOPO/2025-12-15-renamed.gpkg ../data/BD_TOPO/2025-12-15.parquet
    ```

## Clip the BD TOPO without clipping buildings

```bash
pixi run -e cpp gdal vector filter data/BD_TOPO/2025-12-15.gpkg output/bd_topo/2025-12-15.shp --bbox 676000,6852000,677000,6853000
```
