# Scripts

This file contains useful scripts in command line.

## Convert the BD TOPO to Parquet

1. Rename the geometry layer from `geometrie` to `geom`

    ```bash
    pixi run ogr2ogr -overwrite ../data/BD_TOPO/2025-12-15-renamed.gpkg ../data/BD_TOPO/2025-12-15.gpkg -nln batiment -nlt GEOMETRY -sql "SELECT geometrie AS geom, * FROM batiment"
    ```

2. Convert to GeoParquet with geoparquet-io:

    ```bash
    gpio convert ../data/BD_TOPO/2025-12-15-renamed.gpkg ../data/BD_TOPO/2025-12-15.parquet
    ```
