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

## Identify groups of touching outlines in BD TOPO with DuckDB

```bash
duckdb data/database.db --storage-version v1.5.0
```

Load the spatial extension:

```sql
INSTALL spatial;
LOAD spatial;
```

Load the BD TOPO data:

```sql
CREATE TABLE bd_topo AS (SELECT * FROM 'data/input/BD_TOPO/2025-12-15.parquet');
```

Compute the intersections between all buildings:

```sql
CREATE TABLE intersecting_pairs AS (
    SELECT
        a.cleabs AS cleabs_1,
        b.cleabs AS cleabs_2
    FROM bd_topo a
    JOIN bd_topo b
        ON a.cleabs < b.cleabs
    AND ST_Intersects(a.geometry, b.geometry)
);
```

Make a directed edge table:

```sql
CREATE TABLE edges AS
SELECT cleabs_1 AS src, cleabs_2 AS dst
FROM intersecting_pairs
UNION ALL
SELECT cleabs_2 AS src, cleabs_1 AS dst
FROM intersecting_pairs;
```

Build the reachable pairs:

```sql
CREATE TABLE reach AS
WITH RECURSIVE reach(root, node) AS (
    SELECT src AS root, src AS node
    FROM edges

    UNION

    SELECT r.root, e.dst
    FROM reach r
    JOIN edges e
        ON e.src = r.node
)
SELECT * FROM reach;
```

Assign one root per node:

```sql
CREATE TABLE components AS
SELECT
  node,
  MIN(root) AS root_cleabs
FROM reach
GROUP BY node;
```

Add all the isolated buildings:

```sql
INSERT INTO components (node, root_cleabs)
SELECT cleabs, cleabs
FROM bd_topo
WHERE cleabs NOT IN (SELECT node FROM components);
```

Build the grouped lists:

```sql
CREATE TABLE grouped_components AS
SELECT
  root_cleabs,
  list(node ORDER BY node) AS connected_cleabs
FROM components
GROUP BY root_cleabs
ORDER BY root_cleabs;
```

Add a group ID column to the `bd_topo` table:

```sql
ALTER TABLE bd_topo ADD COLUMN group_id VARCHAR;
```

Set the group ID for each building:

```sql
UPDATE bd_topo AS b1
SET group_id = b3.group_id
FROM (SELECT
    b2.cleabs,
    COALESCE(c.root_cleabs, b2.cleabs) AS group_id
FROM bd_topo b2
LEFT JOIN components c
    ON b2.cleabs = c.node
) AS b3
WHERE b1.cleabs = b3.cleabs; 
```

Export everything:

```sql
COPY (SELECT * FROM bd_topo)
TO 'data/bd_topo_grouped.parquet'
(FORMAT parquet, COMPRESSION zstd, ROW_GROUP_SIZE 100_000);
```

## Unnest the MultiPolygonZ geometries in the BD TOPO with DuckDB

Make a test table with only 10 rows:

```sql
CREATE TABLE test_unnest AS (SELECT cleabs, geometry, group_id FROM bd_topo);
SELECT * FROM test_unnest;
```

or make a test table with good examples:

```sql
CREATE TABLE test_unnest (
    cleabs VARCHAR,
    geometry GEOMETRY('EPSG:2154'),
    group_id VARCHAR
);
INSERT INTO test_unnest (cleabs, geometry, group_id) VALUES
    ('1', 'MULTIPOLYGONZ ( ( (0 0 2, 0 3 2, 3 3 2, 3 0 2, 0 0 2), (1 1 2, 1 2 2, 2 2 2, 2 1 2, 1 1 2) ) )'::geometry, 'group1'),
    ('2', 'MULTIPOLYGONZ ( ( (0 0 2, 0 3 2, 3 3 2, 3 0 2, 0 0 2) ), ( (1 1 2, 1 2 2, 2 2 2, 2 1 2, 1 1 2) ) )'::geometry, 'group2');
SELECT * FROM test_unnest;
```

Unnest the MultiPolygonZ into PolygonZ:

```sql
CREATE TABLE test_unnest_polygons AS
SELECT cleabs, path[1] - 1 AS idx_polygon, geom AS geometry FROM (
    SELECT cleabs, UNNEST(ST_Dump(geometry), recursive := true)
    FROM test_unnest
    );
SELECT * FROM test_unnest_polygons;
```

Unnest the PolygonZ into LinearRingZ:

```sql
CREATE TABLE test_unnest_linearrings AS
SELECT 
  cleabs,
  idx_polygon,
  idx_ring AS idx_ring,
  CASE 
    WHEN idx_ring = 0 THEN ST_ExteriorRing(geometry)
    ELSE ST_InteriorRingN(geometry, idx_ring)
  END AS geometry
FROM test_unnest_polygons
CROSS JOIN generate_series(0, ST_NInteriorRings(geometry)) AS i(idx_ring);
SELECT * FROM test_unnest_linearrings;
```

Unnest the LinearRingZ into PointZ:

```sql
CREATE TABLE test_unnest_points AS
SELECT
  cleabs,
  idx_polygon,
  idx_ring,
  idx_point-1 AS idx_point,
  ST_PointN(geometry, idx_point::INTEGER) AS geometry
FROM test_unnest_linearrings
CROSS JOIN generate_series(1, ST_NPoints(geometry)) AS i(idx_point);
SELECT * FROM test_unnest_points ORDER BY cleabs, idx_polygon, idx_ring, idx_point;
```

Pack the indices:

```sql
CREATE TABLE test_unnest_points_packed AS
SELECT
    struct_pack(cleabs:= cleabs, idx_polygon:= idx_polygon, idx_ring:= idx_ring, idx_point:= idx_point) AS idx,
    geometry
FROM test_unnest_points
ORDER BY cleabs, idx_polygon, idx_ring, idx_point;
```

Remove all tables:

```sql
DROP TABLE test_unnest_points_packed;
DROP TABLE test_unnest_points;
DROP TABLE test_unnest_linearrings;
DROP TABLE test_unnest_polygons;
DROP TABLE test_unnest;
```
