# Scripts

This file contains useful scripts in command line.

## Convert LAZ to EPT to open in QGIS

```bash
pixi run entwine build -i <input_laz> -o <output_ept_folder> --deep --srs EPSG:2154
```

## Convert the BD TOPO to Parquet

1. Rename the geometry layer from `geometrie` to `geom`

    ```bash
    pixi run ogr2ogr -overwrite data/input/BD_TOPO/2026-03-15-renamed.gpkg data/input/BD_TOPO/2026-03-15.gpkg -nln batiment -nlt GEOMETRY -sql "SELECT ST_ForcePolygonCCW(geometrie) AS geom, * FROM batiment"
    ```

2. Extract an integer ID from the `cleabs` column (which is a string) and put it in a new `cleabs_int` column:

    ```bash
    pixi run duckdb -c "INSTALL spatial; LOAD spatial; COPY (SELECT CAST(array_slice(cleabs, 9, -1) AS INT64) AS cleabs_int, * FROM 'data/input/BD_TOPO/2026-03-15-renamed.gpkg' ) TO 'data/input/BD_TOPO/2026-03-15-renamed_processed.parquet';"
    ```

3. Convert to GeoParquet with geoparquet-io:

    ```bash
    pixi run gpio convert data/input/BD_TOPO/2026-03-15-renamed_processed.parquet data/input/BD_TOPO/2026-03-15.parquet
    ```

## Clip the BD TOPO without clipping buildings

```bash
pixi run -e cpp gdal vector filter data/BD_TOPO/2026-03-15.gpkg output/bd_topo/2026-03-15.shp --bbox 676000,6852000,677000,6853000
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
CREATE TABLE bd_topo AS (SELECT * FROM 'data/input/BD_TOPO/2026-03-15.parquet');
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
CREATE SCHEMA IF NOT EXISTS test;
```

```sql
CREATE TABLE test.multipoly AS (SELECT cleabs, geometry FROM bd_topo) LIMIT 10;
SELECT * FROM test.multipoly;
```

```sql
-- Unnest the MultiPolygonZ into PolygonZ:
CREATE TABLE test.poly AS
SELECT cleabs, path[1] - 1 AS idx_polygon, geom AS geometry FROM (
    SELECT cleabs, UNNEST(ST_Dump(geometry), recursive := true)
    FROM test.multipoly
    );
-- Show the polygons:
SELECT * FROM test.poly;
```

```sql
-- Unnest the PolygonZ into LinearRingZ:
CREATE TABLE test.rings AS
SELECT 
  cleabs,
  idx_polygon,
  idx_ring AS idx_ring,
  CASE 
    WHEN idx_ring = 0 THEN ST_ExteriorRing(geometry)
    ELSE ST_InteriorRingN(geometry, idx_ring)
  END AS geometry
FROM test.poly
CROSS JOIN generate_series(0, ST_NInteriorRings(geometry)) AS i(idx_ring);
-- Show the rings:
SELECT * FROM test.rings;
```

```sql
-- Unnest the LinearRingZ into LineStringZ with two points per row:
CREATE TABLE test.edges AS
SELECT
  cleabs,
  idx_polygon,
  idx_ring,
  idx_point-1 AS idx_edge,
  ST_MakeLine(ST_PointN(geometry, idx_point::INTEGER), ST_PointN(geometry, (idx_point+1)::INTEGER)) AS geometry
FROM test.rings
CROSS JOIN generate_series(1, ST_NPoints(geometry)-1) AS i(idx_point)
ORDER BY cleabs, idx_polygon, idx_ring, idx_edge;
-- Add an incremental index to the edges:
CREATE SEQUENCE seq_edges_ids START 1;
ALTER TABLE test.edges ADD COLUMN id INTEGER;
UPDATE test.edges SET id = nextval('seq_edges_ids');
-- Show the edges:
SELECT * FROM test.edges;
```

```sql
-- Compute the intersections between all edges from different polygons:
CREATE TABLE test.intersections AS
SELECT
  a.id AS id_a,
  b.id AS id_b,
  ST_Intersection(a.geometry, b.geometry) AS geometry
FROM test.edges a
JOIN test.edges b
  ON a.cleabs < b.cleabs
 AND ST_Intersects(a.geometry, b.geometry);
-- Show the intersections:
SELECT * FROM test.intersections ORDER BY id_a, id_b;
```

```sql
-- Only show the intersections that are lines:
SELECT * FROM test.intersections
WHERE ST_GeometryType(geometry) = 'LINESTRING'
ORDER BY id_a, id_b;
```

<!-- Pack the indices:

```sql
CREATE TABLE test_unnest_points_packed AS
SELECT
    struct_pack(cleabs:= cleabs, idx_polygon:= idx_polygon, idx_ring:= idx_ring, idx_point:= idx_point) AS idx,
    geometry
FROM test_unnest_points
ORDER BY cleabs, idx_polygon, idx_ring, idx_point;
``` -->

```sql
CREATE OR REPLACE TABLE intersections.edges_new AS
SELECT 
    cleabs, idx_polygon, idx_ring, idx_edge, 
    geometry,
    id,
    STRUCT_PACK(
        xmin := ST_XMin("geometry"),
        ymin := ST_YMin("geometry"),
        xmax := ST_XMax("geometry"),
        ymax := ST_YMax("geometry")
    ) AS bbox
FROM intersections.edges;
```

```sql
COPY (
    SELECT * FROM test.edges
    ORDER BY cleabs, idx_polygon, idx_ring, idx_edge)
TO 'data/bd_topo_edges.parquet'
(FORMAT parquet, COMPRESSION zstd, ROW_GROUP_SIZE 100_000);
```

Remove all tables:

```sql
DROP TABLE test.*;
```

## Group buildings

```sql
LOAD spatial;

-- Load the data
CREATE TABLE multipolygons AS SELECT cleabs, geometry FROM 'data/input/BD_TOPO/2026-03-15.parquet';
CREATE TABLE edges AS SELECT * FROM 'data/bd_topo-edges.parquet';
CREATE TABLE intersections AS SELECT * FROM 'data/bd_topo-intersections.parquet' WHERE ST_GeometryType(geometry) = 'LINESTRING';

-- Create a graph schema
CREATE SCHEMA IF NOT EXISTS graph;

-- Create a flat edge table (undirected, avoid duplicates if needed)
CREATE TABLE graph.edges AS
SELECT
    cleabs_a AS src,
    cleabs_b AS dst
FROM (
    SELECT ea.cleabs AS cleabs_a, eb.cleabs AS cleabs_b
    FROM intersections i
    JOIN edges AS ea ON i.id_a = ea.id
    JOIN edges AS eb ON i.id_b = eb.id
    GROUP BY (cleabs_a, cleabs_b)
    HAVING cleabs_a < cleabs_b
) t;

-- Compute the connected components with a recursive CTE
CREATE TABLE graph.node_to_root AS
WITH RECURSIVE reach(root, node) AS (
    SELECT src AS root, src AS node
    FROM graph.edges
    UNION
    SELECT r.root, e.dst
    FROM reach r
    JOIN graph.edges e ON e.src = r.node
)
SELECT node, MIN(root) AS root_id
FROM reach
GROUP BY node;

-- Add the isolated nodes (buildings with no intersection)
INSERT INTO graph.node_to_root (node, root_id)
SELECT cleabs, cleabs
FROM multipolygons
WHERE cleabs NOT IN (SELECT node FROM graph.node_to_root);

-- Group the nodes by their root to get the connected components
-- The group id is a new incremental index
CREATE TABLE graph.components AS
SELECT
    row_number() OVER (ORDER BY root_id) AS group_id,
    list(node ORDER BY node) AS cleabs_list
FROM graph.node_to_root
GROUP BY root_id
ORDER BY root_id;

-- Sum the number of buildings in each group
SELECT group_id, array_length(cleabs_list) AS num_buildings
FROM graph.components
ORDER BY num_buildings DESC;

-- Compute the extent of each group
CREATE TABLE graph.components_with_extent AS
SELECT
    c.group_id,
    c.cleabs_list,
    ST_Extent_Agg(m.geometry) AS extent
FROM (
    SELECT group_id, cleabs_list, UNNEST(cleabs_list) AS cleabs
    FROM graph.components
) c JOIN multipolygons m
ON m.cleabs = c.cleabs
GROUP BY (c.group_id, c.cleabs_list)
ORDER BY c.group_id;

-- Export the components with their extent as GeoParquet
COPY (
    WITH full_extent AS (
        SELECT ST_Extent(ST_Extent_Agg(extent)) AS extent
        FROM graph.components_with_extent
    )
    SELECT * FROM graph.components_with_extent
    ORDER BY ST_Hilbert(extent, (SELECT extent FROM full_extent))
)
TO 'data/bd_topo/groups.parquet'
(FORMAT parquet, COMPRESSION zstd, ROW_GROUP_SIZE 10_000);
```
