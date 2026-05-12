# Documentation

This file contains documentation about the functions and tools to use to run the pipeline.

## Prerequisites

To run the pipeline, you first need to install the dependencies.
The easiest way is using [`pixi`](https://pixi.prefix.dev/latest/):

```bash
pixi install
```

## Steps to run the pipeline

We use the following structure to organize the data:

```bash
data
├── formatted       # Data formatted for the pipeline
├── input           # Raw input data
└── tiles           # Data specific to each tile
```

Most of the commands in the pipeline are still flexible and will ask for the input and output paths, so you can adapt them to your needs.

Regarding the commands themselves, everything below will start with `pixi run <command>` because we assume that everything was installed with `pixi`.
Then, there are three types of commands:

- External tools, which have their own behaviours and documentations
- Python CLI commands, which are Python scripts that accept arguments.
  Most of them should have the following arguments:
    - `--help`: to display the help message with the description of the arguments
    - `--verbose` or `-v`: can be used up to `-vvv` to increase the verbosity of the logs
    - `--overwrite`: to overwrite the output files if they already exist
    - `--skip-existing`: to skip the processing if the output files already exist
- C++ CLI commands, which are C++ programs that accept arguments.
  Most of them should have the `--help` and `--overwrite` arguments, but they don't have the `--verbose` and `--skip-existing` arguments.

Do not hesitate to use the `--help` argument to get more details about the arguments of each command.

### 1. Download the LiDAR HD data

If you know the bounding box on which you want to run the pipeline, you can download the LiDAR data using this command:

```bash
# General command:
pixi run py-run lidarhd -- download_lidar_hd \
    --bbox "<minx>,<miny>,<maxx>,<maxy>" \
    -o "<output_template>"
# Example:
pixi run py-run lidarhd -- download_lidar_hd \
    --bbox "668000,6859000,670000,6861000" \
    -o "data/tiles/{xmin_km}_{ymin_km}/lidarhd.copc.laz" \
    -vv
```

As you can see, we use the `data/tiles` folder to store the data specific to each tile, and each tile has its own folder named with the coordinates of its lower-left corner in kilometers (e.g. the tile `668000,6859000,669000,6860000` will be stored in the folder `data/tiles/668_6859`).

### 2. Download the BD TOPO data

You first need to download the BD TOPO data from [cartes.gouv.fr](https://cartes.gouv.fr/rechercher-une-donnee/dataset/IGNF_BD-TOPO) for your area of interest.
Once extracted (for example with `7z x data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_D077_2026-03-15.7z -odata/input/bd_topo/`), you can convert the data to the expected Parquet format using the following command:

```bash
# General command:
pixi run py-run lidarhd -- bd_topo_convert \
    -i <input> \
    -o <output_parquet>
# Example:
pixi run py-run lidarhd -- bd_topo_convert \
    -i data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_D077_2026-03-15/BDTOPO/1_DONNEES_LIVRAISON_2026-03-00146/BDT_3-5_GPKG_LAMB93_D077_ED2026-03-15/BDT_3-5_GPKG_LAMB93_D077-ED2026-03-15.gpkg \
    -o data/formatted/bd_topo/D077-2026_03_15/bd_topo.parquet \
    -vv
```

As you can see we use the `data/input/bd_topo` folder for the raw BD TOPO data, and the `data/formatted/bd_topo` folder for the formatted BD TOPO data.

### 3. Compute intersections between building outlines in BD TOPO

To identify groups of touching building outlines in the BD TOPO data, you can use the following command:

```bash
# General command:
pixi run py-run lidarhd -- bd_topo_intersections \
    -b <input_bd_topo_parquet> \
    -e <output_edges_parquet> \
    -i <output_intersections_parquet> \
    -g <output_building_groups_parquet>
# Example:
pixi run py-run lidarhd -- bd_topo_intersections \
    -b data/formatted/bd_topo/D077-2026_03_15/bd_topo.parquet \
    -e data/formatted/bd_topo/D077-2026_03_15/edges.parquet \
    -i data/formatted/bd_topo/D077-2026_03_15/intersections.parquet \
    -g data/formatted/bd_topo/D077-2026_03_15/building_groups.parquet \
    -vv
```

### 4. Extract the roof edge points for a given tile

Then, to extract the roof edge points for a given tile, we have a Python script that bundles all the necessary steps:

```bash
# General command:
pixi run py-run lidarhd -- run_pipeline \
    -b <input_bd_topo_parquet> \
    -t <input_tile_folder>
# Example:
pixi run py-run lidarhd -- run_pipeline \
    -b data/formatted/bd_topo/D077-2026_03_15/ \
    -t data/tiles/668_6859/ \
    -vv
```

This command will produce many intermediate and output files in the tile folder.
The most important one is the one called `merged_edges.laz`, which contains the roof edge points that will be used in the next steps of the pipeline.

### 5. Compute the roofprints

To compute the roofprints, you can use the following command:

```bash
# General command:
pixi run cpp run release -- compute_roofprints \
    -l <input_roof_edge_points_laz> \
    -e <input_bd_topo_edges_parquet> \
    -i <input_bd_topo_intersections_parquet> \
    -o <output_roofprints_parquet> \
    --iterations <number_of_iterations>
# Example:
pixi run cpp run release -- compute_roofprints \
    -l data/tiles/668_6859/merged_edges.laz \
    -e data/tiles/668_6859/bd_topo-edges.parquet \
    -i data/tiles/668_6859/bd_topo-intersections.parquet \
    -o data/tiles/668_6859/roofprints-iter_1.parquet \
    --iterations 1
```

The number of iterations can be increased but the results are not always better, so you can experiment with it.

### 6. Compute the 3D roof model

Finally, to compute the 3D roof model, you need to have [`roofer`](https://github.com/3DBAG/roofer) and [`cjseq`](https://github.com/cityjson/cjseq) installed.
Then, you can use the following command:

```bash
# General command:
pixi run py-run roof -- roofprints_to_lod22 \
    -l <input_point_cloud_laz> \
    -r <input_roofprints_gpkg> \
    -o <output_lod22_cityjson>
# Example:
pixi run py-run roof -- roofprints_to_lod22 \
    data/tiles/668_6859/lidarhd.copc.laz \
    data/tiles/668_6859/roofprints-iter_1.parquet \
    data/tiles/668_6859/lod_2_2.city.json \
    -vv
```
