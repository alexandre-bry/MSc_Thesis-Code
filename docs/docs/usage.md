---
icon: lucide/rocket
---

# Usage

This file documents the usage of the CLI tools to run the pipeline to produce roofprints and footprints.

To list the available commands, run `pixi task list`.
The commands are split between two CLIs depending on the language they were implemented in:

- `p2p-py` for the commands implemented in Python.
  You can get the list of available commands with `pixi run p2p-py --help`.
- `p2p-cpp` for the commands implemented in C++.
  You can get the list of available commands with `pixi run p2p-cpp --help`.

## Run the pipeline

We use the following structure to organize the data:

```bash
data
├── input           # Raw input data
├── formatted       # Data formatted for the pipeline
└── tiles           # Data specific to each tile
```

Each tile has its own folder in `data/tiles`, which will have the following structure:

```bash
<tile_folder>
├── bd_topo
├── footprints
├── lidar_hd
├── roof
└── roofprints
```

The sub-commands in the`pipeline` command will expect and follow this structure, but most of the individual commands in the project are still flexible and will ask for the input and output paths, so you can adapt them to your needs.

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
pixi run p2p-py pipeline download_lidar_hd \
    --bbox "<minx>,<miny>,<maxx>,<maxy>" \
    --tiles-dir "output_tiles_directory>"
# Example:
pixi run p2p-py pipeline download_lidar_hd \
    --bbox "676000,6851000,677000,6853000" \
    --tiles-dir "data/tiles" \
    -vv
```

As you can see, we use the `data/tiles` folder to store the data specific to each tile, and each tile has its own folder named with the coordinates of its lower-left corner in kilometers (e.g. the tile `668000,6859000,669000,6860000` will be stored in the folder `data/tiles/668_6859`).

### 2. Download and prepare the BD TOPO data

You first need to download the BD TOPO data from [cartes.gouv.fr](https://cartes.gouv.fr/rechercher-une-donnee/dataset/IGNF_BD-TOPO) for your area of interest.
Once extracted (for example with `7z x data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_D077_2026-03-15.7z -odata/input/bd_topo/`), you can convert the data to the expected Parquet format using the following command:

```bash
# General command:
pixi run p2p-py pipeline prepare_bd_topo \
    -s <input_file> \
    -o <output_folder>
# Example:
pixi run p2p-py pipeline prepare_bd_topo \
    -s data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_R11_2026-03-15/BDTOPO/1_DONNEES_LIVRAISON_2026-03-00147/BDT_3-5_GPKG_LAMB93_R11_ED2026-03-15/BDT_3-5_GPKG_LAMB93_R11-ED2026-03-15.gpkg \
    -o data/formatted/bd_topo/R11-2026_03_15 \
    -vv
```

As you can see we use the `data/input/bd_topo` folder for the raw BD TOPO data, and the `data/formatted/bd_topo` folder for the formatted BD TOPO data.

### 3. Run the pipeline

To run the full pipeline for a given tile, we have a Python script that bundles all the necessary steps.
The final steps of the pipeline require [`roofer`](https://github.com/3DBAG/roofer) and [`cjseq`](https://github.com/cityjson/cjseq) to be installed.

```bash
# General command:
pixi run p2p-py pipeline points_to_prints \
    -b <input_bd_topo_parquet> \
    -t <input_tile_folder>
# Example:
pixi run p2p-py pipeline points_to_prints \
    -b data/formatted/bd_topo/R11-2026_03_15/ \
    -t data/tiles/676_6851/ \
    -vv
```

In case you do not have a lot of RAM available, adding `--num-workers 1` can reduce the amount of RAM used by the pipeline, at the cost of slower performance.
This command will produce many intermediate and output files in the tile folder.

## Validation with the ground-truth dataset

The ground-truth dataset currently contains 170 polygons of roofprints.
It is available in the `data/validation` folder.

There are a few steps to prepare it and compute metrics to assess the quality of the roofprints produced by the pipeline.

### 1. Prepare the ground-truth dataset

Since the separation of adjacent buildings is sometimes difficult to determine, we create an aggregated version of the ground-truth dataset, where adjacent buildings are merged into a single polygon.

```bash
# General command:
pixi run p2p-py validation prepare_validation_dataset \
    <input_ground_truth_parquet> \
    <output_individual_parquet> \
    <output_aggregated_parquet> \
    -k <column_to_keep_1> \
    -k <column_to_keep_2> \
    -k ...
# Example
pixi run p2p-py validation prepare_validation_dataset \
    data/validation/validation-roofprints.parquet \
    data/validation/validation-roofprints-individual.parquet \
    data/validation/validation-roofprints-aggregated.parquet \
    -k custom_category \
    -vv
```

### 2. Compute metrics

Then, a command is available to compute in one go all the metrics for the roofprints produced by the pipeline and for the initial BD TOPO outlines.

```bash
# General command:
pixi run p2p-py pipeline metrics \
    -i <input_individual_parquet> \
    -a <input_aggregated_parquet> \
    -b <input_bd_topo_parquet> \
    -t <input_tile_folder_1> \
    -t <input_tile_folder_2> \
    -t ... \
    -o <output_folder> \
    --output-format <output_format> \
    --id-column <id_column> \
    --spacing-m <spacing_in_meters> \
    -k <column_to_keep_1> \
    -k <column_to_keep_2> \
    -k ...

# Example
pixi run p2p-py pipeline metrics \
    -i data/validation/validation-roofprints-individual.parquet \
    -a data/validation/validation-roofprints-aggregated.parquet \
    -b data/formatted/bd_topo/R11-2026_03_15/bd_topo.parquet \
    -t data/tiles/676_6851/ \
    -t data/tiles/676_6852/ \
    -t data/tiles/656_6861/ \
    -o data/validation/results/ \
    --output-format parquet \
    --id-column cleabs \
    --spacing-m 0.01 \
    -k custom_category \
    -vv
```

## All the commands together on the tiles of the validation dataset

```bash
pixi run p2p-py pipeline download_lidar_hd \
    --bbox "676000,6851000,677000,6853000" \
    --tiles-dir "data/tiles" \
    -vv
pixi run p2p-py pipeline download_lidar_hd \
    --bbox "656000,6861000,657000,6862000" \
    --tiles-dir "data/tiles" \
    -vv
curl https://data.geopf.fr/telechargement/download/BDTOPO/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_R11_2026-03-15/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_R11_2026-03-15.7z --output data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_R11_2026-03-15.7z
7z x data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_R11_2026-03-15.7z -odata/input/bd_topo/
pixi run p2p-py pipeline prepare_bd_topo \
    -s data/input/bd_topo/BDTOPO_3-5_TOUSTHEMES_GPKG_LAMB93_R11_2026-03-15/BDTOPO/1_DONNEES_LIVRAISON_2026-03-00147/BDT_3-5_GPKG_LAMB93_R11_ED2026-03-15/BDT_3-5_GPKG_LAMB93_R11-ED2026-03-15.gpkg \
    -o data/formatted/bd_topo/R11-2026_03_15 \
    -vv
pixi run p2p-py pipeline points_to_prints \
    -b data/formatted/bd_topo/R11-2026_03_15/ \
    -t data/tiles/676_6851/ \
    -vv
pixi run p2p-py pipeline points_to_prints \
    -b data/formatted/bd_topo/R11-2026_03_15/ \
    -t data/tiles/676_6852/ \
    -vv
pixi run p2p-py pipeline points_to_prints \
    -b data/formatted/bd_topo/R11-2026_03_15/ \
    -t data/tiles/656_6861/ \
    -vv
pixi run p2p-py validation prepare_validation_dataset \
    data/validation/validation-roofprints.parquet \
    data/validation/validation-roofprints-individual.parquet \
    data/validation/validation-roofprints-aggregated.parquet \
    -k custom_category \
    -vv
    -i data/validation/validation-roofprints-individual.parquet \
    -a data/validation/validation-roofprints-aggregated.parquet \
    -b data/formatted/bd_topo/R11-2026_03_15/bd_topo.parquet \
    -t data/tiles/676_6851/ \
    -t data/tiles/676_6852/ \
    -t data/tiles/656_6861/ \
    -o data/validation/results/ \
    --output-format parquet \
    --id-column cleabs \
    --spacing-m 0.01 \
    -k custom_category \
    -vv
```
