# From Points to Prints

## Introduction

This repository contains the code produced during my thesis for the MSc Geomatics at TU Delft, in collaboration with the IGN (Institut national de l’information géographique et forestière) from November 2025 to June 2026.

Reports and slides produced during the thesis are available in [this other repository](https://github.com/alexandre-bry/MSc_Thesis-Report).

## Installation

1. Install [pixi](https://pixi.prefix.dev/latest/) and add it to the path.
   For example on Linux:

   ```bash
   curl -fsSL https://pixi.sh/install.sh | sh
   echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
   ```

2. Install most of the dependencies of the project:

   ```bash
   pixi install
   ```

3. Then there are a few other dependencies to install manually:
   - You need to clone [LiDARHD_Traj_Estimation](https://github.com/whuwuteng/LiDARHD_Traj_Estimation) at the root of the project (not public yet)
   - You also need to install [`roofer`](https://github.com/3DBAG/roofer) (documentation [there](https://innovation.3dbag.nl/roofer/getting_started.html)) and the [binary of `cjseq`](https://github.com/cityjson/cjseq).

## Usage

To list the available commands, run `pixi task list`.
The commands are split between two CLIs depending on the language they were implemented in:

- `p2p-py` for the commands implemented in Python.
  You can get the list of available commands with `pixi run p2p-py --help`.
- `p2p-cpp` for the commands implemented in C++.
  You can get the list of available commands with `pixi run p2p-cpp --help`.

See the [documentation](docs/docs/usage.md) for information about the expected pipeline to produce the roofprints and the footprints.

## Tips

### Viewing the point clouds

To view the point clouds in 3D, you can use [CloudCompare](https://www.danielgm.net/cc/).
To view them in 2D together with the roofprints and footprints, you can use [QGIS](https://www.qgis.org/en/site/).
However, QGIS does not support custom fields in LAZ files, so the solution I found to open them is to first convert them to EPT.
For that, you can use [Entwine](https://entwine.io/), which is installed with pixi and that you can run like this:

```bash
pixi run entwine build -i <input_laz> -o <output_ept_folder> --deep --srs EPSG:2154
```

## Troubleshooting

### DuckDB spatial extension installation

If the automatic installation of the DuckDB spatial extension fails, you can download it manually and set the environment variable `POINTS2PRINTS_DUCKDB_INSTALL_PATH` to the path where the extension is downloaded.
The path to the extension should be visible in the error message when the automatic installation fails.

## TODO

- Make sure it is possible to install LiDARHD_Traj_Estimation and give some information about installing and compiling LiDARHD_Traj_Estimation
