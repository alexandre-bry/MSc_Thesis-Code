# AHN Classification by 3DBAG

## Installation

The code runs on the Dev Container provided in `.devcontainer`.
It is base on Ubuntu 24.04.
To install every required library, you can run:

```bash
sudo apt update
sudo apt install build-essential -y
sudo apt install libcgal-dev -y
sudo apt install libboost-all-dev libeigen3-dev libgmp-dev libmpfr-dev -y
```

To format the code, you can also install clangd:

```bash
sudo apt install clangd
```

## Running

### With just

To quickly and simply compile and run, we use [`just`](https://github.com/casey/just) recipes.
To install `just`, run `sudo apt install just`.
You can use the `just` recipes with:

```bash
just run <mode> <commands-and-arguments>
```

where `<mode>` is the cmake compilation mode.

For example

```bash
just run release split data/3DBAG/9-284-564/9-284-564-LoD22-3D.obj output/9-284-564-LoD22-3D.obj
```

### Without just

To build:

```bash
cmake -B build -S . -D CMAKE_BUILD_TYPE=Release
cmake --build build
```

To run:

```bash
./build/executable/AHN_Classification_by_3DBAG split data/3DBAG/9-284-564/9-284-564-LoD22-3D.obj output/output.obj
```
