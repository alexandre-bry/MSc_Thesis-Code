# Installation

Below are installation guidelines for Ubuntu for the tools required in this project.

## Prerequisites to compile from source

If you intend to compile GDAL and/or PDAL from source, please install these libraries before.

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    libgeos-dev \
    libproj-dev \
    libsqlite3-dev \
    libtiff5-dev \
    libcurl4-openssl-dev \
    libgeotiff-dev \
    libboost-all-dev \
    libzstd-dev \
    liblz4-dev
```

<!-- ## Install uv

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
``` -->

## Install Arrow/Parquet C++ support

Instructions come from the [official Arrow website](https://arrow.apache.org/install/).

```bash
sudo apt update
sudo apt install -y -V ca-certificates lsb-release wget
wget https://packages.apache.org/artifactory/arrow/$(lsb_release --id --short | tr 'A-Z' 'a-z')/apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt install -y -V ./apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
rm apache-arrow-apt-source-latest-$(lsb_release --codename --short).deb
sudo apt update
sudo apt install -y -V libarrow-dev libarrow-dataset-dev libparquet-dev 
```

## Install GDAL

There are many ways to install GDAL, but the two ways below are from my experience the simplest that allow to get the latest version of GDAL.

### GDAL with Pixi

[GDAL's website](https://gdal.org/en/stable/download.html#pixi) provides instructions to install GDAL with Pixi, either locally or globally.
To install it globally, use:

```bash
pixi global install gdal libgdal-arrow-parquet
```

### GDAL with conda

[GDAL's website](https://gdal.org/en/stable/download.html#conda) provides instructions to install GDAL with conda.

### GDAL from source

> [!IMPORTANT]
> See the [prerequisites](#prerequisites-to-compile-from-source) to install the necessary libraries first.

I do not like conda so below is a set of commands to compile GDAL v3.12.1 from source.
Replace `~/gdal-build` with the directory of your choice to compile somewhere else.

> [!IMPORTANT]
> To get the Parquet support by compiling from source, first [install Arrow and Parquet](#install-arrowparquet-c-support).

<!-- sudo apt install -y \
    libjpeg-dev \
    libpng-dev \
    libnetcdf-dev \
    libxml2-dev \
    libjson-c-dev \
    python3-dev python3-numpy -->

```bash
mkdir -p ~/gdal-build && cd ~/gdal-build
wget https://download.osgeo.org/gdal/3.12.1/gdal-3.12.1.tar.xz
tar xf gdal-3.12.1.tar.xz
cd gdal-3.12.1
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build . -j"$(nproc)"
sudo cmake --build . --target install
```

## Install PDAL

### PDAL with Pixi

[PDAL's website](https://pdal.io/en/stable/quickstart.html#experimental-install-via-pixi) provides instructions to install PDAL with Pixi, either locally or globally.
To install it globally, use:

```bash
pixi global install pdal
```

### PDAL with conda

[PDAL's website](https://pdal.io/en/stable/quickstart.html#install-the-pdal-package) provides instructions to install PDAL with conda.

### PDAL from source

> [!IMPORTANT]
> See the [prerequisites](#prerequisites-to-compile-from-source) to install the necessary libraries first.

I do not like conda so below is a set of commands to compile PDAL v2.9.3 from source.
Replace `~/pdal-build` with the directory of your choice to compile somewhere else.

> [!IMPORTANT]
> Compiling PDAL from source requires GDAL to be installed.
> See [instructions to install GDAL](#install-gdal) for instructions.

```bash
mkdir -p ~/pdal-build && cd ~/pdal-build
wget https://github.com/PDAL/PDAL/releases/download/2.9.3/PDAL-2.9.3-src.tar.bz2
tar -xvf PDAL-2.9.3-src.tar.bz2
cd PDAL-2.9.3-src
mkdir build && cd build
cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DWITH_TESTS=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

<!-- ## Install LAStools

```bash
mkdir ~/lastools && cd ~/lastools
wget https://downloads.rapidlasso.de/LAStools.tar.gz
tar xvzf LAStools.tar.gz && rm LAStools.tar.gz

sudo apt update
sudo apt install -y \
    libgeotiff-dev libjpeg62 libpng-dev libtiff-dev \
    libjpeg-dev libz-dev libproj-dev liblzma-dev libjbig-dev \
    libzstd-dev libwebp-dev libsqlite3-dev g++ cmake

export LD_LIBRARY_PATH=~/lastools/lib:$LD_LIBRARY_PATH
export PATH=~/lastools/bin:$PATH
source ~/.bashrc
``` -->

## Install GEOS

```bash
sudo apt-get -y install libgeos++-dev
```

## Install just

```bash
sudo apt install just
```

## Install CGAL

```bash
sudo apt install libcgal-dev
```

## Install pipx

Instructions come from the [official pipx website](https://pipx.pypa.io/stable/installation/).

```bash
sudo apt update
sudo apt install -y pipx
pipx ensurepath
```

## Install geoparquet-io

Instructions come from the [official pipx website](https://geoparquet.io/getting-started/installation/).

```bash
pipx install geoparquet-io
```

## Install C++ libraries

First create the include folder.

```bash
# From points_to_prints
mkdir cpp/include
```

### CLI11

```bash
curl -L https://github.com/CLIUtils/CLI11/releases/download/v2.6.1/CLI11.hpp --output cpp/include/CLI11.hpp
```

###
