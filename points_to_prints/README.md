# Roofprint Extractor

## Installation

Commands are given for Ubuntu.

1. Install pixi and add it to the path.

    ```bash
    curl -fsSL https://pixi.sh/install.sh | sh
    echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
    ```

2. Install the dependencies.

    ```bash
    cd cpp
    pixi install
    ```

## Usage

1. Find the available pre-made scripts for convenience:

    ```bash
    pixi run just --list
    ```

2. Find the commands of the C++ script:

    ```bash
    pixi run just run release --help
    ```

3. Find the arguments of a given command:

    ```bash
    pixi run just run release <command> --help
    ```
