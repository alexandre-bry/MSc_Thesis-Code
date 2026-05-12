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
    pixi install
    ```

## Usage

To list the available commands, run `pixi task list`.
See the [documentation](./Documentation.md) for information about the expected pipeline to produce the roofprints and the footprints.
