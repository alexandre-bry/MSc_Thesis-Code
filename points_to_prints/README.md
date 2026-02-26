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

### C++ scripts

1. Find the commands of the C++ scripts:

    ```bash
    pixi run cpp-run --help
    ```

2. Find the arguments of a given command:

    ```bash
    pixi run cpp-run <command> --help
    ```

### Python scripts

1. Find the commands of the Python scripts:

    ```bash
    pixi run py-run --help
    ```

2. Find the arguments of a given command:

    ```bash
    pixi run py-run <command> --help
    ```
