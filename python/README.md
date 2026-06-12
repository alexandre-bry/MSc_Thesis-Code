# Python

## Dependencies

All the dependencies are indicated in the [pixi.toml](../pixi.toml) file.
Since many of the commands invoke other tools or C++ programs, this Python package actually depends on all the dependencies specified there, and not only the ones in the `python` category.

The package is subdivided in many individual packages which can depend one on the other.
Each subpackage creates its own command line interface, and all of them are then combined into a single command line interface.

## Usage

```bash
# The current directory is / (root of the project)

# calling python directly through the main module:
pixi run python -m python.main test_logging -vv

# or with the p2p-py command:
pixi run p2p-py test_logging -vv
```
