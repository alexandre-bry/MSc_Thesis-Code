# C++

## Dependencies

Most of the dependencies are indicated in the [pixi.toml](../pixi.toml) file, under the `[feature.cpp.dependencies]` section.

Some of the dependencies are also header-only libraries, and are instead available in the [include/](./include/) directory.
These libraries are:

- [CLI11](https://github.com/CLIUtils/CLI11)
- [indicators](https://github.com/p-ranav/indicators)
- [json](https://github.com/nlohmann/json)

## Usage

This repo uses CMake as the build system, and executables are built in the [build/](./build/) directory.

For the sake of simplicity, pixi tasks are used to make, build and run the executables.
They are defined in the [pixi.toml](../pixi.toml) at the root of the repository.
This is useful to run the executables in a `pixi` shell, which has all the dependencies available.

For example, to run the command `hello_world` with the argument `name` equal to `Alexandre`, you can run either of:

```bash
# These commands are run from the root

# if already compiled:
./cpp/build/Release/executable/Points2Prints hello_world --name Alexandre

# with pixi if already compiled:
pixi run p2p-cpp-run-only hello_world --name Alexandre

# or with pixi by compiling first if necessary:
pixi run p2p-cpp hello_world --name Alexandre
```
