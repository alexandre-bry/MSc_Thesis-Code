# C++

## Dependencies

Most of the dependencies are indicated in the [pixi.toml](../pixi.toml) file, under the `[feature.cpp.dependencies]` section.

Some of the dependencies are also header-only libraries, and are instead available in the [include/](./include/) directory.
These libraries are:

- [CLI11](https://github.com/CLIUtils/CLI11)
- [indicators](https://github.com/p-ranav/indicators)
- [json](https://github.com/nlohmann/json)

## Compilation and execution

This repo uses CMake as the build system, and executables are built in the [build/](./build/) directory.

For the sake of simplicity, a [`justfile`](https://github.com/casey/just) is used to make, build and run the executables.
It is available [at the root of the repository](../justfile).

Then, if you use `pixi` to install the dependencies, I also created commands in `pixi` that will call `just` to build and run the executables.
This is useful to run the executables in a `pixi` shell, which has all the dependencies available.

For example, the `justfile` defines a `run` command which makes, builds and runs the `main` executable and gives access to the CLI.
It can be run with `just run <mode> <args>`.
If you use `pixi`, you can access all the commands from `just` using `pixi run cpp <just-command> <mode> -- <args>`.
For example, to run the command `hello_world` with the argument `name` equal to `Alexandre`, you can run either of:

```bash
# The current directory is cpp/

# If already compiled:
./build/Release/executable/Points2Prints hello_world --name Alexandre

# or with just:
just run release hello_world --name Alexandre

# or with pixi:
pixi run cpp run release -- hello_world --name Alexandre

# or with pixi and the p2p-cpp command
pixi run p2p-cpp hello_world --name Alexandre
```
