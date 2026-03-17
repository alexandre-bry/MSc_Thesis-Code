import asyncio
import logging
import os
import pty
import re
import subprocess
import sys
import threading
from enum import Enum
from pathlib import Path
from typing import Annotated, List

import typer
from download import download_lidar_hd_data
from las_manipulations import identity_convert, merge_files, split_file
from tqdm.contrib.logging import logging_redirect_tqdm


class Verbose(Enum):
    Error = logging.ERROR
    Warning = logging.WARNING
    Info = logging.INFO
    Debug = logging.DEBUG

    @classmethod
    def from_int(cls, verbose_int: int):
        match verbose_int:
            case 0:
                return cls.Error
            case 1:
                return cls.Warning
            case 2:
                return cls.Info
            case 3:
                return cls.Debug
            case _:
                raise RuntimeError("Verbose has only 4 possible values.")


class LevelColorFormatter(logging.Formatter):
    RESET = "\033[0m"
    COLORS = {
        logging.DEBUG: "\033[36m",  # cyan
        logging.INFO: "\033[32m",  # green
        logging.WARNING: "\033[33m",  # yellow
        logging.ERROR: "\033[31m",  # red
        logging.CRITICAL: "\033[35m",  # magenta
    }

    def __init__(self, use_color: bool = True, datefmt: str | None = None):
        super().__init__(datefmt=datefmt)
        self.use_color = use_color
        self._prefix_formatter = logging.Formatter(
            fmt="%(asctime)s - %(levelname)s - ",
            datefmt=datefmt,
        )

    def format(self, record: logging.LogRecord) -> str:
        record.message = record.getMessage()
        prefix = self._prefix_formatter.format(record)
        message = f"{prefix}{record.message}"

        if record.exc_info:
            message = f"{message}\n{self.formatException(record.exc_info)}"
        if record.stack_info:
            message = f"{message}\n{self.formatStack(record.stack_info)}"

        if not self.use_color:
            return message
        color = self.COLORS.get(record.levelno)
        if color is None:
            return message
        return f"{color}{prefix}{self.RESET}{record.message}"


def setup_logging(verbose: Verbose):
    formatter = LevelColorFormatter(
        use_color=sys.stdout.isatty(),
    )

    handler = logging.StreamHandler(stream=sys.stdout)
    handler.setFormatter(formatter)

    root_logger = logging.getLogger()
    root_logger.handlers.clear()
    root_logger.addHandler(handler)
    root_logger.setLevel(verbose.value)

    if verbose.value == logging.DEBUG:
        logging.getLogger("httpx").setLevel(logging.INFO)
    else:
        logging.getLogger("httpx").setLevel(logging.WARNING)

    return verbose != Verbose.Error


app = typer.Typer()


def run_command_with_tqdm_logging(command: list[str]) -> int:
    env = os.environ.copy()
    env.setdefault("PY_COLORS", "1")
    env.setdefault("CLICOLOR_FORCE", "1")
    env.setdefault("FORCE_COLOR", "1")
    env.setdefault("TERM", "xterm-256color")

    if os.name == "posix":
        master_fd, slave_fd = pty.openpty()
        try:
            process = subprocess.Popen(
                command,
                stdout=slave_fd,
                stderr=slave_fd,
                text=False,
                env=env,
            )
        finally:
            os.close(slave_fd)

        try:
            while True:
                try:
                    chunk = os.read(master_fd, 4096)
                except OSError:
                    break

                if not chunk:
                    break

                sys.stdout.buffer.write(chunk)
                sys.stdout.buffer.flush()
        finally:
            os.close(master_fd)

        return process.wait()

    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=False,
        bufsize=0,
        env=env,
    )

    def _forward_stream(stream, target_buffer):
        if stream is None:
            return

        try:
            while True:
                chunk = stream.read(4096)
                if not chunk:
                    break
                target_buffer.write(chunk)
                target_buffer.flush()
        finally:
            stream.close()

    stdout_thread = threading.Thread(
        target=_forward_stream, args=(process.stdout, sys.stdout.buffer), daemon=True
    )
    stderr_thread = threading.Thread(
        target=_forward_stream,
        args=(process.stderr, sys.stderr.buffer),
        daemon=True,
    )

    stdout_thread.start()
    stderr_thread.start()

    return_code = process.wait()
    stdout_thread.join()
    stderr_thread.join()
    return return_code


@app.command(
    "split_las",
    help="Split a .laz file into one file for each value of a specified dimension.",
)
def split_las(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the .laz file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file_template: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_file_template",
            help="Template for the output .laz files. Use # as a placeholder for the dimension value.",
        ),
    ],
    dimension: Annotated[
        str,
        typer.Option(
            "-d",
            "--dimension",
            help="The dimension to split by (e.g., 'Classification').",
        ),
    ],
    use_value_in_filename: Annotated[
        bool,
        typer.Option(
            "-n",
            "--name_with_value",
            help="Whether to include the dimension value in the output filename.",
        ),
    ] = False,
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        output_file_template.parent.mkdir(parents=True, exist_ok=True)

        split_file(
            input_file=input_file,
            output_file_template=output_file_template,
            dimension=dimension,
            use_value_in_filename=use_value_in_filename,
            overwrite=overwrite,
        )


@app.command("merge_las", help="Merge multiple .laz files into a single .laz file.")
def merge_las(
    input_files: Annotated[
        List[Path],
        typer.Option(
            "-i",
            "--input",
            help="Paths to the .laz files. Can be used multiple times.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_file",
            help="Path to the output .laz file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        output_file.parent.mkdir(parents=True, exist_ok=True)

        merge_files(
            input_files=input_files, output_file=output_file, overwrite=overwrite
        )
        logging.info(f"Successfully merged files into {output_file}")


@app.command(
    "download_lidar_hd", help="Download LiDAR HD data for a specified bounding box."
)
def download_lidar_hd(
    bbox: Annotated[
        str,
        typer.Option(
            "-b",
            "--bbox",
            help="Bounding box to download data for, in the format 'xmin,ymin,xmax,ymax'.",
        ),
    ],
    output_path_template: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_path",
            help="Path to save the downloaded files. The path can contain the values {xmin}, {ymin}, {xmax}, {ymax}, {file_name} which will be replaced with the corresponding values. The values also have their kilometre equivalents {xmin_km}, {ymin_km}, {xmax_km}, {ymax_km}.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        bbox_values = bbox.split(",")
        if len(bbox_values) != 4:
            raise ValueError(
                "Bounding box must be in the format 'xmin,ymin,xmax,ymax'."
            )
        xmin, ymin, xmax, ymax = map(int, bbox_values)

        asyncio.run(
            download_lidar_hd_data(
                xmin=xmin,
                ymin=ymin,
                xmax=xmax,
                ymax=ymax,
                output_path_template=output_path_template,
                overwrite=overwrite,
            )
        )


@app.command(
    "run_pipeline_before",
    help="Run the initial steps of the pipeline before the computation of the trajectory.",
)
def run_pipeline_before(
    tile_dir: Annotated[
        Path,
        typer.Option(
            "-t",
            "--tile_dir",
            help="Directory containing the downloaded tile .laz files.",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    bd_topo_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo_file",
            help="Path to the BD_TOPO parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        source_laz_file = tile_dir / "lidarhd.copc.laz"

        # Split the source .laz file into multiple files based on the "PointSourceId" attribute
        split_las(
            input_file=source_laz_file,
            output_file_template=tile_dir / "axis_#.laz",
            dimension="PointSourceId",
            use_value_in_filename=True,
            overwrite=overwrite,
            verbose_int=verbose_int,
        )


@app.command(
    "run_pipeline",
    help="Run the final steps of the pipeline, after the computation of the trajectory.",
)
def run_pipeline(
    tile_dir: Annotated[
        Path,
        typer.Option(
            "-t",
            "--tile_dir",
            help="Directory containing the downloaded tile .laz files.",
            exists=True,
            file_okay=False,
            dir_okay=True,
            readable=True,
        ),
    ],
    bd_topo_file: Annotated[
        Path,
        typer.Option(
            "-b",
            "--bd_topo_file",
            help="Path to the BD_TOPO parquet file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output files.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        # Find all the axis_*.laz files in the tile_dir
        laz_files = sorted(
            [
                p
                for p in tile_dir.glob("axis_*.laz")
                if re.fullmatch(r"axis_\d+\.laz", p.name)
            ]
        )
        if len(laz_files) == 0:
            logging.error(f"No axis_*.laz files found in directory: {tile_dir}")
            return

        # Build the C++ tools using pixi
        command = ["pixi", "run", "--quiet", "cpp-build"]
        logging.info(f"Building the C++ tools: {' '.join(command)}")
        return_code = run_command_with_tqdm_logging(command)
        if return_code != 0:
            logging.error("C++ build failed.")
        else:
            logging.info("C++ tools built successfully.")

        # Process each .laz file with the C++ pipeline
        total_files = len(laz_files)
        distances_files = []
        edges_files = []
        for index, laz_file in enumerate(laz_files, start=1):
            logging.info(
                f"\n\nO----- [{index}/{total_files}] Processing tile: {laz_file.name} -----O\n"
            )
            distance_file = tile_dir / f"{laz_file.stem}-distances.laz"
            edge_file = tile_dir / f"{laz_file.stem}-edges.laz"
            distances_files.append(distance_file)
            edges_files.append(edge_file)

            if distance_file.exists() and edge_file.exists() and not overwrite:
                logging.info(
                    f"Output files for {laz_file.name} already exist. Skipping processing."
                )
                continue

            command = [
                "pixi",
                "run",
                "--quiet",
                "cpp-run-only",
                "distances_in_order",
                "-i",
                str(laz_file),
                "-t",
                str(tile_dir / f"{laz_file.stem}-trajectory.txt"),
                "-d",
                str(distance_file),
                "-e",
                str(edge_file),
            ]

            if overwrite:
                command.append("--overwrite")

            logging.debug(
                f"Processing {laz_file.name} with command: {' '.join(command)}"
            )
            return_code = run_command_with_tqdm_logging(command)
            logging.debug(f"Return code for {laz_file.name}: {return_code}")
            if return_code != 0:
                logging.error(f"Failed to process {laz_file.name}.")
            else:
                logging.info(f"Successfully processed {laz_file.name}.")

    # After processing all files, merge the distances and edges files into single files
    logging.info("\n\nO----- Merging files -----O\n")

    merged_distances_file = tile_dir / "merged_distances.laz"
    merged_edges_file = tile_dir / "merged_edges.laz"

    try:
        merge_las(
            input_files=distances_files,
            output_file=merged_distances_file,
            verbose_int=verbose_int,
            overwrite=overwrite,
        )
    except FileExistsError as e:
        logging.info(f"Distances file merge skipped: {e}")
    except Exception as e:
        logging.error(
            f"An unexpected error occurred while merging distances files: {e}"
        )

    try:
        merge_las(
            input_files=edges_files,
            output_file=merged_edges_file,
            verbose_int=verbose_int,
            overwrite=overwrite,
        )
    except FileExistsError as e:
        logging.info(f"Edges file merge skipped: {e}")
    except Exception as e:
        logging.error(f"An unexpected error occurred while merging edges files: {e}")

    # Then, compute the roofprints using the merged edges
    logging.info("\n\nO----- Computing roofprints -----O\n")
    roofprints_file = tile_dir / "roofprints.parquet"
    if roofprints_file.exists() and not overwrite:
        logging.info(
            f"Roofprints file already exists: {roofprints_file}. Skipping roofprint computation."
        )
    else:
        command = [
            "pixi",
            "run",
            "--quiet",
            "cpp-run-only",
            "compute_roofprints",
            "-i",
            str(merged_edges_file),
            "-b",
            str(bd_topo_file),
            "-o",
            roofprints_file,
        ]

        if overwrite:
            command.append("--overwrite")

        logging.debug(
            f"Running roofprint computation with command: {' '.join(command)}"
        )
        return_code = run_command_with_tqdm_logging(command)
        if return_code != 0:
            logging.error("Failed to compute roofprints.")
        else:
            logging.info("Successfully computed roofprints.")

    logging.info("\n\nPipeline completed.")


@app.command(
    "identity",
    help="A simple command to test the pipeline by copying an input point cloud to an output file without any modifications.",
)
def identity(
    input_file: Annotated[
        Path,
        typer.Option(
            "-i",
            "--input",
            help="Path to the input point cloud file.",
            exists=True,
            file_okay=True,
            dir_okay=False,
            readable=True,
        ),
    ],
    output_file: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_file",
            help="Path to the output point cloud file.",
        ),
    ],
    overwrite: Annotated[
        bool,
        typer.Option(
            "--overwrite",
            help="Whether to overwrite the output file if it already exists.",
        ),
    ] = False,
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        identity_convert(
            input_file=input_file, output_file=output_file, overwrite=overwrite
        )


@app.command(
    "test_logging",
    help="A simple command to test the logging setup with different verbosity levels.",
)
def test(verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        logging.debug("This is a debug message.")
        logging.info("This is an info message.")
        logging.warning("This is a warning message.")
        logging.error("This is an error message.")


if __name__ == "__main__":
    app()
