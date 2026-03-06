import asyncio
import json
import logging
import os
import pty
import sys
import threading
from enum import Enum
from pathlib import Path
import re
import subprocess
from typing import Annotated, List

import pdal
import typer
from geopandas import GeoDataFrame
from tqdm import tqdm
from tqdm.contrib.logging import logging_redirect_tqdm

from las_manipulations import merge_files, split_file
from download import download_lidar_hd_data


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


@app.command("merge_las")
def merge_las(
    input_file: Annotated[
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
    verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0,
):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        output_file.parent.mkdir(parents=True, exist_ok=True)

        merge_files(input_files=input_file, output_file=output_file)
        logging.info(f"Successfully merged files into {output_file}")


@app.command("split_las")
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


@app.command("download_lidar_hd")
def download_lidar_hd(
    bbox: Annotated[
        str,
        typer.Option(
            "-b",
            "--bbox",
            help="Bounding box to download data for, in the format 'xmin,ymin,xmax,ymax'.",
        ),
    ],
    output_dir: Annotated[
        Path,
        typer.Option(
            "-o",
            "--output_dir",
            help="Directory to save the downloaded files.",
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
        output_dir.mkdir(parents=True, exist_ok=True)

        bbox_values = bbox.split(",")
        if len(bbox_values) != 4:
            raise ValueError(
                "Bounding box must be in the format 'xmin,ymin,xmax,ymax'."
            )
        xmin, ymin, xmax, ymax = map(float, bbox_values)

        asyncio.run(
            download_lidar_hd_data(
                xmin=xmin,
                ymin=ymin,
                xmax=xmax,
                ymax=ymax,
                output_dir=output_dir,
                overwrite=overwrite,
            )
        )


@app.command("run_pipeline")
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
            logging.error(f"No .laz files found in directory: {tile_dir}")
            return

        command = ["pixi", "run", "cpp-build"]
        logging.info(f"Building pipeline with command: {' '.join(command)}")
        return_code = run_command_with_tqdm_logging(command)
        if return_code != 0:
            logging.error("Pipeline build failed.")
        else:
            logging.info("Pipeline build completed successfully.")

        total_files = len(laz_files)
        for index, laz_file in enumerate(laz_files, start=1):
            logging.info(
                f"\n\nO----- [{index}/{total_files}] Processing tile: {laz_file.name} -----O\n"
            )
            command = [
                "pixi",
                "run",
                "cpp-run-only",
                "distances_in_order",
                "-i",
                str(laz_file),
                "-t",
                str(tile_dir / f"{laz_file.stem}-trajectory.txt"),
                "-d",
                str(tile_dir / f"{laz_file.stem}-distances.laz"),
                "-e",
                str(tile_dir / f"{laz_file.stem}-edges.laz"),
            ]

            if overwrite:
                command.append("--overwrite")

            logging.info(f"Running pipeline with command: {' '.join(command)}")
            return_code = run_command_with_tqdm_logging(command)
            if return_code != 0:
                logging.error("Pipeline failed.")
            else:
                logging.info("Pipeline completed successfully.")


@app.command("test")
def test(verbose_int: Annotated[int, typer.Option("--verbose", "-v", count=True)] = 0):
    setup_logging(verbose=Verbose.from_int(verbose_int))

    with logging_redirect_tqdm():
        logging.debug("This is a debug message.")
        logging.info("This is an info message.")
        logging.warning("This is a warning message.")
        logging.error("This is an error message.")


if __name__ == "__main__":
    app()
