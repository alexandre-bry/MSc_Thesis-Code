"""
Generic utilities
"""

from .custom_logging import LoggingContext, Verbose, run_command_with_tqdm_logging
from .duckdb_helpers import DuckDBConnectionManager, DuckDBConnector
from .geom import Box2154, Point2154
from .input_output import InputOutput, OutputAction, OutputActionEnum, OutputBehaviour
from .result import Err, Ok, Result

__all__ = [
    "LoggingContext",
    "Verbose",
    "run_command_with_tqdm_logging",
    "DuckDBConnectionManager",
    "DuckDBConnector",
    "Box2154",
    "Point2154",
    "InputOutput",
    "OutputAction",
    "OutputActionEnum",
    "OutputBehaviour",
    "Err",
    "Ok",
    "Result",
]
