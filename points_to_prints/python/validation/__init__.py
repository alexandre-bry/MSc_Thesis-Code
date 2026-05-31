"""Validation dataset preparation and metrics utilities.

This package provides helpers to persist validation datasets and score
against either the raw individual validation data or the persisted
aggregated validation data.
"""

from .metrics import compare_polygon_datasets_call
from .processing import build_validation_datasets

__all__ = [
    "build_validation_datasets",
    "compare_polygon_datasets_call",
]
