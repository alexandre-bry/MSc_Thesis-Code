"""Validation dataset preparation and metrics utilities.

This package provides helpers to persist validation datasets first and then
score against the persisted aggregated validation data.
"""

from .processing import build_validation_datasets, build_aggregated_validation_dataset
from .metrics import compare_polygon_datasets_call

__all__ = [
    "build_validation_datasets",
    "build_aggregated_validation_dataset",
    "compare_polygon_datasets_call",
]
