"""Canonical Task 1-7 GNSS/IMU fusion pipeline.

The public API deliberately stays small: :func:`run_pipeline` processes one
dataset with one attitude method, while :func:`run_methods` runs any subset of
methods and additionally creates a cross-method comparison. :mod:`catalog`
holds the single source of truth for task, subtask and figure names.
"""

from .catalog import TASKS, catalog_as_dict, describe_catalog
from .pipeline import (
    METHODS,
    PipelineConfig,
    parse_methods,
    parse_tasks,
    run_methods,
    run_pipeline,
)

__all__ = [
    "METHODS",
    "PipelineConfig",
    "TASKS",
    "catalog_as_dict",
    "describe_catalog",
    "parse_methods",
    "parse_tasks",
    "run_methods",
    "run_pipeline",
]
