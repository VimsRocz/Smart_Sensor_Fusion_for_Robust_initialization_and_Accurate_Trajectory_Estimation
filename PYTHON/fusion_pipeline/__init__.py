"""Canonical Task 1-7 GNSS/IMU fusion pipeline.

The public API deliberately stays small: :func:`run_pipeline` processes one
dataset with one attitude method, while :func:`run_methods` additionally
creates a cross-method comparison.
"""

from .pipeline import METHODS, PipelineConfig, run_methods, run_pipeline

__all__ = ["METHODS", "PipelineConfig", "run_methods", "run_pipeline"]
