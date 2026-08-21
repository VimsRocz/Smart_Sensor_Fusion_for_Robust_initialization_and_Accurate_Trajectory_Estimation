"""Independent task entry points backed by the canonical pipeline engine."""

from __future__ import annotations

TASK_MODULES = {
    number: f"fusion_pipeline.tasks.task_{number:02d}" for number in range(1, 8)
}

__all__ = ["TASK_MODULES"]
