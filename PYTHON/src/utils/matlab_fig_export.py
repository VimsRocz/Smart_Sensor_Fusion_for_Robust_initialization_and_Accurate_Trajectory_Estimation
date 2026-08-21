"""Compatibility entry point for the canonical editable figure exporter."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib.pyplot as plt


PYTHON_ROOT = Path(__file__).resolve().parents[2]
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from fusion_pipeline.figure_export import (  # noqa: E402,F401
    NATIVE_WRITTEN,
    WRITTEN,
    save_matlab_fig,
    validate_fig_openable,
    write_mat_companion,
)


def save_all_matplotlib_as_fig(out_dir: str, prefix: str = "") -> None:
    """Save every open Matplotlib figure through the canonical exporter."""
    output = Path(out_dir)
    output.mkdir(parents=True, exist_ok=True)
    for number in plt.get_fignums():
        figure = plt.figure(number)
        title = (
            figure._suptitle.get_text()
            if getattr(figure, "_suptitle", None)
            else f"figure_{number}"
        )
        stem = title.strip().replace(" ", "_").replace("/", "_") or f"figure_{number}"
        if prefix:
            stem = f"{prefix}_{stem}"
        save_matlab_fig(figure, str(output / stem))
