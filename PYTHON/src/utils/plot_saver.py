from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict


def ensure_dir(p: str) -> None:
    os.makedirs(p, exist_ok=True)


def save_png_and_mat(fig, basepath: str, arrays: Dict[str, Any] | None = None) -> None:
    """
    Save a Matplotlib or Plotly figure to PNG and (optionally) a .mat with the underlying data.

    - fig: Matplotlib Figure or Plotly Figure (both expose save APIs)
    - basepath: path without extension, e.g., ".../IMU_X002_GNSS_X002_DAVENPORT_task3_errors"
    - arrays: dict of numpy arrays to store inside the .mat (keys become variable names)
    """
    stem = Path(basepath)
    png = stem.with_suffix(".png")
    mat = stem.with_suffix(".mat")
    is_matplotlib = hasattr(fig, "savefig")
    try:
        fig.savefig(png, dpi=150, bbox_inches="tight")
    except Exception:
        try:
            fig.write_image(png, scale=2)
        except Exception as e:
            print(f"[WARN] Could not save PNG {png}: {e}")
            return
    print(f"[SAVE] {png}")

    if arrays:
        try:
            from scipy.io import savemat
            savemat(mat, arrays, do_compression=True)
            print(f"[MAT ] {mat} keys={list(arrays.keys())}")
        except Exception as e:
            print(f"[WARN] Could not save MAT for {basepath}: {e}")

    if is_matplotlib:
        from utils.matlab_fig_export import write_mat_companion

        write_mat_companion(fig, mat)
