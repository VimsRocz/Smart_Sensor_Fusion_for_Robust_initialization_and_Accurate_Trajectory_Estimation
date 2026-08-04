from __future__ import annotations

from collections import defaultdict
from pathlib import Path
from typing import Dict, List
from utils import save_png_and_mat  # legacy fallback
from utils.matlab_fig_export import save_matlab_fig

_saved: Dict[str, List[str]] = defaultdict(list)


def save_plot(fig, results_dir: str | Path, run_id: str, task_label: str, plot_label: str, ext: str = "png", **kwargs) -> Path:
    """Save *fig* to ``results_dir`` with a standardised filename.

    Parameters
    ----------
    fig : matplotlib.figure.Figure
        Figure object to save.
    results_dir : str or Path
        Base directory where plots are stored.
    run_id : str
        Identifier for the current run (e.g. ``IMU_X002_GNSS_X002_TRIAD``).
    task_label : str
        Task label such as ``"task1"`` or ``"task6"``.
    plot_label : str
        Short descriptor for the specific plot.
    ext : str, optional
        File extension, defaults to ``"png"``.
    kwargs : dict
        Extra arguments forwarded to :meth:`matplotlib.figure.Figure.savefig`.
    """
    results_dir = Path(results_dir)
    results_dir.mkdir(parents=True, exist_ok=True)
    # Base path without suffix; actual files created below.
    stem_path = results_dir / f"{run_id}_{task_label}_{plot_label}"

    # Try to save as a native MATLAB .fig (returned path includes extension).
    fig_path = save_matlab_fig(fig, str(stem_path))
    if fig_path is not None:
        # Always create a PNG sibling for quick viewing.
        try:
            fig.savefig(stem_path.with_suffix(".png"), **kwargs)
        except Exception:  # pragma: no cover - best effort
            pass
        # Additionally honour requested extension if different from PNG.
        if ext != "png":
            try:
                alt_path = stem_path.with_suffix(f".{ext}")
                fig.savefig(alt_path, **kwargs)
                _saved[task_label].append(alt_path.name)
                return alt_path
            except Exception:  # pragma: no cover - best effort
                _saved[task_label].append(fig_path.name)
                return fig_path
        _saved[task_label].append(fig_path.name)
        return fig_path

    # Fallback: save PNG + MAT (+ MAT-based FIG via utils_legacy)
    png_path, _ = save_png_and_mat(fig, str(stem_path), **kwargs)
    if ext != "png":
        try:
            alt_path = stem_path.with_suffix(f".{ext}")
            fig.savefig(alt_path, **kwargs)
            _saved[task_label].append(alt_path.name)
            return alt_path
        except Exception:  # pragma: no cover - best effort
            pass
    _saved[task_label].append(png_path.name)
    return png_path


def task_summary(task_label: str) -> None:
    """Print a summary of saved plots for *task_label*."""
    n = task_label.replace("task", "")
    # Figures reach disk through several helpers, so consult the shared
    # registry as well; otherwise this reported "No plots saved" while the
    # files were sitting in results/.
    import re

    from utils.matlab_fig_export import WRITTEN

    pattern = re.compile(rf"_task{re.escape(n)}(?:_\d+)*_", re.IGNORECASE)
    files = list(dict.fromkeys(
        list(_saved.get(task_label, [])) + [f for f in WRITTEN if pattern.search(f)]
    ))
    if not files:
        print(f"[TASK {n}] No plots saved.")
        return
    print(f"[TASK {n}] Plots saved ({len(files)}):")
    for name in sorted(files):
        print(f"  {name}")
