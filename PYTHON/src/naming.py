"""Utility functions for standardised file naming.

These helpers mirror the MATLAB `naming_utils` functions and are used
across the codebase to generate dataset-method tags, script names and
output filenames. The goal is to keep naming consistent between Python
and MATLAB implementations.
"""
from pathlib import Path
from typing import Union

def make_tag(dataset: str, gnss: str, method: str) -> str:
    """Return a tag like ``TRIAD_IMU_X001_GNSS_X001``.

    Method first, then the IMU dataset, then the GNSS dataset, so files sort by
    method and the two inputs stay adjacent and readable.
    """
    dname = Path(dataset).stem
    gname = Path(gnss).stem
    return f"{method}_{dname}_{gname}"

def script_name(dataset: str, method: str, task: int, ext: str = "py") -> str:
    """Return a script filename for a given dataset, method and task."""
    dname = Path(dataset).stem
    return f"{dname}_{method}_task{task}.{ext}"

def output_dir(
    task: int,
    dataset: str,
    gnss: str,
    method: str,
    base_dir: Union[str, Path] = "results",
) -> Path:
    """Return ``base_dir`` for backward compatibility.

    Historically, outputs were written to subfolders like ``results/taskN/<tag>``
    but the updated convention stores everything directly under ``results/`` with
    the task number encoded in the filename.  This helper now simply returns the
    base directory so existing code continues to work without modification.
    """

    return Path(base_dir)


def results_path(filename: str, base_dir: Union[str, Path] = "results") -> Path:
    """Return ``Path`` to *filename* inside ``results``.

    Parameters
    ----------
    filename : str
        Name of the file to be saved.
    base_dir : str or Path, optional
        Base results directory. Defaults to ``"results"``.
    """

    return Path(base_dir) / filename

def plot_filename(dataset: str, gnss: str, method: str, task: int, subtask: str, out_type: str, ext: str = "pdf") -> str:
    """Return a plot filename following the standard convention."""
    tag = make_tag(dataset, gnss, method)
    return f"{tag}_task{task}_{subtask}_{out_type}.{ext}"

def plot_path(directory: Union[str, Path], tag: str, task: int, subtask: str, out_type: str, ext: str = "pdf") -> Path:
    """Return ``Path`` for a plot following the standard convention."""
    directory = Path(directory)
    fname = f"{tag}_task{task}_{subtask}_{out_type}.{ext}"
    return directory / fname
