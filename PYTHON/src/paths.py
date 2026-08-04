from __future__ import annotations

import logging
from pathlib import Path

logger = logging.getLogger(__name__)

# Core resolvers
ROOT = Path(__file__).resolve().parents[2]   # PYTHON/src -> PYTHON -> ROOT
DATA_DIR   = ROOT / "DATA"
IMU_DIR    = DATA_DIR / "IMU"
GNSS_DIR   = DATA_DIR / "GNSS"
TRUTH_DIR  = DATA_DIR / "Truth"
PY_RES_DIR = ROOT / "results"                # single results dir for the project

def repo_root() -> Path:
    """Return the repository root directory."""
    return ROOT

def data_dir() -> Path:
    """Return the repository DATA directory."""
    return DATA_DIR

def imu_path(name_or_dataset: str) -> Path:
    """Return IMU data path.

    Accepts either a full filename like ``IMU_X002.dat`` or a dataset id like
    ``X002`` and builds the correct path under ``DATA/IMU``.
    """
    n = name_or_dataset
    p = Path(n)
    # Absolute path: use as-is
    if p.is_absolute():
        return p
    # Path-like input: if it exists, use as-is; else treat as under IMU_DIR
    if '/' in n or '\\' in n:
        return p if p.exists() else (IMU_DIR / n)
    # Dataset id or bare filename: normalise under IMU_DIR
    return IMU_DIR / (n if n.endswith('.dat') or n.startswith('IMU_') else f"IMU_{n}.dat")

def gnss_path(name_or_dataset: str) -> Path:
    """Return GNSS data path.

    Accepts either a full filename like ``GNSS_X002.csv`` or a dataset id like
    ``X002`` and builds the correct path under ``DATA/GNSS``.
    """
    n = name_or_dataset
    p = Path(n)
    if p.is_absolute():
        return p
    if '/' in n or '\\' in n:
        return p if p.exists() else (GNSS_DIR / n)
    return GNSS_DIR / (n if n.endswith('.csv') or n.startswith('GNSS_') else f"GNSS_{n}.csv")

def truth_path(name: str = "STATE_X001.txt") -> Path:
    return TRUTH_DIR / name

def python_results_dir() -> Path:
    """Return the PYTHON results directory path (does not create)."""
    return PY_RES_DIR

def ensure_results_dir() -> Path:
    """Ensure the Python results directory exists and return its path."""
    PY_RES_DIR.mkdir(parents=True, exist_ok=True)
    return PY_RES_DIR


def available_imu_files(include_small: bool = False) -> list[str]:
    """Return a sorted list of IMU data files.

    Parameters
    ----------
    include_small : bool, optional
        If ``True``, include files with the ``_small`` suffix.
    """

    return sorted(
        p.name
        for p in IMU_DIR.glob("IMU_*.dat")
        if include_small or "_small" not in p.stem
    )


def available_gnss_files(include_small: bool = False) -> list[str]:
    """Return a sorted list of GNSS data files.

    Parameters
    ----------
    include_small : bool, optional
        If ``True``, include files with the ``_small`` suffix.
    """

    return sorted(
        p.name
        for p in GNSS_DIR.glob("GNSS_*.csv")
        if include_small or "_small" not in p.stem
    )


def available_dataset_ids(include_small: bool = False) -> list[str]:
    """Return dataset identifiers with available IMU or GNSS data.

    Parameters
    ----------
    include_small : bool, optional
        If ``True``, include datasets with the ``_small`` suffix.
    """

    imu_ids = {f[4:-4] for f in available_imu_files(include_small)}
    gnss_ids = {f[5:-4] for f in available_gnss_files(include_small)}
    return sorted(imu_ids | gnss_ids)


# Backwards compatibility helper.  Older code imported ``ensure_py_results``
# which simply ensured the directory existed without returning it.  Re-export
# the new helper under that name so existing imports keep working.
def ensure_py_results() -> Path:  # pragma: no cover - compatibility shim
    return ensure_results_dir()


def normalize_gnss_headers(df):
    """Return a copy of ``df`` with normalised GNSS column headers.

    Column names are stripped of leading/trailing whitespace to avoid
    downstream import errors.  The dataframe is otherwise unchanged.
    """

    return df.rename(columns=lambda c: c.strip())


def default_dataset_pairs() -> list[tuple[str, str]]:
    """Return default list of ``(IMU, GNSS)`` dataset file pairs.

    The function scans :data:`IMU_DIR` and :data:`GNSS_DIR` and returns only
    pairs for which both an IMU and matching GNSS file exist.  IMU files without
    a corresponding GNSS file are skipped with a warning.  If no GNSS files are
    present an exception is raised to alert the caller.
    """

    imu_files = sorted(IMU_DIR.glob("IMU_*.dat"))
    gnss_files = {p.stem.split("_", 1)[1]: p.name for p in GNSS_DIR.glob("GNSS_*.csv")}

    if not gnss_files:
        raise FileNotFoundError(f"No GNSS files found in {GNSS_DIR}")

    pairs: list[tuple[str, str]] = []
    for imu in imu_files:
        dataset = imu.stem.split("_", 1)[1]
        gnss = gnss_files.get(dataset)
        if gnss:
            pairs.append((imu.name, gnss))
        else:
            logger.warning("Skipping %s; no matching GNSS file found", imu.name)
    return pairs
