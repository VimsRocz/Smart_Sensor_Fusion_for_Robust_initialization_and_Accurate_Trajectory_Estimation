from __future__ import annotations

import numpy as np
from typing import Tuple, Optional
from matplotlib.figure import Figure
from pathlib import Path

try:
    from pyproj import Transformer
    _ECEF2LLA = Transformer.from_crs("epsg:4978", "epsg:4979", always_xy=True)
except Exception:  # pragma: no cover - optional dependency
    _ECEF2LLA = None
import pathlib
import subprocess
import sys
import logging


def zero_base_time(t):
    """Return time vector shifted to start at zero.

    If *t* is empty, an empty ``float64`` array is returned.
    """
    t = np.asarray(t, dtype=np.float64)
    if t.size == 0:
        return t
    return t - t[0]


def get_data_file(filename: str) -> pathlib.Path:
    """Return full path to a data file.

    Searches the following locations relative to the repository root:

    1. ``MATLAB/data``
    2. ``data``
    3. repository root
    4. ``tests/data``

    Raises ``FileNotFoundError`` if *filename* cannot be located.
    """

    root = pathlib.Path(__file__).resolve().parents[1]
    candidates = [
        root / "MATLAB" / "data" / filename,
        root / "data" / filename,
        root / filename,
        root / "tests" / "data" / filename,
        pathlib.Path(filename),
    ]
    for path in candidates:
        if path.is_file():
            return path
    raise FileNotFoundError(f"Data file not found: {filename}")


def ensure_dependencies(requirements: Optional[pathlib.Path] = None) -> None:
    """Install and verify the dependencies declared beside the scripts.

    Keep this compatibility function because several legacy entry points
    already call it, but delegate the actual check to the standard-library-only
    bootstrap so every entry point uses the same requirements file and import
    mapping.
    """
    try:
        from .dependency_bootstrap import ensure_dependencies as _ensure_dependencies
    except ImportError:  # legacy top-level import
        from dependency_bootstrap import ensure_dependencies as _ensure_dependencies

    _ensure_dependencies(requirements)


def detect_static_interval(
    accel_data,
    gyro_data,
    window_size=200,
    accel_var_thresh=0.01,
    gyro_var_thresh=1e-6,
    min_length=100,
):
    """Find the longest static segment in the data using variance thresholding.

    The search spans the entire dataset to locate the longest interval where
    both accelerometer and gyroscope variances remain below the specified
    thresholds. The defaults are calibrated so zero-velocity updates fire
    during genuine stationary periods without being overly sensitive to noise.

    Parameters
    ----------
    accel_data : ndarray, shape (N,3)
        Raw accelerometer samples.
    gyro_data : ndarray, shape (N,3)
        Raw gyroscope samples.
    window_size : int, optional
        Samples per rolling window.
    accel_var_thresh : float, optional
        Variance threshold for accelerometer (m/s^2)^2.
    gyro_var_thresh : float, optional
        Variance threshold for gyroscope (rad/s)^2.
    min_length : int, optional
        Minimum length in samples of the static segment.

    Returns
    -------
    tuple of int
        (start_idx, end_idx) indices of the detected static interval.
    """
    N = len(accel_data)
    if N < window_size:
        raise ValueError("window_size larger than data length")

    accel_var = np.array([np.var(accel_data[i:i+window_size], axis=0)
                          for i in range(N - window_size)])
    gyro_var = np.array([np.var(gyro_data[i:i+window_size], axis=0)
                         for i in range(N - window_size)])
    max_accel_var = np.max(accel_var, axis=1)
    max_gyro_var = np.max(gyro_var, axis=1)

    static_mask = (max_accel_var < accel_var_thresh) & (max_gyro_var < gyro_var_thresh)

    from itertools import groupby
    from operator import itemgetter
    groups = []
    for k, g in groupby(enumerate(static_mask), key=itemgetter(1)):
        if k:
            g = list(g)
            i0 = g[0][0]
            i1 = g[-1][0] + window_size
            groups.append((i0, i1))

    # pick the longest static group that satisfies the minimum length
    longest = (0, window_size)
    for i0, i1 in groups:
        if i1 - i0 >= min_length and i1 - i0 > longest[1] - longest[0]:
            longest = (i0, i1)

    return longest


def is_static(accel_win, gyro_win, accel_var_thresh=0.01, gyro_var_thresh=1e-6):
    """Check if the current IMU window is static.

    Stricter gating to reduce false ZUPTs:
    - low variance in accel/gyro (existing checks)
    - accel magnitude close to 9.81 m/s² on average
    - small per-axis standard deviations
    """
    try:
        var_a_ok = np.max(np.var(accel_win, axis=0)) < accel_var_thresh
        var_g_ok = np.max(np.var(gyro_win, axis=0)) < gyro_var_thresh
        # Magnitude closeness
        a_norm = np.linalg.norm(accel_win, axis=1)
        mag_err = np.mean(np.abs(a_norm - 9.81))
        mag_ok = mag_err < 0.03
        # Small per-axis std
        std_a_ok = np.max(np.std(accel_win, axis=0)) < 0.02
        std_g_ok = np.max(np.std(gyro_win, axis=0)) < (np.deg2rad(0.05))
        return var_a_ok and var_g_ok and mag_ok and std_a_ok and std_g_ok
    except Exception:
        # Fallback to legacy behaviour if anything goes wrong
        return (
            np.max(np.var(accel_win, axis=0)) < accel_var_thresh
            and np.max(np.var(gyro_win, axis=0)) < gyro_var_thresh
        )

# Additional utilities for logging and thresholding


def log_static_zupt_params(static_start: int, static_end: int,
                           zupt_count: int, threshold: float,
                           static_detected: bool = True) -> None:
    """Log details about the detected static window and ZUPT events."""
    if not static_detected:
        logging.warning("No static segment detected!")
    else:
        logging.info(
            f"Static window: {static_start} to {static_end} "
            f"({static_end-static_start} samples)"
        )
    logging.info(f"ZUPT threshold used: {threshold}")
    logging.info(f"Total ZUPT events: {zupt_count}")


def save_static_zupt_params(filename: str, static_start: int, static_end: int,
                             zupt_count: int, threshold: float) -> None:
    """Save static/ZUPT parameters to a text file."""
    with open(filename, 'w') as f:
        f.write(f"Static start: {static_start}\n")
        f.write(f"Static end: {static_end}\n")
        f.write(f"Static length: {static_end-static_start}\n")
        f.write(f"ZUPT threshold: {threshold}\n")
        f.write(f"Total ZUPT events: {zupt_count}\n")


def adaptive_zupt_threshold(accel_data: np.ndarray, gyro_data: np.ndarray,
                            factor: float = 3.0) -> Tuple[float, float]:
    """Return adaptive thresholds based on early-sample statistics."""
    norm_accel = np.linalg.norm(accel_data, axis=1)
    norm_gyro = np.linalg.norm(gyro_data, axis=1)
    base = min(400, len(norm_accel))
    accel_thresh = np.mean(norm_accel[:base]) + factor * np.std(norm_accel[:base])
    gyro_thresh = np.mean(norm_gyro[:base]) + factor * np.std(norm_gyro[:base])
    return accel_thresh, gyro_thresh


def save_mat(filename: str, data: dict) -> None:
    """Save *data* dictionary to a MATLAB ``.mat`` file.

    Parameters
    ----------
    filename : str
        Destination path for the ``.mat`` file.
    data : dict
        Mapping of variable names to arrays to be written.
    """
    from scipy.io import savemat

    # Store with compression to reduce disk usage
    savemat(filename, data, do_compression=True)


def save_plot_mat(fig: Figure, filename: str) -> None:
    """Save matplotlib figure *fig* in MATLAB ``.mat`` format.

    Parameters
    ----------
    fig : :class:`matplotlib.figure.Figure`
        Figure to serialise.
    filename : str
        Path of the ``.mat`` file.

    Notes
    -----
    This helper exports the plotted line data and basic axis labels so the
    figure can be recreated in MATLAB. The structure mirrors the axes layout
    where each line is stored as ``ax<i>_line<j>_x`` and ``ax<i>_line<j>_y``.
    """

    from fusion_pipeline.figure_export import _write_generic_mat_companion

    _write_generic_mat_companion(fig, Path(filename))


def save_plot_fig(fig: Figure, filename: str) -> None:
    """Save matplotlib figure *fig* as a genuine native MATLAB ``.fig``.

    Parameters
    ----------
    fig : :class:`matplotlib.figure.Figure`
        Figure to serialise.
    filename : str
        Path of the ``.fig`` file.

    A MAT file renamed to ``.fig`` is not a MATLAB figure and cannot be opened
    with ``openfig``. Native serialization requires MATLAB; callers receive a
    clear error instead of a misleading, unloadable file when it is absent.
    """
    from utils.matlab_fig_export import save_matlab_fig

    target = Path(filename)
    written = save_matlab_fig(fig, str(target.with_suffix("")))
    if written is None:
        raise RuntimeError(
            "Native MATLAB .fig export requires MATLAB Engine or the automatic "
            "MATLAB batch export in PYTHON/run_release.py."
        )


def save_png_and_mat(fig: Figure, filename: str, **savefig_kwargs) -> tuple[Path, Path]:
    """Save figure as PNG and MATLAB MAT side-by-side.

    Parameters
    ----------
    fig : Figure
        Matplotlib figure to save.
    filename : str
        Target path; extension is ignored. Both ``.png`` and ``.mat`` are written
        next to each other using the same stem.
    savefig_kwargs : dict
        Extra kwargs passed to ``fig.savefig`` for the PNG (e.g., dpi, bbox_inches).

    Returns
    -------
    tuple of Path
        ``(png_path, mat_path)`` of the saved files.
    """
    base = Path(filename)
    stem = base.with_suffix("")
    png_path = stem.with_suffix(".png")
    mat_path = stem.with_suffix(".mat")
    png_path.parent.mkdir(parents=True, exist_ok=True)
    # Save PNG (best-effort) — tolerate disk-full errors gracefully
    try:
        fig.savefig(png_path, **savefig_kwargs)
    except OSError as e:
        print(f"[WARN] Skipping PNG save (reason: {e}). Path: {png_path}")
    # Save MAT (best-effort)
    try:
        save_plot_mat(fig, str(mat_path))
    except OSError as e:
        print(f"[WARN] Skipping MAT save (reason: {e}). Path: {mat_path}")
    # Additionally try to save a MATLAB-native .fig via engine.
    # Do NOT write MAT-based '.fig' fallbacks to avoid confusing non-native files.
    try:
        from utils.matlab_fig_export import save_matlab_fig
        save_matlab_fig(fig, str(stem))
    except Exception:
        # Engine unavailable — skip .fig to prevent non-native '.fig' files.
        pass
    return png_path, mat_path


def compute_C_ECEF_to_NED(lat: float, lon: float) -> np.ndarray:
    """Return rotation matrix from ECEF to NED frame.

    Parameters
    ----------
    lat : float
        Geodetic latitude in radians.
    lon : float
        Longitude in radians.

    Returns
    -------
    ndarray of shape (3, 3)
        Rotation matrix from ECEF to NED.
    """
    s_lat = np.sin(lat)
    c_lat = np.cos(lat)
    s_lon = np.sin(lon)
    c_lon = np.cos(lon)
    return np.array([
        [-s_lat * c_lon, -s_lat * s_lon, c_lat],
        [-s_lon, c_lon, 0.0],
        [-c_lat * c_lon, -c_lat * s_lon, -s_lat],
    ])


def ecef_to_ned(pos_ecef: np.ndarray, ref_lat: float, ref_lon: float,
                ref_ecef: np.ndarray) -> np.ndarray:
    """Convert ECEF coordinates to NED frame relative to *ref_ecef*."""
    pos_ecef = np.asarray(pos_ecef)
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    if pos_ecef.ndim == 1:
        return C @ (pos_ecef - ref_ecef)
    return np.array([C @ (p - ref_ecef) for p in pos_ecef])


def euler_to_rot(eul: np.ndarray) -> np.ndarray:
    """Convert XYZ Euler angles to a rotation matrix.

    Parameters
    ----------
    eul : array-like, shape (3,)
        ``[roll, pitch, yaw]`` in **radians**.

    Returns
    -------
    ndarray of shape (3, 3)
        Rotation matrix from Body to NED using the XYZ convention.
    """

    eul = np.asarray(eul).reshape(3)
    cr, sr = np.cos(eul[0]), np.sin(eul[0])
    cp, sp = np.cos(eul[1]), np.sin(eul[1])
    cy, sy = np.cos(eul[2]), np.sin(eul[2])
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def ecef_to_geodetic(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert ECEF coordinates to geodetic latitude, longitude and altitude.

    Parameters
    ----------
    x, y, z : float
        Coordinates in the Earth-Centred Earth-Fixed (ECEF) frame, metres.

    Returns
    -------
    tuple of float
        ``(latitude_deg, longitude_deg, altitude_m)`` using the WGS‑84 model.
    """
    if _ECEF2LLA is not None:
        lon, lat, alt = _ECEF2LLA.transform(x, y, z)
        return float(lat), float(lon), float(alt)

    a = 6378137.0
    e_sq = 6.69437999014e-3
    p = np.sqrt(x ** 2 + y ** 2)
    lon = np.arctan2(y, x)

    # Initial latitude estimate (Bowring's formula)
    b = a * np.sqrt(1 - e_sq)
    theta = np.arctan2(z * a, p * b)
    lat = np.arctan2(
        z + (e_sq * b) * np.sin(theta) ** 3,
        p - e_sq * a * np.cos(theta) ** 3,
    )

    # Iterate to refine latitude and altitude; typically converges in <5 steps
    alt = 0.0
    for _ in range(5):
        N = a / np.sqrt(1 - e_sq * np.sin(lat) ** 2)
        alt_prev = alt
        alt = p / np.cos(lat) - N
        lat_prev = lat
        lat = np.arctan2(z, p * (1 - e_sq * N / (N + alt)))
        if abs(lat - lat_prev) < 1e-12 and abs(alt - alt_prev) < 1e-9:
            break

    return float(np.degrees(lat)), float(np.degrees(lon)), float(alt)


def normal_gravity(lat: float, h: float = 0.0) -> float:
    """Return gravity magnitude at latitude ``lat`` and height ``h``.

    Parameters
    ----------
    lat : float
        Geodetic latitude in radians.
    h : float, optional
        Height above the ellipsoid in metres.

    Returns
    -------
    float
        Gravity magnitude in m/s² using the WGS‑84 model.
    """
    # Allow ``lat`` and ``h`` to be passed as numpy arrays of size one or other
    # array-like types.  ``np.asarray(x).item()`` reliably extracts the scalar
    # value without raising "setting an array element with a sequence" errors
    # when these functions are called with values produced by interpolation
    # routines.
    lat = np.asarray(lat).item()
    h = np.asarray(h).item()
    sin_lat = np.sin(lat)
    g = (
        9.7803253359
        * (1 + 0.00193185265241 * sin_lat**2)
        / np.sqrt(1 - 0.00669437999013 * sin_lat**2)
    )
    return g - 3.086e-6 * h


def gravity_ecef(lat: float, lon: float, h: float = 0.0) -> np.ndarray:
    """Return gravity vector in ECEF coordinates."""
    # Accept latitude, longitude and height provided as scalars or length-one
    # arrays.  ``np.asarray(x).item()`` extracts the underlying float so
    # downstream computations operate on plain Python scalars even if callers
    # supply numpy scalars or arrays.
    lat = np.asarray(lat).item()
    lon = np.asarray(lon).item()
    h = np.asarray(h).item()
    g_ned = np.array([0.0, 0.0, normal_gravity(lat, h)])
    return compute_C_ECEF_to_NED(lat, lon).T @ g_ned


def validate_gravity_vector(lat_deg: float, h: float = 0.0) -> np.ndarray:
    """Print and return the gravity vector in NED coordinates.

    Parameters
    ----------
    lat_deg : float
        Geodetic latitude in degrees.
    h : float, optional
        Height above the ellipsoid in metres.

    Returns
    -------
    ndarray of shape (3,)
        Gravity vector ``[0, 0, g]`` in the NED frame.
    """
    lat_rad = np.deg2rad(lat_deg)
    g = normal_gravity(lat_rad, h)
    print(
        f"[Gravity Validation] Latitude: {lat_deg:.3f} deg, altitude: {h:.1f} m"
        f" --> Gravity: {g:.6f} m/s^2 (NED +Z is down)"
    )
    return np.array([0.0, 0.0, g])


def interpolate_series(
    t_ref: np.ndarray, t_data: np.ndarray, series: np.ndarray
) -> np.ndarray:
    """Interpolate *series* to match *t_ref* using linear interpolation.

    Parameters
    ----------
    t_ref : ndarray, shape (N,)
        Reference timestamps in seconds.
    t_data : ndarray, shape (M,)
        Timestamps corresponding to ``series``.
    series : ndarray, shape (M, ...)``
        Samples to be interpolated along the first dimension.

    Returns
    -------
    ndarray
        ``series`` resampled so that ``series[i]`` corresponds to ``t_ref[i]``.
    """

    t_ref = np.asarray(t_ref)
    t_data = np.asarray(t_data)
    series = np.asarray(series)

    if series.ndim == 1:
        return np.interp(t_ref, t_data, series)

    out = np.vstack([np.interp(t_ref, t_data, series[:, i]) for i in range(series.shape[1])]).T
    return out
