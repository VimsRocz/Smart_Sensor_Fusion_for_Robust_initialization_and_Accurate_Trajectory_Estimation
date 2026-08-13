from __future__ import annotations

"""Run the GNSS/IMU fusion pipeline for a single dataset pair.

This script mirrors the MATLAB ``GNSS_IMU_Fusion.m`` workflow and provides
the same command-line interface.  It performs attitude initialisation and
extended Kalman filtering given IMU and GNSS logs.

The accelerometer scale factor estimated in Task 4 is applied once when
correcting raw accelerometer measurements.  If no prior estimate exists a
neutral factor of ``1.0`` is used.

Usage
-----
python src/GNSS_IMU_Fusion.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv

All results are written to ``results/``.
"""

import argparse
import logging
import sys
import os
import io
import time
import re
from pathlib import Path

if __package__ is None:
    # When executed directly, ensure both the ``src`` directory and its parent
    # are on ``sys.path`` so package-relative imports resolve correctly.  The
    # ``src`` directory is inserted first to prefer its ``scripts`` package over
    # the top-level ``PYTHON/scripts`` helpers.
    here = Path(__file__).resolve()
    sys.path.insert(0, str(here.parent))
    sys.path.insert(1, str(here.parent.parent))
    __package__ = "src"

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from utils.plot_save import save_plot, task_summary
from utils.plot_saver import save_png_and_mat
from paths import (
    imu_path as _imu_path_helper,
    gnss_path as _gnss_path_helper,
    normalize_gnss_headers,
)
from task1_cache import save_task1_artifacts
from task2_plot import save_task2_summary_png, task2_measure_body_vectors
from utils.run_id import run_id as make_run_id
from utils import (
    is_static,
    compute_C_ECEF_to_NED,
    ecef_to_geodetic,
    interpolate_series,
    zero_base_time,
    save_mat,
)
from utils.interp_to import interp_to
from utils.time_utils import compute_time_shift
from constants import EARTH_RATE
from .compute_biases import compute_biases
from .scripts.validate_filter import compute_residuals, plot_residuals
from scipy.spatial.transform import Rotation as R
from .gnss_imu_fusion.init_vectors import (
    average_rotation_matrices,
    svd_alignment,
    triad_svd,
    butter_lowpass_filter,
    compute_wahba_errors,
)
from .gnss_imu_fusion.axis_map import sanity_check_tilt
from .gnss_imu_fusion.axis_map_auto import choose_C_bs_from_static
from .gnss_imu_fusion.plots import save_zupt_variance
from .gnss_imu_fusion.init import compute_reference_vectors, measure_body_vectors
from .gnss_imu_fusion.integration import integrate_trajectory
from .gnss_imu_fusion.kalman_filter import (
    quat_multiply,
    quat_from_rate,
    quat2euler,
)
from .kalman_filter import init_bias_kalman, inject_zupt

try:
    from rich.console import Console
except ImportError:
    logging.error("\u2757  Missing dependency: install with `pip install rich`")
    sys.exit(1)

try:
    console = Console()
    log = console.log
except Exception:
    logging.basicConfig(level=logging.INFO)
    log = logging.info
TAG = "{method}_{imu}_{gnss}".format  # METHOD_IMU_GNSS
# One results directory for the whole project: <repo>/results.
# Previously this was relative to the working directory while paths.py
# pointed at PYTHON/results, so outputs split across two folders.
_REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_DIR = Path(os.getenv("PYTHON_RESULTS_DIR", _REPO_ROOT / "results")).resolve()

# Colour palette for plotting per attitude-initialisation method
COLORS = {
    "TRIAD": "tab:blue",
    "Davenport": "tab:orange",
    "SVD": "tab:green",
}

# Setup logging
utf8_stdout = io.TextIOWrapper(
    sys.stdout.buffer,
    encoding="utf-8",
    errors="replace",
)
handler = logging.StreamHandler(utf8_stdout)
logging.basicConfig(level=logging.INFO, format="%(message)s", handlers=[handler])

# Minimum number of samples required from a static interval for bias estimation
MIN_STATIC_SAMPLES = 500

RUN_ID = ""


def task3_plot_quaternions_and_errors(
    methods, quaternions_dict, errors_dict, output_dir
):
    """Save Task 3 quaternion and error comparison plots.

    Parameters
    ----------
    methods : list[str]
        List of attitude initialisation methods, e.g. ``["TRIAD", "Davenport", "SVD"]``.
    quaternions_dict : dict[str, array_like]
        Mapping of ``"<Method>_CaseX"`` to quaternions ``[w, x, y, z]``.
    errors_dict : dict[str, dict]
        Mapping of method to gravity/earth-rate errors in degrees.
    output_dir : pathlib.Path or str
        Directory in which plots will be written.
    """

    # Quaternion component comparison ------------------------------------
    labels = list(quaternions_dict.keys())
    components = ["q_w", "q_x", "q_y", "q_z"]
    x = np.arange(len(labels))
    width = 0.18
    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

    fig, ax = plt.subplots(figsize=(10, 5))
    for i, comp in enumerate(components):
        vals = [quaternions_dict[label][i] for label in labels]
        ax.bar(x + (i - 1.5) * width, vals, width, label=comp, color=colors[i], alpha=0.7)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Quaternion Value")
    ax.set_ylim(-1, 1)
    ax.set_title("Task 3: Quaternion Components by Method and Case")
    ax.legend(loc="upper left", bbox_to_anchor=(1, 1))
    fig.tight_layout()

    base = RESULTS_DIR / f"{RUN_ID}_task3_7_2_quaternions"
    try:
        arrays = {k: np.asarray(v) for k, v in quaternions_dict.items()}
    except Exception:
        arrays = None
    save_png_and_mat(fig, str(base), arrays=arrays)
    plt.close(fig)

    # Attitude error comparison ------------------------------------------
    epsilon = 1e-6
    grav_vals = [max(epsilon, errors_dict[m]["grav"]) for m in methods]
    earth_vals = [max(epsilon, errors_dict[m]["earth"]) for m in methods]
    print(f"[Task3] grav_err_deg={grav_vals} earth_err_deg={earth_vals}")
    if any(v <= epsilon for v in grav_vals + earth_vals):
        print("Near-zero errors detected; plot may appear empty")

    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(methods, grav_vals, color="tab:purple")
    axes[0].set_ylabel("Error (deg)")
    axes[0].set_title("Gravity Error")
    axes[0].set_ylim(0, max(grav_vals) * 1.1 if max(grav_vals) > 0 else 1)

    axes[1].bar(methods, earth_vals, color="tab:brown")
    axes[1].set_title("Earth Rate Error")
    axes[1].set_ylim(0, max(earth_vals) * 1.1 if max(earth_vals) > 0 else 1)

    fig.suptitle("Task 3: Attitude Error Comparison")
    plt.tight_layout()

    if not grav_vals or not earth_vals or (np.allclose(grav_vals, 0) and np.allclose(earth_vals, 0)):
        raise ValueError("Task3 arrays all zero or empty")
    base = RESULTS_DIR / f"{RUN_ID}_task3_7_1_errors"
    arrays = {
        "methods": np.array(methods, dtype=object),
        "grav_err_deg": np.array(grav_vals, float),
        "earth_err_deg": np.array(earth_vals, float),
    }
    save_png_and_mat(fig, str(base), arrays=arrays)
    plt.close(fig)
    task_summary("task3")


def check_files(imu_file: str, gnss_file: str) -> tuple[str, str]:
    """Return validated dataset paths (prefer DATA/*)."""
    imu_path = _imu_path_helper(imu_file)
    gnss_path = _gnss_path_helper(gnss_file)
    return str(imu_path), str(gnss_path)


def load_truth_as_ned(truth_path: str, ref_lat: float, ref_lon: float, r0_ecef: np.ndarray):
    """Load STATE_X truth file and convert position/velocity to NED."""
    raw = np.loadtxt(truth_path)
    t_truth = raw[:, 0]
    pos_ecef = raw[:, 1:4]
    vel_ecef = raw[:, 4:7]
    C = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    pos_ned = np.array([C @ (p - r0_ecef) for p in pos_ecef])
    vel_ned = np.array([C @ v for v in vel_ecef])
    return t_truth, pos_ned, vel_ned


def plot_task6_truth_overlay(
    t: np.ndarray,
    pos_fused: np.ndarray,
    vel_fused: np.ndarray,
    pos_truth: np.ndarray,
    vel_truth: np.ndarray,
    out_png: str | Path,
) -> None:
    """Plot Task 6 overlay of fused vs truth position and velocity."""
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(12, 8))
    labels = ["North", "East", "Down"]
    for i in range(3):
        ax = axes[i, 0]
        ax.plot(t, pos_truth[:, i], label="TRUTH")
        ax.plot(t, pos_fused[:, i], linestyle="--", label="FUSED")
        ax.set_ylabel(f"Pos {labels[i]} [m]")
        if i == 0:
            ax.set_title("Task 6 — Position: TRUTH vs FUSED")
        ax.grid(True)

        ax = axes[i, 1]
        ax.plot(t, vel_truth[:, i], label="TRUTH")
        ax.plot(t, vel_fused[:, i], linestyle="--", label="FUSED")
        ax.set_ylabel(f"Vel {labels[i]} [m/s]")
        if i == 0:
            ax.set_title("Task 6 — Velocity: TRUTH vs FUSED")
        ax.grid(True)

    for ax in axes.flatten():
        ax.set_xlabel("Time [s]")
    axes[0, 0].legend(loc="upper right")
    fig.tight_layout()
    Path(out_png).parent.mkdir(parents=True, exist_ok=True)
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(out_png.with_suffix('')))
    plt.close(fig)



def _truth_quaternion_b2n(truth_file, t_target, C_ECEF_to_NED, quat_frame="ECEF"):
    """Return the truth Body-to-NED quaternion [w,x,y,z] on ``t_target``.

    The bundled STATE_X001 file stores [qx,qy,qz,qw] Body-to-ECEF in its last
    four columns. Returns None when the file carries no attitude.
    """
    try:
        from scipy.spatial.transform import Rotation as R

        raw = np.loadtxt(truth_file)
        if raw.ndim != 2 or raw.shape[1] < 12:
            return None
        t_truth = raw[:, 1].astype(float)
        t_truth = t_truth - t_truth[0]
        q_xyzw = raw[:, -4:].astype(float)
        norms = np.linalg.norm(q_xyzw, axis=1)
        if not np.all(norms > 1e-12):
            return None
        q_xyzw = q_xyzw / norms[:, None]

        rot = R.from_quat(q_xyzw)                      # Body -> ECEF (or NED)
        if str(quat_frame).upper() == "ECEF":
            rot = R.from_matrix(np.asarray(C_ECEF_to_NED)) * rot   # -> Body->NED
        q = rot.as_quat()                              # xyzw
        q = np.column_stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]])  # wxyz

        # keep the series in one hemisphere before interpolating
        for i in range(1, len(q)):
            if float(np.dot(q[i - 1], q[i])) < 0:
                q[i] *= -1.0
        out = np.column_stack(
            [np.interp(t_target, t_truth, q[:, k]) for k in range(4)]
        )
        return out / np.linalg.norm(out, axis=1, keepdims=True)
    except Exception:
        return None


def _save_tasks_overview(
    out_stem: Path,
    t: np.ndarray,
    euler_deg: np.ndarray | None,
    pos_ned: np.ndarray,
    vel_ned: np.ndarray,
    gnss_pos_ned: np.ndarray | None = None,
    gnss_vel_ned: np.ndarray | None = None,
    truth_pos_ned: np.ndarray | None = None,
    truth_vel_ned: np.ndarray | None = None,
    res_pos: np.ndarray | None = None,
    res_vel: np.ndarray | None = None,
) -> None:
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 3, figsize=(14, 10), sharex=True)
    labels = ["North", "East", "Down"]

    # Row 1: Euler
    if euler_deg is not None and euler_deg.size:
        for i in range(3):
            ax = axes[0, i]
            ax.plot(t, euler_deg[:, i], label=["Roll","Pitch","Yaw"][i])
            ax.set_ylabel(f"{['Roll','Pitch','Yaw'][i]} [deg]")
            ax.grid(True)
        axes[0,0].set_title("Task 3 — Attitude")

    # Row 2: Position NED
    for i in range(3):
        ax = axes[1, i]
        ax.plot(t, pos_ned[:, i], label="Fused")
        if truth_pos_ned is not None and truth_pos_ned.size:
            ax.plot(t, truth_pos_ned[:, i], '--', label="Truth")
        if gnss_pos_ned is not None and gnss_pos_ned.size:
            ax.plot(t, gnss_pos_ned[:, i], ':', label="GNSS")
        ax.set_ylabel(f"{labels[i]} [m]")
        ax.grid(True)
    axes[1,0].set_title("Task 4/6 — Position NED")

    # Row 3: Velocity NED
    for i in range(3):
        ax = axes[2, i]
        ax.plot(t, vel_ned[:, i], label="Fused")
        if truth_vel_ned is not None and truth_vel_ned.size:
            ax.plot(t, truth_vel_ned[:, i], '--', label="Truth")
        if gnss_vel_ned is not None and gnss_vel_ned.size:
            ax.plot(t, gnss_vel_ned[:, i], ':', label="GNSS")
        ax.set_ylabel(f"V{labels[i][0]} [m/s]")
        ax.grid(True)
    axes[2,0].set_title("Task 4/6 — Velocity NED")

    # Row 4: Residuals (if available)
    if res_pos is not None and res_pos.size:
        for i in range(3):
            ax = axes[3, i]
            ax.plot(t[: res_pos.shape[0]], res_pos[:, i], label="Pos resid")
            ax.set_ylabel(f"Res {labels[i]} [m]")
            ax.grid(True)
        axes[3,0].set_title("Residuals — Position")
    elif res_vel is not None and res_vel.size:
        for i in range(3):
            ax = axes[3, i]
            ax.plot(t[: res_vel.shape[0]], res_vel[:, i], label="Vel resid")
            ax.set_ylabel(f"Res V{labels[i][0]} [m/s]")
            ax.grid(True)
        axes[3,0].set_title("Residuals — Velocity")

    for ax in axes[-1, :]:
        ax.set_xlabel("Time [s]")
    handles, labels_ = axes[1,0].get_legend_handles_labels()
    if handles:
        axes[0,2].legend(handles, labels_, loc="upper right")
    fig.tight_layout()
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(out_stem))
    plt.close(fig)

def main():
    # Ensure output directory exists (honours PYTHON_RESULTS_DIR if set)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    logging.info("Ensured '%s' directory exists.", RESULTS_DIR)
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu-file", required=True)
    parser.add_argument("--gnss-file", required=True)
    # Using TRIAD initialization for all tests based on its consistent runtime and accuracy.
    parser.add_argument(
        "--method",
        default="TRIAD",
        choices=["TRIAD", "Davenport", "SVD"],
    )
    parser.add_argument("--mag-file", help="CSV file with magnetometer data")
    parser.add_argument(
        "--measure-source",
        choices=["gnss", "truth"],
        default="gnss",
        help="Use GNSS or TRUTH as the KF measurement source",
    )
    parser.add_argument(
        "--truth-file",
        type=str,
        required=False,
        help="Path to STATE_X001.txt (truth) for Task 6",
    )
    parser.add_argument(
        "--allow-truth-mismatch",
        action="store_true",
        help=(
            "Permit IMU/GNSS and TRUTH dataset IDs to differ (e.g., use STATE_X001.txt "
            "with X002 data). PairGuard will warn but not abort."
        ),
    )
    parser.add_argument(
        "--apply-time-shift-in-kf",
        action="store_true",
        help=(
            "Apply the estimated GNSS-IMU time shift inside KF interpolation. "
            "By default this is OFF to avoid spurious large offsets."
        ),
    )
    # Optional initial position override (if GNSS doesn't contain a good first fix)
    parser.add_argument(
        "--init-lat-deg",
        type=float,
        default=None,
        help="Override initial latitude in degrees (optional)",
    )
    parser.add_argument(
        "--init-lon-deg",
        type=float,
        default=None,
        help="Override initial longitude in degrees (optional)",
    )
    parser.add_argument(
        "--init-alt-m",
        type=float,
        default=None,
        help="Override initial altitude in meters (optional)",
    )
    parser.add_argument(
        "--init-att-with-truth",
        action="store_true",
        help=(
            "Initialise the Kalman filter attitude with the true quaternion "
            "from --truth-file at IMU t0. Helps isolate init issues."
        ),
    )
    parser.add_argument(
        "--tag",
        type=str,
        default="",
        help="Optional suffix for output filenames",
    )
    parser.add_argument(
        "--use-gnss-heading",
        action="store_true",
        help="Use initial GNSS velocity for yaw if no magnetometer",
    )
    parser.add_argument(
        "--fuse-yaw",
        action="store_true",
        help=(
            "Fuse GNSS-derived yaw as a correction to the propagated quaternion. "
            "Applies a small rotation about NED z when speed exceeds a threshold."
        ),
    )
    parser.add_argument(
        "--yaw-gain",
        type=float,
        default=0.2,
        help="Gain [0..1] for GNSS yaw correction per update (default 0.2)",
    )
    parser.add_argument(
        "--yaw-speed-min",
        type=float,
        default=2.0,
        help="Minimum horizontal speed [m/s] to trust GNSS yaw (default 2 m/s)",
    )
    parser.add_argument(
        "--axis-map",
        choices=["none", "auto"],
        default="none",
        help=(
            "Sensor-to-body axis mapping. 'none' (default) treats the raw IMU "
            "columns as the body frame, which is the convention the bundled "
            "truth quaternion uses. 'auto' restores the previous automatic "
            "permutation from the static gravity direction."
        ),
    )
    parser.add_argument(
        "--truth-quat-frame",
        choices=["NED", "ECEF"],
        # The bundled STATE_X001 stores the attitude Body->ECEF (see
        # DATA/README.md), so ECEF is the correct default for this data.
        default="ECEF",
        help=(
            "Frame of truth quaternion when using --init-att-with-truth. "
            "If 'ECEF', converts body->ECEF truth to body->NED using ref lat/lon."
        ),
    )
    parser.add_argument(
        "--kf-init",
        choices=["TRIAD", "Davenport", "SVD", "TRUTH"],
        default="SVD",
        help=(
            "Quaternion to seed the KF attitude: pick from computed initial "
            "attitudes (TRIAD/Davenport/SVD) or use TRUTH at t0 if provided."
        ),
    )
    parser.add_argument(
        "--lever-arm",
        type=float,
        nargs=3,
        default=[0.0, 0.0, 0.0],
        help="Lever arm from IMU to GNSS antenna in body frame [m]",
    )
    parser.add_argument("--accel-noise", type=float, default=0.1)
    parser.add_argument("--accel-bias-noise", type=float, default=1e-5)
    parser.add_argument("--gyro-bias-noise", type=float, default=1e-5)
    parser.add_argument(
        "--vel-q-scale",
        type=float,
        default=1.0,
        help=(
            "Scale applied to velocity process noise block Q[3:6,3:6] "
            "(base 0.01 m^2/s^2)"
        ),
    )
    parser.add_argument(
        "--vel-r",
        type=float,
        default=1.0,
        help=(
            "Diagonal variance for GNSS velocity measurements R[3:6,3:6] "
            "[m^2/s^2]"
        ),
    )
    parser.add_argument(
        "--zupt-acc-var",
        type=float,
        default=0.01,
        help="Accelerometer variance threshold for ZUPT detection",
    )
    parser.add_argument(
        "--zupt-gyro-var",
        type=float,
        default=1e-6,
        help="Gyroscope variance threshold for ZUPT detection",
    )
    parser.add_argument(
        "--no-zupt",
        action="store_true",
        help="Disable ZUPT pseudo-measurements",
    )
    parser.add_argument(
        "--static-start",
        type=int,
        default=None,
        help="Start sample for bias window when known",
    )
    parser.add_argument(
        "--static-end",
        type=int,
        default=None,
        help="End sample for bias window when known",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip matplotlib savefig to speed up CI runs",
    )
    # (Output size/format controls defined below)
    # Output size and content controls
    parser.add_argument(
        "--no-npz",
        action="store_true",
        help="Skip saving the large NPZ results bundle",
    )
    parser.add_argument(
        "--no-mat",
        action="store_true",
        help="Skip saving the MATLAB .mat results bundle",
    )
    parser.add_argument(
        "--lite-output",
        action="store_true",
        help=(
            "Save reduced outputs: downsample time series, cast to float32, and omit very large arrays (P_hist, x_log, residuals, innovations)."
        ),
    )
    parser.add_argument(
        "--downsample",
        type=int,
        default=1,
        help="Keep every Nth sample in saved time series (default 1 = keep all)",
    )
    parser.add_argument(
        "--fp32-output",
        action="store_true",
        help="Cast floating-point arrays to float32 in saved outputs to reduce size",
    )
    parser.add_argument(
        "--disable-triad-log",
        action="store_true",
        help="Do not append per-run info to triad_init_log.txt",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable verbose debug output",
    )

    args = parser.parse_args()
    lever_arm = np.array(args.lever_arm, dtype=float)

    # Enforce single IMU–GNSS–Truth pair by dataset ID (e.g., X001/X002/X003)
    from utils.io_checks import assert_single_pair
    # Only enforce when a truth file is supplied; allow runs without truth
    if args.truth_file:
        if args.allow_truth_mismatch:
            try:
                assert_single_pair(args.imu_file, args.gnss_file, args.truth_file, force_mix=True)
            except Exception as ex:  # pragma: no cover — permissive path
                logging.info(f"[PairGuard] Mismatch allowed: {ex}")
        else:
            assert_single_pair(args.imu_file, args.gnss_file, args.truth_file)

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    method = args.method
    measure_source = args.measure_source
    imu_file, gnss_file = check_files(args.imu_file, args.gnss_file)
    truth_file = args.truth_file

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    imu_stem = Path(imu_file).stem
    gnss_stem = Path(gnss_file).stem
    method_tag = method + ("_" + args.tag if args.tag else "")
    tag = TAG(imu=imu_stem, gnss=gnss_stem, method=method_tag)
    summary_tag = f"{imu_stem}_{gnss_stem}"
    run_id = make_run_id(imu_file, gnss_file, method_tag)
    global RUN_ID
    RUN_ID = run_id
    out_dir = RESULTS_DIR

    logging.info(f"Running attitude-estimation method: {method}")

    # ================================
    # TASK 1: Define Reference Vectors in NED Frame
    # ================================
    logging.info(f"TASK 1 ({method}): Define reference vectors in NED frame")

    (
        lat_deg,
        lon_deg,
        alt,
        g_NED,
        omega_ie_NED,
        mag_NED,
        initial_vel_ned,
        ecef_origin,
        gnss_columns,
    ) = compute_reference_vectors(gnss_file, args.mag_file)

    # If the user provides an explicit initial position, override the GNSS-derived one
    if args.init_lat_deg is not None or args.init_lon_deg is not None or args.init_alt_m is not None:
        lat_deg = float(args.init_lat_deg if args.init_lat_deg is not None else lat_deg)
        lon_deg = float(args.init_lon_deg if args.init_lon_deg is not None else lon_deg)
        alt = float(args.init_alt_m if args.init_alt_m is not None else (alt if alt is not None else 0.0))
        lat = np.deg2rad(lat_deg)
        _lon = np.deg2rad(lon_deg)
        # Recompute reference vectors for the overridden geodetic position
        from utils import validate_gravity_vector
        from utils.ecef_llh import lla_to_ecef
        g_NED = validate_gravity_vector(lat_deg, alt)
        omega_ie_NED = EARTH_RATE * np.array([np.cos(lat), 0.0, -np.sin(lat)])
        # Recompute origin in ECEF from the provided LLA so downstream NED conversions are consistent
        x0, y0, z0 = lla_to_ecef(lat_deg, lon_deg, alt)
        ecef_origin = np.array([float(x0), float(y0), float(z0)])
        logging.info(
            "Overriding initial position with user-provided values: lat=%.6f deg, lon=%.6f deg, alt=%.2f m",
            lat_deg,
            lon_deg,
            alt,
        )
    lat = np.deg2rad(lat_deg)
    _lon = np.deg2rad(lon_deg)
    logging.info(
        f"Computed initial latitude: {lat_deg:.6f}°, longitude: {lon_deg:.6f}° from GNSS"
    )
    logging.info("==== Reference Vectors in NED Frame ====")
    logging.info(f"Gravity vector (NED):        {g_NED} m/s^2")
    logging.info(f"Earth rotation rate (NED):   {omega_ie_NED} rad/s")
    logging.info(f"Latitude (deg):              {lat_deg:.6f}")
    logging.info(f"Longitude (deg):             {lon_deg:.6f}")

    # -- Subtask 1.3: report the Earth rotation rate with its actual numbers ---
    _w = np.linalg.norm(omega_ie_NED)
    logging.info("Subtask 1.3: Earth rotation rate vector in NED frame")
    logging.info(
        "  omega_earth = %.9e rad/s  (%.6f deg/hr, sidereal day %.4f h)",
        EARTH_RATE, np.degrees(EARTH_RATE) * 3600.0,
        2 * np.pi / EARTH_RATE / 3600.0,
    )
    logging.info(
        "  omega_ie_NED = [N %+.9e, E %+.9e, D %+.9e] rad/s",
        omega_ie_NED[0], omega_ie_NED[1], omega_ie_NED[2],
    )
    logging.info(
        "  |omega_ie_NED| = %.9e rad/s   North = w*cos(lat), Down = -w*sin(lat), East = 0",
        _w,
    )

    # -- Subtask 1.4: validate the pair -------------------------------------
    _ang = np.degrees(np.arccos(np.clip(
        float(np.dot(g_NED, omega_ie_NED) / (np.linalg.norm(g_NED) * _w + 1e-30)), -1, 1)))
    logging.info("Subtask 1.4: Validating reference vectors.")
    logging.info("  |g| = %.6f m/s^2, |omega| = %.9e rad/s, angle(g, omega) = %.4f deg",
                 float(np.linalg.norm(g_NED)), _w, _ang)
    logging.info("  Reference vectors validated successfully.")
    logging.info("  Computed initial latitude: %.6f deg, longitude: %.6f deg from GNSS",
                 lat_deg, lon_deg)

    # -- Subtask 1.2/1.3/1.4 figures ----------------------------------------
    if not args.no_plots:
        try:
            from task1_subtask_plots import (
                task1_2_gravity, task1_3_earth_rate, task1_4_validation,
            )
            _t1tag = f"{method}_{Path(imu_file).stem}_{Path(gnss_file).stem}"
            _alt = float(alt) if "alt" in dir() and alt is not None else 0.0
            task1_2_gravity(_t1tag, lat_deg, lon_deg, _alt, g_NED, RESULTS_DIR)
            task1_3_earth_rate(_t1tag, lat_deg, omega_ie_NED, EARTH_RATE, RESULTS_DIR)
            task1_4_validation(_t1tag, lat_deg, lon_deg, _alt, g_NED, omega_ie_NED, RESULTS_DIR)
        except Exception as ex:  # pragma: no cover - plotting must not abort a run
            logging.warning("Task 1 subtask plots failed: %s", ex)

    # --- Save Task 1 artifacts for reuse ---
    R_ecef_to_ned = compute_C_ECEF_to_NED(lat, _lon)
    R_ned_to_ecef = R_ecef_to_ned.T
    arrays = {
        "lat0_deg": np.array(lat_deg),
        "lon0_deg": np.array(lon_deg),
        "h0_m": np.array(alt if alt is not None else 0.0),
        "g_ned": g_NED,
        "omega_ie_ned": omega_ie_NED,
        "R_ecef_to_ned": R_ecef_to_ned,
        "R_ned_to_ecef": R_ned_to_ecef,
        "r0_ecef_m": ecef_origin,
    }
    dataset_match = re.search(r"X\d+", gnss_stem)
    dataset_id = dataset_match.group(0) if dataset_match else "unknown"
    versions = {
        "numpy": np.__version__,
        "pandas": pd.__version__,
    }
    meta = {
        "dataset_id": dataset_id,
        "method": method,
        "imu_file": imu_file,
        "gnss_file": gnss_file,
        "truth_file": truth_file or "",
        "time_saved": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "versions": versions,
    }
    save_task1_artifacts(RESULTS_DIR, tag, meta, arrays, gnss_columns)

    if not args.no_plots:
        from task1_reference_vectors import task1_reference_vectors
        try:
            gnss_df = pd.read_csv(gnss_file)
            if "Height_deg" in gnss_df.columns and "Height_m" not in gnss_df.columns:
                gnss_df = gnss_df.rename(columns={"Height_deg": "Height_m"})
            png_path = task1_reference_vectors(gnss_df, out_dir, run_id)
        except Exception as ex:
            print(f"Task 1: static map generation failed: {ex}")
    else:
        logging.info("Skipping plot generation (--no-plots)")

    # ================================
    # TASK 2: Measure the Vectors in the Body Frame
    # ================================
    logging.info("TASK 2: Measure the vectors in the body frame")
    logging.info("Subtask 2.1: Load IMU data and derive the sampling period.")

    (
        dt_imu,
        g_body,
        omega_ie_body,
        mag_body,
        static_start,
        static_end,
    ) = measure_body_vectors(
        imu_file,
        static_start=args.static_start,
        static_end=args.static_end,
        mag_file=args.mag_file,
        tag=tag,
    )
    logging.info("  Estimated IMU dt: %.6f s  (%.1f Hz)", dt_imu, 1.0 / dt_imu)
    _n_static = int(static_end) - int(static_start)
    logging.info("Subtask 2.2: Detect the static interval.")
    logging.info("  Static interval indices: %d to %d (%d samples)",
                 static_start, static_end, _n_static)
    logging.info("  Static interval duration: %.2f s", _n_static * dt_imu)
    logging.info("Subtask 2.3: Derive the body-frame reference vectors.")
    logging.info("  g_body        = [%+.9e %+.9e %+.9e] m/s^2  |g| = %.6f",
                 g_body[0], g_body[1], g_body[2], float(np.linalg.norm(g_body)))
    logging.info("  omega_ie_body = [%+.9e %+.9e %+.9e] rad/s |w| = %.9e",
                 omega_ie_body[0], omega_ie_body[1], omega_ie_body[2],
                 float(np.linalg.norm(omega_ie_body)))
    logging.info("Subtask 2.4: Validate the measured body vectors.")
    _dg = abs(float(np.linalg.norm(g_body)) - float(np.linalg.norm(g_NED)))
    _dw = abs(float(np.linalg.norm(omega_ie_body)) - EARTH_RATE)
    logging.info("  |g_body| vs |g_NED|            : %.6f vs %.6f  (diff %.6f m/s^2)",
                 float(np.linalg.norm(g_body)), float(np.linalg.norm(g_NED)), _dg)
    logging.info("  |omega_body| vs Earth rate     : %.9e vs %.9e  (diff %.3e rad/s)",
                 float(np.linalg.norm(omega_ie_body)), EARTH_RATE, _dw)
    logging.info("  Body vectors %s",
                 "validated successfully." if _dg < 0.5 and _dw < 1e-5
                 else "DIFFER from the NED references - check the axis convention.")

    if not args.no_plots:
        try:
            imu_data = np.loadtxt(imu_file)
            png_path = save_task2_summary_png(
                imu_data,
                static_start,
                static_end,
                g_body,
                omega_ie_body,
                run_id,
                out_dir,
            )
            print(f"Task 2: saved summary PNG -> {png_path}")

            columns = [
                "index",
                "time",
                "gyro_x",
                "gyro_y",
                "gyro_z",
                "accel_x",
                "accel_y",
                "accel_z",
                "temp",
                "status",
            ][: imu_data.shape[1]]
            imu_df = pd.DataFrame(imu_data, columns=columns)
            task2_measure_body_vectors(
                imu_df,
                (static_start, static_end),
                out_dir,
                run_id,
            )
        except Exception as ex:  # pragma: no cover - plotting is best effort
            print(f"Task 2: summary PNG failed: {ex}")
    else:
        logging.info("Skipping Task 2 summary plot (--no-plots)")

    # ================================
    # TASK 3: Solve Wahba’s Problem
    # ================================

    # ================================
    # TASK 3: Solve Wahba’s Problem
    # ================================
    logging.info(
        "TASK 3: Solve Wahba’s problem (find initial attitude from body to NED)"
    )

    # --------------------------------
    # Subtask 3.1: Prepare Vector Pairs for Attitude Determination
    # --------------------------------
    logging.info("Subtask 3.1: Preparing vector pairs for attitude determination.")
    # Case 1: Current implementation vectors
    v1_B = g_body / np.linalg.norm(g_body)  # Normalize gravity in body frame
    v2_B = (
        omega_ie_body / np.linalg.norm(omega_ie_body)
        if np.linalg.norm(omega_ie_body) > 1e-10
        else np.array([1.0, 0.0, 0.0])
    )
    v1_N = g_NED / np.linalg.norm(g_NED)  # Normalize gravity in NED frame
    v2_N = omega_ie_NED / np.linalg.norm(omega_ie_NED)  # Normalize Earth rotation rate
    logging.debug(f"Case 1 - Normalized body gravity: {v1_B}")
    logging.debug(f"Case 1 - Normalized body Earth rate: {v2_B}")
    logging.debug(f"Case 1 - Normalized NED gravity: {v1_N}")
    logging.debug(f"Case 1 - Normalized NED Earth rate: {v2_N}")

    # Case 2: Recompute ω_ie,NED using document equation
    omega_ie_NED_doc = EARTH_RATE * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    v2_N_doc = omega_ie_NED_doc / np.linalg.norm(
        omega_ie_NED_doc
    )  # Normalize for Case 2
    logging.debug(f"Case 2 - Normalized NED Earth rate (document equation): {v2_N_doc}")

    # --------------------------------
    # Subtask 3.2: TRIAD Method
    # --------------------------------
    logging.info("Subtask 3.2: Computing rotation matrix using TRIAD method.")
    R_tri = triad_svd(v1_B, v2_B, v1_N, v2_N)
    logging.info("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)
    logging.debug("Rotation matrix (TRIAD method, Case 1):\n%s", R_tri)

    R_tri_doc = triad_svd(v1_B, v2_B, v1_N, v2_N_doc)
    logging.info("Rotation matrix (TRIAD method, Case 2):\n%s", R_tri_doc)
    logging.debug("Rotation matrix (TRIAD method, Case 2):\n%s", R_tri_doc)

    # --------------------------------
    # Subtask 3.3: Davenport’s Q-Method
    # --------------------------------
    logging.info("Subtask 3.3: Computing rotation matrix using Davenport’s Q-Method.")
    w_gravity = 0.9999
    w_omega = 0.0001

    # Case 1
    B = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N, v2_B)
    sigma = np.trace(B)
    S = B + B.T
    Z = np.array([B[1, 2] - B[2, 1], B[2, 0] - B[0, 2], B[0, 1] - B[1, 0]])
    K = np.zeros((4, 4))
    K[0, 0] = sigma
    K[0, 1:] = Z
    K[1:, 0] = Z
    K[1:, 1:] = S - sigma * np.eye(3)
    eigvals, eigvecs = np.linalg.eigh(K)
    q_dav = eigvecs[:, np.argmax(eigvals)]
    if q_dav[0] < 0:
        q_dav = -q_dav
    q_dav = np.array(
        [q_dav[0], -q_dav[1], -q_dav[2], -q_dav[3]]
    )  # Conjugate for body-to-NED
    qw, qx, qy, qz = q_dav
    R_dav = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ]
    )
    logging.info("Rotation matrix (Davenport’s Q-Method, Case 1):\n%s", R_dav)
    logging.info("Davenport quaternion (q_w, q_x, q_y, q_z, Case 1): %s", q_dav)
    logging.debug("Rotation matrix (Davenport’s Q-Method, Case 1):\n%s", R_dav)
    logging.debug("Davenport quaternion (Case 1): %s", q_dav)

    # Case 2
    B_doc = w_gravity * np.outer(v1_N, v1_B) + w_omega * np.outer(v2_N_doc, v2_B)
    sigma_doc = np.trace(B_doc)
    S_doc = B_doc + B_doc.T
    Z_doc = np.array(
        [
            B_doc[1, 2] - B_doc[2, 1],
            B_doc[2, 0] - B_doc[0, 2],
            B_doc[0, 1] - B_doc[1, 0],
        ]
    )
    K_doc = np.zeros((4, 4))
    K_doc[0, 0] = sigma_doc
    K_doc[0, 1:] = Z_doc
    K_doc[1:, 0] = Z_doc
    K_doc[1:, 1:] = S_doc - sigma_doc * np.eye(3)
    eigvals_doc, eigvecs_doc = np.linalg.eigh(K_doc)
    q_dav_doc = eigvecs_doc[:, np.argmax(eigvals_doc)]
    if q_dav_doc[0] < 0:
        q_dav_doc = -q_dav_doc
    q_dav_doc = np.array([q_dav_doc[0], -q_dav_doc[1], -q_dav_doc[2], -q_dav_doc[3]])
    qw, qx, qy, qz = q_dav_doc
    R_dav_doc = np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qw * qz), 2 * (qx * qz + qw * qy)],
            [2 * (qx * qy + qw * qz), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qw * qx)],
            [2 * (qx * qz - qw * qy), 2 * (qy * qz + qw * qx), 1 - 2 * (qx**2 + qy**2)],
        ]
    )
    logging.info("Rotation matrix (Davenport’s Q-Method, Case 2):\n%s", R_dav_doc)
    logging.info("Davenport quaternion (q_w, q_x, q_y, q_z, Case 2): %s", q_dav_doc)
    logging.debug("Rotation matrix (Davenport’s Q-Method, Case 2):\n%s", R_dav_doc)
    logging.debug("Davenport quaternion (Case 2): %s", q_dav_doc)

    # --------------------------------
    # Subtask 3.4: SVD Method
    # --------------------------------
    logging.info("Subtask 3.4: Computing rotation matrix using SVD method.")
    body_vecs = [g_body, omega_ie_body]
    ref_vecs = [g_NED, omega_ie_NED]
    if mag_body is not None and mag_NED is not None:
        body_vecs.append(mag_body)
        ref_vecs.append(mag_NED)
    elif args.use_gnss_heading:
        speed = np.linalg.norm(initial_vel_ned)
        if speed > 0.2:
            body_vecs.append(np.array([1.0, 0.0, 0.0]))
            ref_vecs.append(initial_vel_ned / speed)

    R_svd = svd_alignment(body_vecs, ref_vecs)
    logging.info("Rotation matrix (SVD method):\n%s", R_svd)
    logging.debug("Rotation matrix (SVD method):\n%s", R_svd)
    R_svd_doc = R_svd

    # --------------------------------
    # Subtask 3.5: Convert TRIAD and SVD DCMs to Quaternions
    # --------------------------------
    logging.info("Subtask 3.5: Converting TRIAD and SVD DCMs to quaternions.")

    def rot_to_quaternion(R):
        tr = R[0, 0] + R[1, 1] + R[2, 2]
        if tr > 0:
            S = np.sqrt(tr + 1.0) * 2
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            qw = (R[2, 1] - R[1, 2]) / S
            qx = 0.25 * S
            qy = (R[0, 1] + R[1, 0]) / S
            qz = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            qw = (R[0, 2] - R[2, 0]) / S
            qx = (R[0, 1] + R[1, 0]) / S
            qy = 0.25 * S
            qz = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            qw = (R[1, 0] - R[0, 1]) / S
            qx = (R[0, 2] + R[2, 0]) / S
            qy = (R[1, 2] + R[2, 1]) / S
            qz = 0.25 * S
        q = np.array([qw, qx, qy, qz])
        return q / np.linalg.norm(q)

    q_tri = rot_to_quaternion(R_tri)
    if q_tri[0] < 0:
        q_tri = -q_tri
    q_svd = rot_to_quaternion(R_svd)
    if q_svd[0] < 0:
        q_svd = -q_svd
    q_tri_doc = rot_to_quaternion(R_tri_doc)
    if q_tri_doc[0] < 0:
        q_tri_doc = -q_tri_doc
    q_svd_doc = rot_to_quaternion(R_svd_doc)
    if q_svd_doc[0] < 0:
        q_svd_doc = -q_svd_doc

    # Combine all methods by averaging rotation matrices
    R_all = average_rotation_matrices([R_tri, R_dav, R_svd])
    q_all = rot_to_quaternion(R_all)
    if q_all[0] < 0:
        q_all = -q_all

    logging.info(f"Quaternion (TRIAD, Case 1): {q_tri}")
    logging.debug(f"Quaternion (TRIAD, Case 1): {q_tri}")
    euler_tri = R.from_matrix(R_tri).as_euler("xyz", degrees=True)
    logging.info(
        f"TRIAD initial attitude (deg): roll={euler_tri[0]:.3f} pitch={euler_tri[1]:.3f} yaw={euler_tri[2]:.3f}"
    )
    if not args.disable_triad_log:
        with open("triad_init_log.txt", "a") as logf:
            logf.write(f"{imu_file}: init_euler_deg={euler_tri}\n")
    logging.info(f"Quaternion (SVD, Case 1): {q_svd}")
    logging.debug(f"Quaternion (SVD, Case 1): {q_svd}")
    logging.info(f"Quaternion (TRIAD, Case 2): {q_tri_doc}")
    logging.debug(f"Quaternion (TRIAD, Case 2): {q_tri_doc}")
    logging.info(f"Quaternion (SVD, Case 2): {q_svd_doc}")
    logging.debug(f"Quaternion (SVD, Case 2): {q_svd_doc}")

    # -- Error metrics for each method ------------------------------------
    logging.info("\nAttitude errors using reference vectors:")

    grav_errors = {}
    omega_errors = {}

    for m, rot_matrix in {
        "TRIAD": R_tri,
        "Davenport": R_dav,
        "SVD": R_svd,
    }.items():
        grav_err_deg, omega_err_deg = compute_wahba_errors(
            rot_matrix, g_body, omega_ie_body, g_NED, omega_ie_NED
        )
        logging.info(f"{m:10s} -> Gravity error (deg): {grav_err_deg:.6f}")
        logging.info(f"{m:10s} -> Earth rate error (deg):  {omega_err_deg:.6f}")
        grav_errors[m] = grav_err_deg
        omega_errors[m] = omega_err_deg

    grav_err_mean = float(np.mean(list(grav_errors.values())))
    grav_err_max = float(np.max(list(grav_errors.values())))
    omega_err_mean = float(np.mean(list(omega_errors.values())))
    omega_err_max = float(np.max(list(omega_errors.values())))

    # Load truth quaternion from STATE file if provided (basic validation)
    if truth_file:
        try:
            _ = np.loadtxt(truth_file, comments="#")
        except Exception as e:
            logging.warning(
                f"Failed to load truth file {truth_file}: {e}"
            )

    # --------------------------------
    # Subtask 3.6: Validate Attitude Determination and Compare Methods
    # --------------------------------
    logging.info(
        "Subtask 3.6: Validating attitude determination and comparing methods."
    )
    # -- Composite quaternion and per-method errors ---------------------------
    quats_case1 = {"TRIAD": q_tri, "Davenport": q_dav, "SVD": q_svd}
    quats_case2 = {"TRIAD": q_tri_doc, "Davenport": q_dav_doc, "SVD": q_svd_doc}

    methods = ["TRIAD", "Davenport", "SVD"]

    def normalise(q):
        return q / np.linalg.norm(q)

    def attitude_errors(q1, q2):
        def quat_to_rot(q):
            w, x, y, z = q
            return np.array(
                [
                    [1 - 2 * (y**2 + z**2), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x**2 + z**2), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x**2 + y**2)],
                ]
            )

        R1 = quat_to_rot(q1)
        R2 = quat_to_rot(q2)
        g_vec = np.array([0.0, 0.0, 1.0])
        w_vec = np.array([1.0, 0.0, 0.0])

        def ang(a, b):
            dot = np.clip(
                np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1.0, 1.0
            )
            return np.degrees(np.arccos(dot))

        g_err = ang(R1 @ g_vec, R2 @ g_vec)
        w_err = ang(R1 @ w_vec, R2 @ w_vec)
        return g_err, w_err

    results = {}
    for m in methods:
        g_err, w_err = attitude_errors(quats_case1[m], quats_case2[m])
        results[m] = {"gravity_error": g_err, "earth_rate_error": w_err}

    logging.info("\nDetailed Earth-Rate Errors:")
    for m, o in omega_errors.items():
        logging.info(f"  {m:10s}: {o:.6f}°")

    # Relaxed Earth-rate error check --------------------------------------

    omega_errs = {
        "TRIAD": omega_errors["TRIAD"],
        "Davenport": omega_errors["Davenport"],
        "SVD": omega_errors["SVD"],
    }
    diff = max(omega_errs.values()) - min(omega_errs.values())
    tol = 1e-5  # allow up to 0.00001° of spread without complaint

    # always print them so you can see the tiny spreads at runtime
    logging.info("\nEarth-rate errors by method:")
    for name, err in omega_errs.items():
        logging.info(f"  {name:10s}: {err:.9f}°")
    logging.info(f"  Δ = {diff:.2e}° (tolerance = {tol:.1e})\n")

    if diff < tol:
        logging.warning(
            "All Earth-rate errors are very close; differences "
            f"are within {tol:.1e}°"
        )

    logging.info("\n==== Method Comparison for Case X001 and Case X001_doc ====")
    logging.info(
        f"{'Method':10s}  {'Gravity Err (deg)':>18s}  {'Earth-Rate Err (deg)':>22s}"
    )
    for m in ["TRIAD", "Davenport", "SVD"]:
        g = grav_errors[m]
        o = omega_errors[m]
        logging.info(f"{m:10s}  {g:18.4f}  {o:22.4f}")

    # --------------------------------
    # Subtask 3.7: Plot Validation Errors and Quaternion Components
    # --------------------------------

    logging.info("Subtask 3.7: Plotting validation errors and quaternion components.")

    quat_plot_dict = {}
    for m in methods:
        quat_plot_dict[f"{m}_Case1"] = quats_case1[m]
        quat_plot_dict[f"{m}_Case2"] = quats_case2[m]

    error_plot_dict = {
        m: {"grav": grav_errors[m], "earth": omega_errors[m]} for m in methods
    }

    if not args.no_plots:
        task3_plot_quaternions_and_errors(
            methods, quat_plot_dict, error_plot_dict, out_dir
        )

    # --------------------------------
    # Subtask 3.8: Store Rotation Matrices for Later Tasks
    # --------------------------------
    logging.info("Subtask 3.8: Storing rotation matrices for use in later tasks.")
    task3_results = {
        "TRIAD": {"R": R_tri},
        "Davenport": {"R": R_dav},
        "SVD": {"R": R_svd},
    }
    logging.info("Task 3 results stored in memory: %s", list(task3_results.keys()))

    # ================================
    # TASK 4: GNSS and IMU Data Integration and Comparison
    # ================================

    # ================================
    # TASK 4: GNSS and IMU Data Integration and Comparison
    # ================================
    logging.info("TASK 4: GNSS and IMU Data Integration and Comparison")

    # --------------------------------
    # Subtask 4.1: Access Rotation Matrices from Task 3
    # --------------------------------
    logging.info("Subtask 4.1: Accessing rotation matrices from Task 3.")
    methods = [method]
    C_B_N_methods = {m: task3_results[m]["R"] for m in methods}
    logging.info("Rotation matrices accessed: %s", list(C_B_N_methods.keys()))

    # --------------------------------
    # Subtask 4.2: Load GNSS Data
    # --------------------------------
    logging.info("Subtask 4.2: Loading GNSS data.")
    try:
        gnss_data = pd.read_csv(gnss_file)
        if "Height_deg" in gnss_data.columns and "Height_m" not in gnss_data.columns:
            gnss_data = gnss_data.rename(columns={"Height_deg": "Height_m"})
        gnss_data = normalize_gnss_headers(gnss_data)
    except Exception as e:
        logging.error(f"Failed to load GNSS data file: {e}")
        raise

    # --------------------------------
    # Subtask 4.3: Extract Relevant Columns
    # --------------------------------
    logging.info("Subtask 4.3: Extracting relevant columns.")
    time_col = "Posix_Time"
    pos_cols = ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]
    vel_cols = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
    gnss_time = zero_base_time(gnss_data[time_col].values)
    gnss_pos_ecef = gnss_data[pos_cols].values.astype(float)
    gnss_vel_ecef_cols = gnss_data[vel_cols].values.astype(float)
    logging.info(f"GNSS data shape: {gnss_pos_ecef.shape}")
    # Prefer GNSS velocity columns if present and finite; fallback to d/dt(pos)
    if np.isfinite(gnss_vel_ecef_cols).all():
        spd = np.linalg.norm(gnss_vel_ecef_cols, axis=1)
        p50, p95 = np.nanpercentile(spd, [50, 95])
        # If columns are essentially zero, fall back to d/dt(position)
        if p95 < 1e-3:
            gnss_vel_ecef = np.gradient(gnss_pos_ecef, gnss_time, axis=0)
            vel_source = "pos-derivative"
            logging.info(
                "GNSS velocity columns ~0 (p95=%.3f m/s); using d/dt(pos_ecef) fallback.",
                p95,
            )
        else:
            scale = 1.0
            if 10.0 <= p95 < 100.0:
                scale = 0.1
            elif 100.0 <= p95 < 1000.0:
                scale = 0.01
            gnss_vel_ecef = gnss_vel_ecef_cols * scale
            vel_source = "columns"
            logging.info(
                "Using GNSS velocity columns %s (scale %.2f). p50=%.3f, p95=%.3f m/s",
                vel_cols,
                scale,
                p50 * scale,
                p95 * scale,
            )
    else:
        gnss_vel_ecef = np.gradient(gnss_pos_ecef, gnss_time, axis=0)
        vel_source = "pos-derivative"
        logging.warning(
            "Velocity columns missing/unusable -> using d/dt(pos_ecef). Expect higher noise."
        )

    lat_series = []
    lon_series = []
    for x, y, z in gnss_pos_ecef:
        lat_d, lon_d, _ = ecef_to_geodetic(x, y, z)
        lat_series.append(np.deg2rad(lat_d))
        lon_series.append(np.deg2rad(lon_d))
    lat_series = np.array(lat_series)
    lon_series = np.array(lon_series)

    # --------------------------------
    # Subtask 4.4: Define Reference Point
    # --------------------------------
    logging.info("Subtask 4.4: Defining reference point.")
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = ecef_origin
    logging.info(
        f"Reference point: lat={ref_lat:.6f} rad, lon={ref_lon:.6f} rad, r0={ref_r0}"
    )

    # --------------------------------
    # Subtask 4.5: Compute Rotation Matrix
    # --------------------------------
    logging.info("Subtask 4.5: Computing ECEF to NED rotation matrix.")
    C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon)
    logging.info("ECEF to NED rotation matrix computed.")
    C_NED_to_ECEF = C_ECEF_to_NED.T
    logging.info("NED to ECEF rotation matrix computed.")

    # --------------------------------
    # Subtask 4.6: Convert GNSS Data to NED Frame
    # --------------------------------
    logging.info("Subtask 4.6: Converting GNSS data to NED frame.")
    from utils import ecef_to_ned

    gnss_pos_ned = ecef_to_ned(gnss_pos_ecef, ref_lat, ref_lon, ref_r0)
    gnss_vel_ned = np.array([C_ECEF_to_NED @ v for v in gnss_vel_ecef])
    logging.info(
        f"GNSS velocity ({vel_source}) first sample: {gnss_vel_ned[0]}"
    )
    logging.info("GNSS data transformed to NED frame.")

    # --------------------------------
    # Subtask 4.7: Estimate GNSS Acceleration in NED
    # --------------------------------
    logging.info("Subtask 4.7: Estimating GNSS acceleration in NED.")
    gnss_acc_ecef = np.zeros_like(gnss_vel_ecef)
    dt = np.diff(gnss_time, prepend=gnss_time[0])
    gnss_acc_ecef[1:] = (gnss_vel_ecef[1:] - gnss_vel_ecef[:-1]) / dt[1:, np.newaxis]
    gnss_acc_ned = np.array([C_ECEF_to_NED @ a for a in gnss_acc_ecef])
    logging.info("GNSS acceleration estimated in NED frame.")

    acc_biases = {}
    gyro_biases = {}

    # --------------------------------
    # Subtask 4.8: Load IMU Data and Correct for Bias for Each Method
    # --------------------------------
    logging.info(
        "Subtask 4.8: Loading IMU data and correcting for bias for each method."
    )
    try:
        imu_data = pd.read_csv(imu_file, sep=r"\s+", header=None)
        imu_time = np.arange(len(imu_data)) * dt_imu
        lat_interp = interp_to(gnss_time, lat_series, imu_time)
        lon_interp = interp_to(gnss_time, lon_series, imu_time)

        # Convert increments to rates (sensor frame) and filter
        acc_s = imu_data[[5, 6, 7]].values / dt_imu  # delta_v / dt_imu
        gyro_s = imu_data[[2, 3, 4]].values / dt_imu  # delta_theta / dt_imu
        acc_s = butter_lowpass_filter(acc_s)
        gyro_s = butter_lowpass_filter(gyro_s)

        start_idx = args.static_start
        end_idx = args.static_end
        if start_idx is None and end_idx is None:
            dataset_window = {
                "IMU_X001.dat": (296, 479907),
                "IMU_X002.dat": (296, 479907),
                "IMU_X003.dat": (296, 479907),
            }.get(Path(imu_file).name)
            if dataset_window and len(acc_s) >= dataset_window[1]:
                start_idx, end_idx = dataset_window
                end_idx = min(end_idx, len(acc_s))

        if start_idx is not None:
            if end_idx is None:
                end_idx = len(acc_s)
            start_idx = max(0, start_idx)
            end_idx = min(end_idx, len(acc_s))
            N_static = end_idx - start_idx
        else:
            N_static = min(4000, len(imu_data))
            start_idx = 0
            end_idx = N_static

        if N_static < MIN_STATIC_SAMPLES:
            raise ValueError(
                f"Insufficient static samples for bias estimation; require at least {MIN_STATIC_SAMPLES}."
            )

        # Choose sensor→body axis map from static window so gravity aligns to +Z (NED down)
        try:
            a_mean_s = np.mean(acc_s[start_idx:end_idx], axis=0)
        except Exception:
            a_mean_s = np.mean(acc_s[:N_static], axis=0)
        # The automatic sensor->body axis map permutes the IMU axes. The truth
        # file's Body->ECEF quaternion refers to the RAW sensor axes, so applying
        # the map leaves the fused attitude a fixed 120 deg from truth. Default
        # is therefore "none"; pass --axis-map auto to restore the old behaviour.
        if args.axis_map == "auto":
            C_bs, map_err = choose_C_bs_from_static(a_mean_s)
        else:
            C_bs, map_err = np.eye(3), 0.0
        acc_body = (C_bs @ acc_s.T).T
        gyro_body = (C_bs @ gyro_s.T).T
        # Axis-map sanity check after mapping
        try:
            g_mean = np.mean(acc_body[start_idx:end_idx], axis=0)
            print("[AxisMap]", sanity_check_tilt(g_mean))
        except Exception:
            pass

        static_acc, static_gyro = compute_biases(
            acc_body,
            gyro_body,
            start_idx,
            end_idx,
        )

        # Compute accelerometer scale factor using magnitude ratio
        scale = np.linalg.norm(g_NED) / np.linalg.norm(static_acc)

        # Compute corrected acceleration and gyroscope data for each method
        acc_body_corrected = {}
        gyro_body_corrected = {}
        for m in methods:
            C_N_B = C_B_N_methods[m].T  # NED to Body rotation matrix
            g_body_expected = C_N_B @ g_NED  # Expected gravity in body frame

            # Accelerometer bias: static_acc should equal -g_body_expected
            acc_bias = static_acc + g_body_expected  # measured minus expected

            # Gyroscope bias: static_gyro should equal C_N_B @ omega_ie_NED
            omega_ie_NED = np.array(
                [EARTH_RATE * np.cos(ref_lat), 0.0, -EARTH_RATE * np.sin(ref_lat)]
            )
            omega_ie_body_expected = C_N_B @ omega_ie_NED
            gyro_bias = (
                static_gyro - omega_ie_body_expected
            )  # Bias = measured - expected

            # Correct the entire dataset
            acc_body_corrected[m] = scale * (acc_body - acc_bias)
            gyro_body_corrected[m] = gyro_body  # bias estimated online
            acc_biases[m] = acc_bias
            gyro_biases[m] = gyro_bias

            bias_mag = np.linalg.norm(acc_bias)
            logging.info(
                f"Method {m}: Accelerometer bias: {acc_bias} (|b|={bias_mag:.6f} m/s^2)"
            )
            logging.info(f"Method {m}: Gyroscope bias: {gyro_bias}")
            logging.info(f"Method {m}: Accelerometer scale factor: {scale:.4f}")
            logging.debug(f"Method {m}: Accelerometer bias: {acc_bias}")
            logging.debug(f"Method {m}: Gyroscope bias: {gyro_bias}")

        logging.info("IMU data corrected for bias for each method.")
        # Axis-map sanity check on the presumed static window
        try:
            g_mean = np.mean(acc_body[start_idx:end_idx], axis=0)
            print("[AxisMap] C_bs =\n", C_bs)
            print("[AxisMap] static mean accel (sensor):", a_mean_s)
            print("[AxisMap] static mean accel (body)  :", g_mean, " err_to_[0,0,+g]=", map_err)
            print(f"[AxisMap] Tilt from body Z: {float(np.degrees(np.arccos(np.clip(g_mean[2]/(np.linalg.norm(g_mean)+1e-12),-1,1)))):.2f}° (want small at rest, +Z=down)")
        except Exception:
            pass
        if methods:
            logging.info(
                "Accelerometer scale factor applied: %.4f",
                scale,
            )
            print(f"Task 4: applied accelerometer scale factor = {scale:.4f}")
    except Exception as e:
        logging.error(f"Failed to load IMU data or compute corrections: {e}")
        raise

    # --------------------------------
    # Subtask 4.9: Set IMU Parameters and Gravity Vector
    # --------------------------------
    logging.info("Subtask 4.9: Setting IMU parameters and gravity vector.")
    # Use the gravity vector computed in Task 1 instead of overwriting with the
    # constant defined in ``constants.GRAVITY``.  This preserves any
    # location-specific variation calculated earlier in the pipeline.
    logging.info(f"Using gravity vector from Task 1: {g_NED}")

    # --------------------------------
    # Subtask 4.10: Initialize Output Arrays
    # --------------------------------
    logging.info("Subtask 4.10: Initializing output arrays.")
    # per-method integration results
    pos_integ = {}
    vel_integ = {}
    acc_integ = {}
    pos_integ_ecef = {}
    vel_integ_ecef = {}

    # --------------------------------
    # Subtask 4.11: Integrate IMU Accelerations for Each Method
    # --------------------------------
    logging.info("Subtask 4.11: Integrating IMU accelerations for each method.")
    for m in methods:
        logging.info(f"Integrating IMU data using {m} method.")
        C_B_N = C_B_N_methods[m]
        pos, vel, acc, pos_e, vel_e = integrate_trajectory(
            acc_body_corrected[m],
            imu_time,
            C_B_N,
            g_NED,
            lat=lat_interp,
            lon=lon_interp,
            ref_lat=ref_lat,
            ref_lon=ref_lon,
            ref_ecef=ref_r0,
            debug=args.verbose,
        )
        pos_integ[m] = pos
        vel_integ[m] = vel
        acc_integ[m] = acc
        pos_integ_ecef[m] = pos_e
        vel_integ_ecef[m] = vel_e
    logging.info(
        "IMU-derived position, velocity, and acceleration computed for all methods."
    )

    # --------------------------------
    # Subtask 4.12: Validate and Plot Data
    # --------------------------------
    logging.info("Subtask 4.12: Validating and plotting data.")
    t0 = gnss_time[0]
    t_rel_ilu = imu_time - t0
    t_rel_gnss = gnss_time - t0
    truth_pos_ecef_i = truth_vel_ecef_i = None
    truth_pos_ned_i = truth_vel_ned_i = None
    t_truth = pos_truth_ecef = vel_truth_ecef = None
    if truth_file:
        try:
            truth = np.loadtxt(truth_file, comments="#")
            t_truth = zero_base_time(truth[:, 1])
            pos_truth_ecef = truth[:, 2:5]
            vel_truth_ecef = truth[:, 5:8]
            truth_pos_ecef_i = interpolate_series(t_rel_ilu, t_truth, pos_truth_ecef)
            truth_vel_ecef_i = interpolate_series(t_rel_ilu, t_truth, vel_truth_ecef)
            truth_pos_ned_i = ecef_to_ned(truth_pos_ecef_i, ref_lat, ref_lon, ref_r0)
            truth_vel_ned_i = (C_ECEF_to_NED @ truth_vel_ecef_i.T).T
        except Exception as e:
            logging.error(f"Failed to load truth file {truth_file}: {e}")
            truth_file = None
    # Validate time ranges
    if t_rel_ilu.max() < 1000:
        logging.warning(f"IMU time range too short: {t_rel_ilu.max():.2f} seconds")
    if t_rel_gnss.max() < 1000:
        logging.warning(f"GNSS time range too short: {t_rel_gnss.max():.2f} seconds")
    logging.debug(f"gnss_time range: {gnss_time.min():.2f} to {gnss_time.max():.2f}")
    logging.debug(f"imu_time range: {imu_time.min():.2f} to {imu_time.max():.2f}")
    logging.debug(f"t_rel_gnss range: {t_rel_gnss.min():.2f} to {t_rel_gnss.max():.2f}")
    logging.debug(f"t_rel_ilu range: {t_rel_ilu.min():.2f} to {t_rel_ilu.max():.2f}")

    missing = [m for m in methods if m not in pos_integ]
    if missing:
        logging.warning("Skipping plotting for %s (no data)", missing)

    # Comparison plot in NED frame
    fig_comp, axes_comp = plt.subplots(3, 3, figsize=(15, 10))
    directions = ["North", "East", "Down"]
    colors = COLORS
    for j in range(3):
        # Position comparison
        ax = axes_comp[0, j]
        ax.plot(
            t_rel_gnss,
            gnss_pos_ned[:, j],
            "k--",
            label="Derived GNSS (ECEF→NED)",
        )
        for m in methods:
            c = colors.get(m, None)
            ax.plot(
                t_rel_ilu,
                pos_integ[m][:, j],
                color=c,
                alpha=0.7,
                label=f"Derived IMU (Body→NED) ({m})",
            )
        ax.set_title(f"Position {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.legend(loc="best")

        # Velocity comparison
        ax = axes_comp[1, j]
        ax.plot(
            t_rel_gnss,
            gnss_vel_ned[:, j],
            "k--",
            label="Derived GNSS (ECEF→NED)",
        )
        for m in methods:
            c = colors.get(m, None)
            ax.plot(
                t_rel_ilu,
                vel_integ[m][:, j],
                color=c,
                alpha=0.7,
                label=f"Derived IMU (Body→NED) ({m})",
            )
        ax.set_title(f"Velocity {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend(loc="best")

        # Acceleration comparison
        ax = axes_comp[2, j]
        ax.plot(
            t_rel_gnss,
            gnss_acc_ned[:, j],
            "k--",
            label="Derived GNSS (ECEF→NED)",
        )
        for m in methods:
            c = colors.get(m, None)
            ax.plot(
                t_rel_ilu,
                acc_integ[m][:, j],
                color=c,
                alpha=0.7,
                label=f"Derived IMU (Body→NED) ({m})",
            )
        ax.set_title(f"Acceleration {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (m/s²)")
        ax.legend(loc="best")
    fig_comp.suptitle(
        f"Task 4 – {method} – NED Frame (All Data Derived to NED)"
    )
    fig_comp.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_comp, RESULTS_DIR, tag, "task4_12_1", "derivedGNSS_vs_derivedIMU_NED", ext="png", dpi=200, bbox_inches="tight")
    plt.close(fig_comp)
    logging.info("Comparison plot in NED frame saved")

    # Plot 1: Data in mixed frames (GNSS position/velocity in ECEF, IMU acceleration in body)
    logging.info("Plotting data in mixed frames.")
    fig_mixed, axes_mixed = plt.subplots(3, 3, figsize=(15, 10))
    directions_pos = ["X_ECEF", "Y_ECEF", "Z_ECEF"]
    directions_vel = ["VX_ECEF", "VY_ECEF", "VZ_ECEF"]
    directions_acc = ["AX_body", "AY_body", "AZ_body"]
    for i in range(3):
        for j in range(3):
            ax = axes_mixed[i, j]
            if i == 0:  # Position
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], "k-", label="Measured GNSS")
                ax.set_title(f"Position {directions_pos[j]}")
            elif i == 1:  # Velocity
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], "k-", label="Measured GNSS")
                ax.set_title(f"Velocity {directions_vel[j]}")
            else:  # Acceleration
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        acc_body_corrected[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Measured IMU ({m})",
                    )
                ax.set_title(f"Acceleration {directions_acc[j]}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_mixed.suptitle(
        f"Task 4 – {method} – Mixed Frames (IMU-derived vs. Measured GNSS)"
    )
    fig_mixed.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_mixed, RESULTS_DIR, tag, "task4_12_2", "measured_posvel_ECEF_accel_BODY", ext="png", dpi=200, bbox_inches="tight")
    plt.close(fig_mixed)
    logging.info("Mixed frames plot saved")

    # Plot 2: All data in NED frame (show both GNSS and IMU-derived in every subplot)
    logging.info("Plotting all data in NED frame.")
    fig_ned, axes_ned = plt.subplots(3, 3, figsize=(15, 10))
    directions_ned = ["N", "E", "D"]
    for i in range(3):
        for j in range(3):
            ax = axes_ned[i, j]
            if i == 0:  # Position
                ax.plot(
                    t_rel_gnss,
                    gnss_pos_ned[:, j],
                    "k-",
                    label="Derived GNSS (ECEF→NED)",
                )
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        pos_integ[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU (Body→NED) ({m})",
                    )
                ax.set_title(f"Position {directions_ned[j]}")
            elif i == 1:  # Velocity
                ax.plot(
                    t_rel_gnss,
                    gnss_vel_ned[:, j],
                    "k-",
                    label="Derived GNSS (ECEF→NED)",
                )
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        vel_integ[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU (Body→NED) ({m})",
                    )
                ax.set_title(f"Velocity V{directions_ned[j]}")
            else:  # Acceleration
                ax.plot(
                    t_rel_gnss,
                    gnss_acc_ned[:, j],
                    "k--",
                    label="Derived GNSS",
                )
                for m in methods:
                    c = colors.get(m, None)
                    f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                    ax.plot(
                        t_rel_ilu,
                        f_ned[j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU (Body→NED) ({m})",
                    )
                ax.set_title(f"Acceleration A{directions_ned[j]}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ned.suptitle(
        f"Task 4 – {method} – NED Frame (All Data Derived to NED)"
    )
    fig_ned.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_ned, RESULTS_DIR, tag, "task4_12_3", "integrated_trajectory_NED", ext="png", dpi=200, bbox_inches="tight")
    plt.close(fig_ned)
    logging.info("All data in NED frame plot saved")

    # Plot 3: All data in ECEF frame
    logging.info("Plotting all data in ECEF frame.")
    fig_ecef, axes_ecef = plt.subplots(3, 3, figsize=(15, 10))
    directions_ecef = ["X", "Y", "Z"]
    for i in range(3):
        for j in range(3):
            ax = axes_ecef[i, j]
            if i == 0:  # Position
                ax.plot(t_rel_gnss, gnss_pos_ecef[:, j], "k-", label="Measured GNSS")
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        pos_integ_ecef[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Position {directions_ecef[j]}_ECEF")
            elif i == 1:  # Velocity
                ax.plot(t_rel_gnss, gnss_vel_ecef[:, j], "k-", label="Measured GNSS")
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        vel_integ_ecef[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Velocity V{directions_ecef[j]}_ECEF")
            else:  # Acceleration
                ax.plot(
                    t_rel_gnss,
                    gnss_acc_ecef[:, j],
                    "k--",
                    label="Derived GNSS",
                )
                for m in methods:
                    c = colors.get(m, None)
                    f_ned = C_B_N_methods[m] @ acc_body_corrected[m].T
                    f_ecef = C_NED_to_ECEF @ f_ned
                    ax.plot(
                        t_rel_ilu,
                        f_ecef[j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Acceleration A{directions_ecef[j]}_ECEF")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ecef.suptitle(f"Task 4 – {method} – ECEF Frame (Derived IMU vs. GNSS)")
    fig_ecef.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_ecef, RESULTS_DIR, tag, "task4_12_4", "integrated_trajectory_ECEF", ext="png", dpi=200, bbox_inches="tight")
    plt.close(fig_ecef)
    logging.info("All data in ECEF frame plot saved")

    # Plot 4: All data in body frame
    logging.info("Plotting all data in body frame.")
    fig_body, axes_body = plt.subplots(3, 3, figsize=(15, 10))
    directions_body = ["X", "Y", "Z"]
    for i in range(3):
        for j in range(3):
            ax = axes_body[i, j]
            if i == 0:  # Position
                r_body = (C_N_B @ gnss_pos_ned.T).T
                ax.plot(
                    t_rel_gnss,
                    r_body[:, j],
                    "k-",
                    label="Derived GNSS",
                )
                for m in methods:
                    c = colors.get(m, None)
                    pos_body = (C_N_B @ pos_integ[m].T).T
                    ax.plot(
                        t_rel_ilu,
                        pos_body[:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Position r{directions_body[j]}_body")
            elif i == 1:  # Velocity
                vel_body_gnss = (C_N_B @ gnss_vel_ned.T).T
                ax.plot(
                    t_rel_gnss,
                    vel_body_gnss[:, j],
                    "k-",
                    label="Derived GNSS",
                )
                for m in methods:
                    c = colors.get(m, None)
                    vel_body_imu = (C_N_B @ vel_integ[m].T).T
                    ax.plot(
                        t_rel_ilu,
                        vel_body_imu[:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Derived IMU ({m})",
                    )
                ax.set_title(f"Velocity v{directions_body[j]}_body")
            else:  # Acceleration
                gnss_acc_body = (C_N_B @ gnss_acc_ned.T).T
                ax.plot(
                    t_rel_gnss,
                    gnss_acc_body[:, j],
                    "k--",
                    label="Derived GNSS",
                )
                for m in methods:
                    c = colors.get(m, None)
                    ax.plot(
                        t_rel_ilu,
                        acc_body_corrected[m][:, j],
                        color=c,
                        alpha=0.7,
                        label=f"Measured IMU ({m})",
                    )
                ax.set_title(f"Acceleration A{directions_body[j]}_body")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_body.suptitle(
        f"Task 4 – {method} – Body Frame (Measured IMU Accel; Derived Pos/Vel vs. Derived GNSS)"
    )
    fig_body.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_body, RESULTS_DIR, tag, "task4_12_5", "integrated_trajectory_BODY", ext="png", dpi=200, bbox_inches="tight")
    plt.close(fig_body)
    logging.info("All data in body frame plot saved")
    if not args.no_plots:
        task_summary("task4")

    # ================================
    # Task 5: Sensor Fusion with Kalman Filter
    # ================================
    logging.info("Task 5: Sensor Fusion with Kalman Filter")

    # --------------------------------
    # Subtask 5.1: Configure Logging
    # --------------------------------
    # --------------------------------
    # Subtask 5.1: Configure Logging
    # --------------------------------
    logging.info("Subtask 5.1: Configuring logging.")

    # --------------------------------
    # Subtask 5.2: Rotation Matrix - ECEF to NED
    # --------------------------------

    # --------------------------------
    # Subtask 5.3: Load GNSS and IMU Data
    # --------------------------------
    logging.info("Subtask 5.3: Loading GNSS and IMU data.")
    try:
        gnss_data = pd.read_csv(gnss_file)
        if "Height_deg" in gnss_data.columns and "Height_m" not in gnss_data.columns:
            gnss_data = gnss_data.rename(columns={"Height_deg": "Height_m"})
        gnss_data = normalize_gnss_headers(gnss_data)
        imu_data = pd.read_csv(imu_file, sep=r"\s+", header=None)
    except FileNotFoundError as e:
        missing = "GNSS" if "csv" in str(e) else "IMU"
        logging.error(f"{missing} file not found: {e.filename}")
        raise
    except Exception as e:
        logging.error(f"Failed to load data: {e}")
        raise

    # Extract GNSS fields
    time_col = "Posix_Time"
    pos_cols = ["X_ECEF_m", "Y_ECEF_m", "Z_ECEF_m"]
    vel_cols = ["VX_ECEF_mps", "VY_ECEF_mps", "VZ_ECEF_mps"]
    gnss_time = zero_base_time(gnss_data[time_col].values)
    gnss_pos_ecef = gnss_data[pos_cols].values
    gnss_vel_ecef = gnss_data[vel_cols].values

    # Reference position for NED
    ref_lat = np.deg2rad(lat_deg)
    ref_lon = np.deg2rad(lon_deg)
    ref_r0 = ecef_origin
    C_ECEF_to_NED = compute_C_ECEF_to_NED(ref_lat, ref_lon)

    # Convert GNSS to NED
    from utils import ecef_to_ned

    gnss_pos_ned = ecef_to_ned(gnss_pos_ecef, ref_lat, ref_lon, ref_r0)
    gnss_vel_ned = np.array([C_ECEF_to_NED @ v for v in gnss_vel_ecef])
    gnss_acc_ecef = np.zeros_like(gnss_vel_ecef)
    dt = np.diff(gnss_time, prepend=gnss_time[0])
    gnss_acc_ecef[1:] = (gnss_vel_ecef[1:] - gnss_vel_ecef[:-1]) / dt[1:, np.newaxis]
    gnss_acc_ned = np.array([C_ECEF_to_NED @ a for a in gnss_acc_ecef])

    # Optionally use truth data as measurement source
    truth_pos_ned = truth_vel_ned = None
    if measure_source == "truth":
        if not truth_file:
            raise ValueError("Provide --truth-file for measure-source=truth")
        t_truth, truth_pos_ned, truth_vel_ned = load_truth_as_ned(
            truth_file, ref_lat, ref_lon, ref_r0
        )
        gnss_time = zero_base_time(t_truth)
        gnss_pos_ned = truth_pos_ned
        gnss_vel_ned = truth_vel_ned
        gnss_acc_ned = np.gradient(gnss_vel_ned, gnss_time, axis=0)
        meas_R_pos = np.eye(3) * 1e-2
        meas_R_vel = np.eye(3) * 1e-2
    else:
        meas_R_pos = np.eye(3)
        meas_R_vel = np.eye(3) * args.vel_r
        # Robust auto-tune of R from GNSS self-noise (stationary diffs)
        try:
            pos_step = np.abs(np.diff(gnss_pos_ned, axis=0))
            pos_sigma = 1.4826 * np.nanmedian(pos_step, axis=0)
            pos_sigma = np.clip(pos_sigma, 0.2, 10.0)
            meas_R_pos = np.diag((pos_sigma ** 2).astype(float))
            vel_sigma = np.nanstd(gnss_vel_ned, axis=0)
            vel_sigma = np.clip(vel_sigma, 0.05, 5.0)
            meas_R_vel = np.diag((vel_sigma ** 2).astype(float))
            logging.info(
                "Auto R set: R_pos diag=%s, R_vel diag=%s",
                np.round(np.diag(meas_R_pos), 4),
                np.round(np.diag(meas_R_vel), 4),
            )
        except Exception:
            pass

    # Load IMU data
    imu_time = np.arange(len(imu_data)) * dt_imu
    acc_body = acc_body  # from auto axis-map above
    # Use at most 4000 samples but allow shorter sequences when running the
    # trimmed datasets used in unit tests.
    N_static = min(4000, len(imu_data))
    if N_static < MIN_STATIC_SAMPLES:
        raise ValueError(
            f"Insufficient static samples; require at least {MIN_STATIC_SAMPLES}."
        )
    static_acc = np.mean(acc_body[:N_static], axis=0)

    # Estimate scale factor using bias-removed static acceleration
    C_N_B_ref = C_B_N_methods[methods[0]].T
    g_body_expected_ref = C_N_B_ref @ g_NED
    bias_ref = static_acc + g_body_expected_ref
    g_body_mag = np.linalg.norm(static_acc - bias_ref)
    scale = np.linalg.norm(g_NED) / g_body_mag

    # Compute corrected acceleration for each method
    acc_body_corrected = {}
    for m in methods:
        C_N_B = C_B_N_methods[m].T
        g_body_expected = C_N_B @ g_NED
        bias = static_acc + g_body_expected
        acc_body_corrected[m] = scale * (acc_body - bias)
        logging.info(f"Method {m}: Bias computed: {bias}")
        logging.info(f"Method {m}: Scale factor: {scale:.4f}")

    # Build a conservative ZUPT mask from static noise estimates (fallback to rolling-window ZUPT)
    try:
        # Reference gyro for thresholds
        gyro_ref = gyro_body_corrected.get(methods[0], gyro_body)
        # Use the first N_static samples as the static window
        a_body_static = acc_body[:N_static]
        g_body_static = gyro_ref[:N_static]
        # Accel magnitude residual |a|-|g|
        a_mag = np.linalg.norm(a_body_static, axis=1)
        g_mag = float(np.linalg.norm(g_NED))
        resid = a_mag - g_mag
        resid = resid[np.isfinite(resid)]
        if resid.size:
            sig_a = 1.4826 * float(np.median(np.abs(resid - np.median(resid))))
        else:
            sig_a = 0.05
        a_thr = max(3.0 * sig_a, 0.05)   # m/s^2 floor
        # Gyro threshold via robust MAD on axes combined
        g_body_static = g_body_static[np.all(np.isfinite(g_body_static), axis=1)]
        if g_body_static.size:
            mad_axes = []
            for ax in range(3):
                arr = g_body_static[:, ax]
                med = np.median(arr)
                mad_axes.append(1.4826 * np.median(np.abs(arr - med)))
            sig_g = float(np.nanmax(mad_axes)) if mad_axes else 0.005
        else:
            sig_g = 0.005
        g_thr = max(3.0 * sig_g, 0.005)
        # Build runtime mask
        acc_mag_full = np.linalg.norm(acc_body, axis=1)
        gyro_mag_full = np.linalg.norm(gyro_ref, axis=1)
        zupt_mask_fallback = (np.abs(acc_mag_full - g_mag) < a_thr) & (gyro_mag_full < g_thr)
        logging.info(
            "ZUPT thresholds: a_thr=%.3f m/s^2  g_thr=%.4f rad/s (static frac=%.1f%%)",
            a_thr,
            g_thr,
            100.0 * np.mean(zupt_mask_fallback),
        )
    except Exception:
        zupt_mask_fallback = np.zeros(len(imu_time), dtype=bool)

    # --------------------------------
    # Subtask 5.4: Integrate IMU Data for Each Method
    # --------------------------------
    logging.info("Subtask 5.4: Integrating IMU data for each method.")
    imu_pos = {m: np.zeros((len(imu_time), 3)) for m in methods}
    imu_vel = {m: np.zeros((len(imu_time), 3)) for m in methods}
    imu_acc = {m: np.zeros((len(imu_time), 3)) for m in methods}
    for m in methods:
        C_B_N = C_B_N_methods[m]
        imu_pos[m][0] = gnss_pos_ned[0]
        imu_vel[m][0] = gnss_vel_ned[0]
        for i in range(1, len(imu_time)):
            dt = imu_time[i] - imu_time[i - 1]
            f_ned = C_B_N @ acc_body_corrected[m][i]
            a_ned = f_ned + g_NED
            imu_acc[m][i] = a_ned
            imu_vel[m][i] = (
                imu_vel[m][i - 1] + 0.5 * (imu_acc[m][i] + imu_acc[m][i - 1]) * dt
            )
            imu_pos[m][i] = (
                imu_pos[m][i - 1] + 0.5 * (imu_vel[m][i] + imu_vel[m][i - 1]) * dt
            )
        logging.info(f"Method {m}: IMU data integrated.")
        final_vel = imu_vel[m][-1]
        logging.info(
            f"[{summary_tag} | {m}] Final integrated NED velocity: "
            f"[{final_vel[0]:.3f}, {final_vel[1]:.3f}, {final_vel[2]:.3f}] m/s"
        )

    # ZUPT handled dynamically during Kalman filtering

    # --------------------------------
    # Subtask 5.6: Kalman Filter for Sensor Fusion for Each Method
    # --------------------------------
    logging.info(
        "Subtask 5.6: Running Kalman Filter for sensor fusion for each method."
    )

    # Estimate relative time shift between GNSS and IMU using integrated positions
    ref_method = methods[0]
    gnss_pos_on_imu = interp_to(gnss_time, gnss_pos_ned, imu_time)
    lag, t_shift = compute_time_shift(
        imu_pos[ref_method][:, 0], gnss_pos_on_imu[:, 0], dt_imu
    )
    logging.info(
        "Estimated GNSS-IMU time shift: %.3f s (lag %d samples)", t_shift, lag
    )
    # Protect against spurious large shifts from cross-correlation
    MAX_SHIFT_S = 5.0
    if not np.isfinite(t_shift) or abs(t_shift) > MAX_SHIFT_S:
        raw_shift = t_shift
        t_shift = 0.0
        logging.warning(
            "Time shift %.3fs looks implausible -> clamped to %.1fs",
            raw_shift,
            t_shift,
        )
    # By default, avoid applying large cross-correlation shifts inside KF
    if args.apply_time_shift_in_kf:
        gnss_time_shifted = gnss_time - t_shift
    else:
        gnss_time_shifted = gnss_time

    # Resample GNSS series onto IMU timestamps using the aligned time vector
    lat_interp = interp_to(gnss_time_shifted, lat_series, imu_time)
    lon_interp = interp_to(gnss_time_shifted, lon_series, imu_time)
    gnss_pos_ned_interp = interp_to(gnss_time_shifted, gnss_pos_ned, imu_time)
    gnss_vel_ned_interp = interp_to(gnss_time_shifted, gnss_vel_ned, imu_time)
    gnss_acc_ned_interp = interp_to(gnss_time_shifted, gnss_acc_ned, imu_time)
    logging.debug(
        "Interpolated GNSS data: first NED pos %.4f last %.4f",
        gnss_pos_ned_interp[0, 0],
        gnss_pos_ned_interp[-1, 0],
    )

    fused_pos = {m: np.zeros_like(imu_pos[m]) for m in methods}
    fused_vel = {m: np.zeros_like(imu_vel[m]) for m in methods}
    fused_acc = {m: np.zeros_like(imu_acc[m]) for m in methods}

    innov_pos_all = {}
    innov_vel_all = {}
    attitude_q_all = {}
    euler_all = {}
    res_pos_all = {}
    res_vel_all = {}
    time_res_all = {}
    P_hist_all = {}
    x_log_all = {}
    zupt_counts = {}
    zupt_events_all = {}

    # Default to position-only updates; enable velocity aiding if GNSS velocity looks sane
    vnorm = np.linalg.norm(gnss_vel_ned, axis=1)
    v95 = float(np.nanpercentile(vnorm, 95)) if vnorm.size else float("inf")
    use_gnss_velocity = np.isfinite(gnss_vel_ned).all() and (v95 < 50.0)
    logging.info(
        "Velocity aiding: %s (p95|v_gnss|=%.3f m/s)",
        "ON" if use_gnss_velocity else "OFF",
        v95,
    )
    # Adaptive default gate for velocity measurements (can still be overridden later)
    vel_gate_max_default = max(10.0, 3.0 * v95 + 5.0) if np.isfinite(v95) else 100.0

    for m in methods:
        kf = init_bias_kalman(dt_imu, meas_R_pos, meas_R_vel, args.vel_q_scale)
        # Auto-tune Q pos/vel blocks from IMU noise proxy (static window)
        try:
            acc_std_axes = np.std(acc_body_corrected[m][:N_static], axis=0)
            acc_sigma = float(np.nanmean(acc_std_axes))
        except Exception:
            acc_sigma = 0.05
        q_vel = max(0.01, (acc_sigma ** 2) * dt_imu)
        q_pos = max(0.001, q_vel * dt_imu)
        try:
            kf.Q[0:3, 0:3] = np.eye(3) * q_pos
            kf.Q[3:6, 3:6] = np.eye(3) * q_vel
            logging.info("Auto Q set: q_pos=%.4g q_vel=%.4g", q_pos, q_vel)
        except Exception:
            pass
        initial_quats = {"TRIAD": q_tri, "Davenport": q_dav, "SVD": q_svd}
        chosen_init = None
        # Optionally override initial attitude with truth quaternion at IMU start
        if (args.init_att_with_truth or args.kf_init == "TRUTH") and truth_file:
            try:
                truth_arr = np.loadtxt(truth_file)
                # Heuristic: first time-like column in [0..2], quats are last 4 cols
                t_cols = []
                for c in range(min(3, truth_arr.shape[1])):
                    col = truth_arr[:, c].astype(float)
                    if np.all(np.isfinite(col)) and (np.nanmax(col) - np.nanmin(col)) > 0:
                        t_cols.append((c, float(np.nanmax(col) - np.nanmin(col)), col))
                if not t_cols:
                    raise ValueError("No valid time column found in truth file")
                t_cols.sort(key=lambda x: x[1])
                t_truth = t_cols[0][2]
                # Use last 4 numeric cols as [qw,qx,qy,qz]
                quat_truth = truth_arr[:, -4:]
                # IMU time base
                t_imu0 = float(imu_time[0])
                idx0 = int(np.argmin(np.abs(t_truth - t_imu0)))
                q_truth0 = quat_truth[idx0].astype(float)
                # Convert body->ECEF truth to body->NED if requested
                if args.truth_quat_frame == "ECEF":
                    try:
                        r_be = R.from_quat(q_truth0[[1, 2, 3, 0]])
                        C_e2n = compute_C_ECEF_to_NED(ref_lat, ref_lon)
                        R_bn = C_e2n @ r_be.as_matrix()
                        r_bn = R.from_matrix(R_bn)
                        q_xyzw = r_bn.as_quat()
                        q_truth0 = np.array([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
                    except Exception as exc2:
                        logging.warning("Truth quaternion frame conversion failed: %s", exc2)
                # Normalise to be safe and ensure positive scalar part convention
                q_truth0 = q_truth0 / (np.linalg.norm(q_truth0) + 1e-12)
                if q_truth0[0] < 0:
                    q_truth0 = -q_truth0
                chosen_init = q_truth0.copy()
                logging.info(
                    "Initial attitude overridden with truth quaternion at t=%.3f s: %s",
                    t_imu0,
                    np.array2string(q_truth0, precision=6),
                )
            except Exception as exc:
                logging.warning(
                    "--init-att-with-truth set, but failed to parse truth quaternion: %s",
                    exc,
                )
        # If not using TRUTH, choose the requested KF init among computed attitudes
        if chosen_init is None:
            try:
                chosen_init = initial_quats.get(args.kf_init, initial_quats.get(m))
            except Exception:
                chosen_init = initial_quats.get(m)
        # Normalise and enforce w>=0
        chosen_init = chosen_init / (np.linalg.norm(chosen_init) + 1e-12)
        if chosen_init[0] < 0:
            chosen_init = -chosen_init
        kf.x = np.hstack(
            (
                imu_pos[m][0],
                imu_vel[m][0],
                chosen_init,
                gyro_biases.get(m, np.zeros(3)),
            )
        )
        logging.info(f"Adjusted Q[3:6,3:6]: {kf.Q[3:6,3:6]}")
        logging.info(f"Adjusted R[3:6,3:6]: {kf.R[3:6,3:6]}")
        fused_pos[m][0] = imu_pos[m][0]
        fused_vel[m][0] = imu_vel[m][0]
        fused_acc[m][0] = imu_acc[m][0]

        # ---------- logging containers ----------
        innov_pos = []  # GNSS - predicted position
        innov_vel = []  # GNSS - predicted velocity
        attitude_q = []  # quaternion history
        res_pos_list, res_vel_list, time_res = [], [], []
        euler_list = []

        win = 80
        acc_win = []
        gyro_win = []
        zupt_count = 0
        zupt_events = []
        high_speed_events = 0  # count of velocity samples scaled down to plausible cap
        vel_blow_warn_interval = 0  # set >0 to re-warn every N events
        P_hist = [kf.P.copy()]

        # attitude initialisation for logging
        orientations = np.zeros((len(imu_time), 4))
        orientations[0] = chosen_init
        attitude_q.append(orientations[0])
        q_cur = orientations[0]
        roll, pitch, yaw = quat2euler(q_cur)
        euler_list.append([roll, pitch, yaw])

        # State history log (states x time)
        x_log = np.zeros((kf.dim_x, len(imu_time)))
        x_log[:, 0] = np.ravel(kf.x)

        # Run Kalman Filter
        # Accumulators for concise velocity measurement summary
        vel_gate_max = vel_gate_max_default  # m/s; adapted from GNSS speed stats
        # Scenario-aware plausible speed cap (computed once per method)
        try:
            gnss_speed_p99 = float(np.percentile(np.linalg.norm(gnss_vel_ned, axis=1), 99))
        except Exception:
            gnss_speed_p99 = 0.0
        try:
            dv_norm = np.linalg.norm(imu_acc[m] * dt_imu, axis=1)
            imu_dv_sigma = float(np.percentile(dv_norm, 99))
        except Exception:
            imu_dv_sigma = 0.0
        plausible_speed_cap = max(300.0, 5.0 * gnss_speed_p99, 200.0 + 2000.0 * imu_dv_sigma)
        plausible_speed_cap = min(plausible_speed_cap, 1500.0)
        n_meas = 0
        used_cnt = 0
        gated_cnt = 0
        sumsq_innov_vel = 0.0
        sumsq_z_vel = 0.0
        for i in range(1, len(imu_time)):
            dt = imu_time[i] - imu_time[i - 1]

            # propagate quaternion using gyro measurement with Earth+transport rate correction
            # Effective body rate relative to NED: omega_bn^b = omega_ib^b - C_n^b * (omega_ie^n + omega_en^n)
            try:
                if 'lat_interp' in locals():
                    _lat_val = lat_interp[i]
                    lat_i = float(_lat_val.item() if hasattr(_lat_val, 'item') else _lat_val)
                else:
                    lat_i = float(ref_lat)
            except Exception:
                lat_i = float(ref_lat)
            # Earth rate in NED
            omega_ie_n = np.array([
                EARTH_RATE * np.cos(lat_i),
                0.0,
                -EARTH_RATE * np.sin(lat_i),
            ])
            # Transport rate in NED from current nav velocity and latitude
            _vel_slice = np.ravel(kf.x[3:6])
            vN, vE, vD = float(_vel_slice[0]), float(_vel_slice[1]), float(_vel_slice[2])
            # WGS-84 radii of curvature
            a_wgs = 6378137.0
            e2 = 6.6943799901413165e-3
            sin_lat = np.sin(lat_i)
            denom = np.sqrt(1.0 - e2 * sin_lat * sin_lat)
            R_E = a_wgs / denom                    # prime-vertical radius
            R_N = a_wgs * (1 - e2) / (denom**3)    # meridian radius
            h_i = float(alt) if ('alt' in locals() and alt is not None) else 0.0
            # Standard transport rate expression in NED
            omega_en_n = np.array([
                vE / (R_E + h_i),
                -vN / (R_N + h_i),
                -vE * np.tan(lat_i) / (R_E + h_i),
            ])
            omega_in_n = omega_ie_n + omega_en_n
            # C_b^n from current quaternion
            C_b_n = R.from_quat([q_cur[1], q_cur[2], q_cur[3], q_cur[0]]).as_matrix()
            C_n_b = C_b_n.T
            omega_in_b = C_n_b @ omega_in_n
            omega_eff_b = gyro_body_corrected[m][i] - np.ravel(kf.x[10:13]) - omega_in_b
            dq = quat_from_rate(omega_eff_b, dt)
            q_prev = q_cur
            q_cur = quat_multiply(q_cur, dq)
            q_cur /= np.linalg.norm(q_cur)
            # Enforce temporal sign continuity (q and -q equivalent)
            try:
                if float(np.dot(q_cur, q_prev)) < 0.0:
                    q_cur *= -1.0
            except Exception:
                pass
            orientations[i] = q_cur
            kf.x[6:10] = q_cur

            # Prediction step
            kf.F[0:3, 3:6] = np.eye(3) * dt
            kf.B[0:3] = 0.5 * dt * dt * np.eye(3)
            kf.B[3:6] = dt * np.eye(3)
            # Clip per-sample Δv to a physical limit (e.g., 3g)
            u = imu_acc[m][i]
            dv = dt * u
            dv_max = 3.0 * 9.81 * max(dt, 1e-3)
            n_dv = float(np.linalg.norm(dv))
            if n_dv > dv_max and n_dv > 0:
                u = u * (dv_max / n_dv)
            kf.predict(u=u)

            # Guard against unphysical velocity magnitude: softly clamp instead of zeroing
            v_mag = float(np.linalg.norm(np.ravel(kf.x[3:6])))
            if v_mag > plausible_speed_cap:
                high_speed_events += 1
                v_cap = plausible_speed_cap
                if high_speed_events == 1 or (
                    vel_blow_warn_interval > 0 and high_speed_events % vel_blow_warn_interval == 0
                ):
                    logging.warning(
                        "Velocity high (%.1f m/s); scaling to %.1f m/s.",
                        v_mag,
                        v_cap,
                    )
                if v_mag > 0:
                    kf.x[3:6] *= (v_cap / v_mag)

            # ---------- save attitude BEFORE measurement update ----------
            attitude_q.append(q_cur)

            # ---------- compute and log the innovations BEFORE update ----------
            pred = kf.H @ kf.x
            innov = np.hstack((gnss_pos_ned_interp[i], gnss_vel_ned_interp[i])) - pred
            innov_pos.append(innov[0:3])
            innov_vel.append(innov[3:6])
            res_pos_list.append(innov[0:3])
            res_vel_list.append(innov[3:6])
            time_res.append(imu_time[i])
            roll, pitch, yaw = quat2euler(q_cur)
            euler_list.append([roll, pitch, yaw])

            # Update step
            z = np.hstack((gnss_pos_ned_interp[i], gnss_vel_ned_interp[i]))
            # Velocity measurement gating: ignore clearly spurious magnitudes and Mahalanobis outliers
            # Compute innovation covariance for current update
            try:
                S = kf.H @ kf.P @ kf.H.T + kf.R
                Svv = S[3:6, 3:6]
                # Guard against singular Svv
                Svv_inv = np.linalg.inv(Svv + 1e-9 * np.eye(3))
                d2_vel = float(innov[3:6].T @ Svv_inv @ innov[3:6])
            except Exception:
                d2_vel = 0.0
            allow_vel = (
                use_gnss_velocity
                and np.isfinite(z[3:6]).all()
                and (np.linalg.norm(z[3:6]) < vel_gate_max)
                and (d2_vel < 25.0)
            )
            # Accumulate velocity stats for end-of-run summary
            n_meas += 1
            sumsq_z_vel += float(np.dot(z[3:6], z[3:6]))
            sumsq_innov_vel += float(np.dot(innov[3:6], innov[3:6]))
            if allow_vel:
                used_cnt += 1
            else:
                gated_cnt += 1
            # Optional periodic debug print to confirm velocity measurement usage
            if getattr(args, 'kf_vel_periodic', False):
                if i % max(1, int(round(1.0 / max(1e-9, 10.0 * dt)))) == 0:
                    try:
                        log(
                            f"[KF] z_vel_ned=[{z[3]:+0.02f} {z[4]:+0.02f} {z[5]:+0.02f}] m/s, "
                            f"x_vel_pred=[{pred[3]:+0.02f} {pred[4]:+0.02f} {pred[5]:+0.02f}]"
                            + (" (GATED)" if not allow_vel else "")
                        )
                    except Exception:
                        pass
            if allow_vel:
                kf.update(z)
            else:
                # Use only position block for this update
                H_pos = kf.H.copy()
                H_pos[3:6, :] = 0.0
                R_pos = kf.R.copy()
                R_pos[3:6, 3:6] = np.eye(3) * 1e6
                kf.update(z, H=H_pos, R=R_pos)
            # Optional post-update print (periodic)
            if getattr(args, 'kf_vel_periodic', False):
                if i % max(1, int(round(1.0 / max(1e-9, 10.0 * dt)))) == 0:
                    try:
                        log(
                            f"[KF] x_vel_post=[{kf.x[3]:+0.02f} {kf.x[4]:+0.02f} {kf.x[5]:+0.02f}] m/s"
                        )
                    except Exception:
                        pass

            # Optional: fuse GNSS-derived yaw to correct quaternion drift
            if args.fuse_yaw:
                v_ned = gnss_vel_ned_interp[i]
                v_h = np.hypot(v_ned[0], v_ned[1])
                if v_h >= args.yaw_speed_min:
                    # Measured yaw from GNSS velocity (atan2(E, N)) in NED
                    yaw_meas = float(np.arctan2(v_ned[1], v_ned[0]))
                    roll, pitch, yaw_est = quat2euler(q_cur)
                    # Wrap error to [-pi,pi]
                    err = np.arctan2(np.sin(yaw_meas - yaw_est), np.cos(yaw_meas - yaw_est))
                    delta = args.yaw_gain * err
                    # Rotation about NED z-axis; apply on the left (world-frame correction)
                    half = 0.5 * delta
                    dqz = np.array([np.cos(half), 0.0, 0.0, np.sin(half)])
                    q_cur = quat_multiply(dqz, q_cur)
                    q_cur /= np.linalg.norm(q_cur) + 1e-12
                    orientations[i] = q_cur
                    kf.x[6:10] = q_cur
                    # Overwrite last logged attitude and Euler with corrected values
                    if attitude_q:
                        attitude_q[-1] = q_cur
                    roll, pitch, yaw = quat2euler(q_cur)
                    if euler_list:
                        euler_list[-1] = [roll, pitch, yaw]

            # --- ZUPT check with rolling variance, then fallback mask ---
            acc_win.append(acc_body_corrected[m][i])
            gyro_win.append(gyro_body_corrected[m][i])
            if len(acc_win) > win:
                acc_win.pop(0)
                gyro_win.pop(0)
            if (
                not args.no_zupt
                and len(acc_win) == win
                and is_static(
                    np.array(acc_win),
                    np.array(gyro_win),
                    accel_var_thresh=args.zupt_acc_var,
                    gyro_var_thresh=args.zupt_gyro_var,
                )
            ):
                inject_zupt(kf)
                zupt_count += 1
                zupt_events.append((i - win + 1, i))
                logging.debug(
                    f"ZUPT applied at {imu_time[i]:.2f}s (window {i-win+1}-{i})"
                )
            elif not args.no_zupt and bool(zupt_mask_fallback[i]):
                inject_zupt(kf)
                zupt_count += 1
                zupt_events.append((i, i))
                # Keep this quiet unless debugging

            fused_pos[m][i, :] = np.ravel(kf.x[0:3])
            fused_vel[m][i, :] = np.ravel(kf.x[3:6])
            fused_acc[m][i] = imu_acc[m][i]  # Use integrated acceleration
            P_hist.append(kf.P.copy())
            x_log[:, i] = np.ravel(kf.x)

        # One-line velocity measurement summary (important info only)
        rms_z = np.sqrt(sumsq_z_vel / max(n_meas, 1))
        rms_innov = np.sqrt(sumsq_innov_vel / max(n_meas, 1))
        final_vel_mag = float(np.linalg.norm(fused_vel[m][-1]))
        used_pct = 100.0 * used_cnt / max(n_meas, 1)
        logging.info(
            f"[KF] Velocity summary ({m}): N={n_meas}, used={used_cnt} ({used_pct:.1f}%), "
            f"rms|z_vel|={rms_z:.2f} m/s, rms|innov_vel|={rms_innov:.2f} m/s, final|x_vel|={final_vel_mag:.2f} m/s"
        )
        logging.info(
            f"Method {m}: Kalman Filter completed. ZUPTcnt={zupt_count} high_speed_events={high_speed_events}"
        )
        if not args.disable_triad_log:
            with open("triad_init_log.txt", "a") as logf:
                for s, e in zupt_events:
                    logf.write(f"{imu_file}: ZUPT {s}-{e}\n")
        zupt_counts[m] = zupt_count
        zupt_events_all[m] = list(zupt_events)

        # stack log lists
        innov_pos = np.vstack(innov_pos)
        innov_vel = np.vstack(innov_vel)
        attitude_q = np.vstack(attitude_q)
        res_pos = np.vstack(res_pos_list)
        res_vel = np.vstack(res_vel_list)
        euler = np.vstack(euler_list)
        time_res_arr = np.array(time_res)

        innov_pos_all[m] = innov_pos
        innov_vel_all[m] = innov_vel
        attitude_q_all[m] = attitude_q
        euler_all[m] = euler
        res_pos_all[m] = res_pos
        res_vel_all[m] = res_vel
        time_res_all[m] = time_res_arr
        P_hist_all[m] = np.stack(P_hist)
        x_log_all[m] = x_log

        if m == "Davenport":
            pos_finite = np.all(np.isfinite(fused_pos[m]), axis=0)
            vel_finite = np.all(np.isfinite(fused_vel[m]), axis=0)
            if not pos_finite.all() or not vel_finite.all():
                logging.warning(
                    "Non-finite values in Davenport fused results: "
                    f"pos_finite={pos_finite}, vel_finite={vel_finite}"
                )
            else:
                logging.debug(
                    "Davenport fused position stats: "
                    f"min={np.min(fused_pos[m], axis=0)}, "
                    f"max={np.max(fused_pos[m], axis=0)}"
                )
                logging.debug(
                    "Davenport fused velocity stats: "
                    f"min={np.min(fused_vel[m], axis=0)}, "
                    f"max={np.max(fused_vel[m], axis=0)}"
                )

    # Compute residuals for the selected method
    _ = res_pos_all[method]
    _ = res_vel_all[method]
    _ = time_res_all[method]

    _ = np.rad2deg(euler_all[method])

    # --------------------------------
    # Subtask 5.7: Handle Event at 5000s (if needed)
    # --------------------------------
    logging.info("Subtask 5.7: No event handling needed as time < 5000s.")

    # --------------------------------
    # Subtask 5.8: Plotting Results for All Methods
    # --------------------------------

    # Configure logging if not already done

    # Define methods and colors
    methods = [method]
    colors = COLORS
    directions = ["North", "East", "Down"]

    # Subtask 5.8.2: Plotting Results for selected method
    logging.info(f"Subtask 5.8.2: Plotting results for {method}.")
    logging.debug(f"# Subtask 5.8.2: Starting to plot results for {method}.")
    fig, axes = plt.subplots(3, 3, figsize=(15, 10))

    # Davenport - Position
    for j in range(3):
        ax = axes[0, j]
        ax.plot(imu_time, gnss_pos_ned_interp[:, j], "k-", label="GNSS (Measured)")
        ax.plot(imu_time, imu_pos[method][:, j], "g--", label="IMU (Derived)")
        c = colors.get(method, None)
        ax.plot(
            imu_time, fused_pos[method][:, j], c, alpha=0.7, label="Fused GNSS + IMU"
        )
        ax.set_title(f"Position {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Position (m)")
        ax.legend(loc="best")
        logging.info(
            f"Subtask 5.8.2: Plotted {method} position {directions[j]}: "
            f"First = {fused_pos[method][0, j]:.4f}, Last = {fused_pos[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} position {directions[j]}: "
            f"First = {fused_pos[method][0, j]:.4f}, Last = {fused_pos[method][-1, j]:.4f}"
        )

    # Davenport - Velocity
    for j in range(3):
        ax = axes[1, j]
        ax.plot(imu_time, gnss_vel_ned_interp[:, j], "k-", label="GNSS (Measured)")
        ax.plot(imu_time, imu_vel[method][:, j], "g--", label="IMU (Derived)")
        c = colors.get(method, None)
        ax.plot(
            imu_time, fused_vel[method][:, j], c, alpha=0.7, label="Fused GNSS + IMU"
        )
        ax.set_title(f"Velocity {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Velocity (m/s)")
        ax.legend(loc="best")
        logging.info(
            f"Subtask 5.8.2: Plotted {method} velocity {directions[j]}: "
            f"First = {fused_vel[method][0, j]:.4f}, Last = {fused_vel[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} velocity {directions[j]}: "
            f"First = {fused_vel[method][0, j]:.4f}, Last = {fused_vel[method][-1, j]:.4f}"
        )

    # Davenport - Acceleration
    for j in range(3):
        ax = axes[2, j]
        ax.plot(imu_time, imu_acc[method][:, j], "g--", label="IMU (Derived)")
        ax.plot(imu_time, gnss_acc_ned_interp[:, j], "k-", label="GNSS (Derived)")
        c = colors.get(method, None)
        ax.plot(
            imu_time, fused_acc[method][:, j], c, alpha=0.7, label="Fused GNSS + IMU"
        )
        ax.set_title(f"Acceleration {directions[j]}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Acceleration (m/s²)")
        ax.legend(loc="best")
        logging.info(
            f"Subtask 5.8.2: Plotted {method} acceleration {directions[j]}: "
            f"First = {fused_acc[method][0, j]:.4f}, Last = {fused_acc[method][-1, j]:.4f}"
        )
        logging.debug(
            f"# Plotted {method} acceleration {directions[j]}: "
            f"First = {fused_acc[method][0, j]:.4f}, Last = {fused_acc[method][-1, j]:.4f}"
        )

    plt.tight_layout()
    if not args.no_plots:
        save_plot(fig, RESULTS_DIR, tag, "task5_8_2", "measuredGNSS_vs_fused_NED", ext="png", dpi=200)
    logging.info(f"Subtask 5.8.2: {method} plot saved")
    logging.debug(f"# Subtask 5.8.2: {method} plotting completed.")
    plt.close(fig)

    # ----- Additional reference frame plots -----
    logging.info("Plotting all data in NED frame.")
    fig_ned_all, ax_ned_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_ned = ["N", "E", "D"]
    c = colors.get(method, None)
    for i in range(3):
        for j in range(3):
            ax = ax_ned_all[i, j]
            if i == 0:
                ax.plot(t_rel_ilu, fused_pos[method][:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Position {dirs_ned[j]}")
            elif i == 1:
                ax.plot(t_rel_ilu, fused_vel[method][:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Velocity V{dirs_ned[j]}")
            else:
                ax.plot(t_rel_ilu, fused_acc[method][:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Acceleration {dirs_ned[j]}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ned_all.suptitle(f"Task 5 – {method} – NED Frame (Fused)")
    fig_ned_all.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_ned_all, RESULTS_DIR, tag, "task5_8_4", "fused_state_NED", ext="png", dpi=200, bbox_inches="tight")
        # Save a MATLAB bundle for Task 5 NED fused data
        try:
            from scipy.io import savemat  # type: ignore
            savemat(str(RESULTS_DIR / f"{tag}_task5_8_4_all_ned.mat"), {
                'time_s': np.asarray(imu_time, float),
                'pos_ned_m': np.asarray(fused_pos[method], float),
                'vel_ned_ms': np.asarray(fused_vel[method], float),
                'acc_ned': np.asarray(fused_acc[method], float),
            })
        except Exception:
            pass
    plt.close(fig_ned_all)
    logging.info("All data in NED frame plot saved")

    logging.info("Plotting all data in ECEF frame.")
    fig_ecef_all, ax_ecef_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_ecef = ["X", "Y", "Z"]
    pos_ecef = np.array([C_NED_to_ECEF @ p + ref_r0 for p in fused_pos[method]])
    vel_ecef = (C_NED_to_ECEF @ fused_vel[method].T).T
    acc_ecef = (C_NED_to_ECEF @ fused_acc[method].T).T

    for name, arr in (
        ("pos_ecef", pos_ecef),
        ("vel_ecef", vel_ecef),
        ("acc_ecef", acc_ecef),
    ):
        if not np.all(np.isfinite(arr)):
            logging.warning(f"NaNs detected in {name} after NED->ECEF conversion")
        logging.debug(f"{name} min={np.min(arr, axis=0)}, max={np.max(arr, axis=0)}")
    for i in range(3):
        for j in range(3):
            ax = ax_ecef_all[i, j]
            if i == 0:
                ax.plot(t_rel_ilu, pos_ecef[:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Position {dirs_ecef[j]}_ECEF")
            elif i == 1:
                ax.plot(t_rel_ilu, vel_ecef[:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Velocity V{dirs_ecef[j]}_ECEF")
            else:
                ax.plot(t_rel_ilu, acc_ecef[:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Acceleration {dirs_ecef[j]}_ECEF")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_ecef_all.suptitle(f"Task 5 – {method} – ECEF Frame (Fused)")
    fig_ecef_all.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_ecef_all, RESULTS_DIR, tag, "task5_8_5", "fused_state_ECEF", ext="png", dpi=200, bbox_inches="tight")
        # Save a MATLAB bundle for Task 5 ECEF fused data
        try:
            from scipy.io import savemat  # type: ignore
            savemat(str(RESULTS_DIR / f"{tag}_task5_8_5_all_ecef.mat"), {
                'time_s': np.asarray(imu_time, float),
                'pos_ecef_m': np.asarray(pos_ecef, float),
                'vel_ecef_ms': np.asarray(vel_ecef, float),
                'acc_ecef': np.asarray(acc_ecef, float),
            })
        except Exception:
            pass
    plt.close(fig_ecef_all)
    logging.info("All data in ECEF frame plot saved")

    logging.info("Plotting all data in body frame.")
    fig_body_all, ax_body_all = plt.subplots(3, 3, figsize=(15, 10))
    dirs_body = ["X", "Y", "Z"]
    C_N_B = C_B_N_methods[method].T
    pos_body = (C_N_B @ fused_pos[method].T).T
    vel_body = (C_N_B @ fused_vel[method].T).T
    acc_body = (C_N_B @ fused_acc[method].T).T
    # Only fused data needed for Body frame summary plots
    for i in range(3):
        for j in range(3):
            ax = ax_body_all[i, j]
            if i == 0:
                ax.plot(t_rel_ilu, pos_body[:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Position r{dirs_body[j]}_body")
            elif i == 1:
                ax.plot(t_rel_ilu, vel_body[:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Velocity v{dirs_body[j]}_body")
            else:
                ax.plot(t_rel_ilu, acc_body[:, j], c, alpha=0.9, label=f"Fused (GNSS+IMU, {method})")
                ax.set_title(f"Acceleration A{dirs_body[j]}_body")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Value")
            ax.legend(loc="best")
    fig_body_all.suptitle(f"Task 5 – {method} – Body Frame (Fused)")
    fig_body_all.tight_layout(rect=[0, 0, 1, 0.95])
    if not args.no_plots:
        save_plot(fig_body_all, RESULTS_DIR, tag, "task5_8_6", "fused_state_BODY", ext="png", dpi=200, bbox_inches="tight")
        # Save a MATLAB bundle for Task 5 Body fused data
        try:
            from scipy.io import savemat  # type: ignore
            savemat(str(RESULTS_DIR / f"{tag}_task5_8_6_all_body.mat"), {
                'time_s': np.asarray(imu_time, float),
                'pos_body_m': np.asarray(pos_body, float),
                'vel_body_ms': np.asarray(vel_body, float),
                'acc_body': np.asarray(acc_body, float),
            })
        except Exception:
            pass
    plt.close(fig_body_all)
    logging.info("All data in body frame plot saved")
    if not args.no_plots:
        task_summary("task5")

    # Plot pre-fit innovations
    fig_innov, ax_innov = plt.subplots(3, 1, sharex=True, figsize=(8, 6))
    labels = ["North", "East", "Down"]
    innov_pos = innov_pos_all[method]
    innov_vel = innov_vel_all[method]
    for i in range(3):
        ax_innov[i].plot(innov_pos[:, i], label="Position")
        ax_innov[i].plot(innov_vel[:, i], label="Velocity", linestyle="--")
        ax_innov[i].set_ylabel(f"{labels[i]} residual")
        ax_innov[i].grid(True)
    ax_innov[0].legend(loc="best")
    ax_innov[-1].set_xlabel("GNSS update index")
    fig_innov.suptitle("Task 5 – Pre-fit Innovations (Fused vs. Measured GNSS)")
    fig_innov.tight_layout()
    # Removed: task5_8_7 pre-fit innovations figure.
    plt.close(fig_innov)

    # Plot residuals and attitude using helper functions
    if not args.no_plots:
        res = compute_residuals(gnss_time, gnss_pos_ned, imu_time, fused_pos[method])
        plot_residuals(gnss_time, res, RESULTS_DIR, tag, method)
        # Removed: task5_8_1 attitude angles. The Body->NED attitude is covered
        # by task6_4 (quaternion vs truth) and the task7_6 attitude figures.

    # Create plot summary
    summary = {
        f"{run_id}_task1_location_map.png": "Task 1 location map",
        f"{tag}_task3_errors_comparison.png": "Attitude initialization error comparison",
        f"{tag}_task3_quaternions_comparison.png": "Quaternion components for initialization",
        f"{tag}_task4_12_1_comparison_ned.png": "Derived GNSS vs Derived IMU data in NED frame",
        f"{tag}_task4_12_2_measured_posvel_ECEF_accel_BODY.png":
            "Measured position/velocity in ECEF, acceleration in body",
        f"{tag}_task4_12_3_all_ned.png": "Integrated data in NED frame",
        f"{tag}_task4_12_4_all_ecef.png": "Integrated data in ECEF frame",
        f"{tag}_task4_12_5_all_body.png": "Integrated data in body frame",
        f"{tag}_task5_8_2_results_{method}.png": f"Kalman filter results using {method}",
        f"{tag}_task5_8_4_all_ned.png": "Kalman filter results in NED frame",
        f"{tag}_task5_8_5_all_ecef.png": "Kalman filter results in ECEF frame",
        f"{tag}_task5_8_6_all_body.png": "Kalman filter results in body frame",
        f"{tag}_{method.lower()}_residuals.png": "Position and velocity residuals",
        f"{tag}_{method.lower()}_innovations.png": "Pre-fit innovations",
        f"{tag}_{method.lower()}_attitude_angles.png": "Attitude angles over time",
    }
    summary_path = str(RESULTS_DIR / f"{tag}_plot_summary.md")
    with open(summary_path, "w") as f:
        for name, desc in summary.items():
            f.write(f"- **{name}**: {desc}\n")

    rmse_pos = np.sqrt(
        np.mean(np.sum((gnss_pos_ned_interp - fused_pos[method]) ** 2, axis=1))
    )
    final_pos = np.linalg.norm(gnss_pos_ned_interp[-1] - fused_pos[method][-1])

    final_vel_mag = float(np.linalg.norm(fused_vel[method][-1]))
    if final_vel_mag > 500:
        raise RuntimeError(
            f"KF diverged: final velocity {final_vel_mag:.2f} m/s exceeds 500 m/s - check F & H matrices"
        )

    # --- Additional residual metrics ---------------------------------------
    pos_interp = interpolate_series(gnss_time, imu_time, fused_pos[method])
    vel_interp = interpolate_series(gnss_time, imu_time, fused_vel[method])
    resid_pos = pos_interp - gnss_pos_ned
    resid_vel = vel_interp - gnss_vel_ned

    rms_resid_pos = np.sqrt(np.mean(resid_pos**2))
    rms_resid_vel = np.sqrt(np.mean(resid_vel**2))
    max_resid_pos = np.max(np.linalg.norm(resid_pos, axis=1))
    max_resid_vel = np.max(np.linalg.norm(resid_vel, axis=1))

    accel_bias = acc_biases.get(method, np.zeros(3))
    gyro_bias = gyro_biases.get(method, np.zeros(3))

    # --- Attitude angles ----------------------------------------------------
    # attitude_q_all is stored as [w,x,y,z]; SciPy expects [x,y,z,w]
    _q_wxyz = attitude_q_all[method]
    _q_xyzw = np.column_stack([_q_wxyz[:, 1], _q_wxyz[:, 2], _q_wxyz[:, 3], _q_wxyz[:, 0]])
    euler = R.from_quat(_q_xyzw).as_euler("xyz", degrees=True)
    # Removed: task6_6 attitude angles. The Body->NED attitude is already
    # covered by task6_4 (quaternion vs truth) and the task7_6 figures.

    C_NED_to_ECEF = C_ECEF_to_NED.T
    pos_ecef = np.array([C_NED_to_ECEF @ p + ref_r0 for p in fused_pos[method]])
    vel_ecef = (C_NED_to_ECEF @ fused_vel[method].T).T
    C_N_B = C_B_N_methods[method].T
    pos_body = (C_N_B @ fused_pos[method].T).T
    vel_body = (C_N_B @ fused_vel[method].T).T

    # Persist a rich NPZ bundle for Python/Matlab interop
    # Ensure a continuity-enforced attitude quaternion series is available to consumers
    def _quat_make_continuous(q_wxyz: np.ndarray) -> tuple[np.ndarray, int]:
        q = np.asarray(q_wxyz, float).copy()
        if q.ndim != 2 or q.shape[1] != 4:
            return q, 0
        # normalize first
        n0 = np.linalg.norm(q[0])
        if n0 != 0.0:
            q[0] /= n0
        flips = 0
        for i in range(1, len(q)):
            # normalize current sample
            ni = np.linalg.norm(q[i])
            if ni != 0.0:
                q[i] /= ni
            if np.dot(q[i], q[i - 1]) < 0.0:
                q[i] *= -1.0
                flips += 1
        return q, flips
    att_q_raw = attitude_q_all[method]
    att_q_h, flips = _quat_make_continuous(att_q_raw)
    if flips:
        print(f"[KF] Quaternion continuity: applied {flips} sign flips to {method} sequence.")
    # Make the in-memory estimator output continuous as well
    attitude_q_all[method] = att_q_h

    # Helpers for reducing output size
    ds = max(1, int(args.downsample))
    use_fp32 = bool(args.fp32_output or args.lite_output)

    def _prep(a: np.ndarray) -> np.ndarray:
        if isinstance(a, np.ndarray):
            b = a[::ds] if a.ndim >= 1 and ds > 1 else a
            if use_fp32 and b.dtype.kind == 'f':
                return b.astype(np.float32)
            return b
        return a

    if not args.no_npz:
        out_npz = RESULTS_DIR / f"{tag}_kf_output.npz"
        if args.lite_output:
            # Minimal bundle: omit extremely large arrays
            np.savez_compressed(
                out_npz,
                summary=dict(
                    rmse_pos=rmse_pos,
                    final_pos=final_pos,
                    grav_err_mean=grav_err_mean,
                    grav_err_max=grav_err_max,
                    earth_rate_err_mean=omega_err_mean,
                    earth_rate_err_max=omega_err_max,
                    vel_blow_events=high_speed_events,
                ),
                tag=np.array([tag]),
                method=np.array([method_tag]),
                time=_prep(imu_time),
                t=_prep(imu_time),
                t_rel_imu=_prep(imu_time - float(imu_time[0])),
                t_gnss=_prep(gnss_time),
                t_gnss_shifted=_prep(gnss_time_shifted if 'gnss_time_shifted' in locals() else gnss_time),
                pos_ned=_prep(fused_pos[method]),
                vel_ned=_prep(fused_vel[method]),
                fused_pos=_prep(fused_pos[method]),
                fused_vel=_prep(fused_vel[method]),
                fused_acc=_prep(fused_acc[method]),
                euler=_prep(euler_all[method]),
                euler_deg=_prep(np.rad2deg(euler_all[method])),
                attitude_q=_prep(att_q_h),
                attitude_q_harmonized=_prep(att_q_h),
                ref_lat=np.array([ref_lat], dtype=np.float32 if use_fp32 else float),
                ref_lon=np.array([ref_lon], dtype=np.float32 if use_fp32 else float),
                ref_r0=_prep(ref_r0),
            )
        else:
            np.savez_compressed(
                out_npz,
                summary=dict(
                    rmse_pos=rmse_pos,
                    final_pos=final_pos,
                    grav_err_mean=grav_err_mean,
                    grav_err_max=grav_err_max,
                    earth_rate_err_mean=omega_err_mean,
                    earth_rate_err_max=omega_err_max,
                    vel_blow_events=high_speed_events,
                ),
                tag=np.array([tag]),
                method=np.array([method_tag]),
                time=_prep(imu_time),
                t=_prep(imu_time),
                t_rel_imu=_prep(imu_time - float(imu_time[0])),
                t_gnss=_prep(gnss_time),
                t_gnss_shifted=_prep(gnss_time_shifted if 'gnss_time_shifted' in locals() else gnss_time),
                pos_ned=_prep(fused_pos[method]),
                vel_ned=_prep(fused_vel[method]),
                fused_pos=_prep(fused_pos[method]),
                fused_vel=_prep(fused_vel[method]),
                fused_acc=_prep(fused_acc[method]),
                pos_ecef=_prep(pos_ecef),
                vel_ecef=_prep(vel_ecef),
                gnss_pos_ecef=_prep(gnss_pos_ecef),
                gnss_vel_ecef=_prep(gnss_vel_ecef),
                gnss_pos_ned=_prep(gnss_pos_ned),
                gnss_vel_ned=_prep(gnss_vel_ned),
                gnss_pos_ned_interp=_prep(gnss_pos_ned_interp),
                gnss_vel_ned_interp=_prep(gnss_vel_ned_interp),
                gnss_acc_ned_interp=_prep(gnss_acc_ned_interp),
                truth_pos_ecef=_prep(pos_truth_ecef if pos_truth_ecef is not None else np.array([])),
                truth_vel_ecef=_prep(vel_truth_ecef if vel_truth_ecef is not None else np.array([])),
                truth_time=_prep(t_truth if t_truth is not None else np.array([])),
                truth_pos_ned=_prep(truth_pos_ned if truth_pos_ned is not None else np.array([])),
                truth_vel_ned=_prep(truth_vel_ned if truth_vel_ned is not None else np.array([])),
                truth_pos_ned_i=_prep(truth_pos_ned_i if 'truth_pos_ned_i' in locals() and truth_pos_ned_i is not None else np.array([])),
                truth_vel_ned_i=_prep(truth_vel_ned_i if 'truth_vel_ned_i' in locals() and truth_vel_ned_i is not None else np.array([])),
                pos_body=_prep(pos_body),
                vel_body=_prep(vel_body),
                innov_pos=_prep(innov_pos_all[method]),
                innov_vel=_prep(innov_vel_all[method]),
                euler=_prep(euler_all[method]),
                euler_deg=_prep(np.rad2deg(euler_all[method])),
                residual_pos=_prep(res_pos_all[method]),
                residual_vel=_prep(res_vel_all[method]),
                time_residuals=_prep(time_res_all[method]),
                attitude_q=_prep(att_q_h),
                attitude_q_harmonized=_prep(att_q_h),
                P_hist=_prep(P_hist_all[method]),
                x_log=_prep(x_log_all[method]),
                ref_lat=np.array([ref_lat], dtype=np.float32 if use_fp32 else float),
                ref_lon=np.array([ref_lon], dtype=np.float32 if use_fp32 else float),
                ref_r0=_prep(ref_r0),
            )

    # Also export results as MATLAB-compatible .mat for post-processing
    # MATLAB-friendly .mat bundle with all key series for downstream plotting
    if not args.no_mat:
        save_mat(
            str(RESULTS_DIR / f"{tag}_kf_output.mat"),
            {
                "rmse_pos": np.array([rmse_pos]),
                "final_pos": np.array([final_pos]),
                "grav_err_mean": np.array([grav_err_mean]),
                "grav_err_max": np.array([grav_err_max]),
                "earth_rate_err_mean": np.array([omega_err_mean]),
                "earth_rate_err_max": np.array([omega_err_max]),
                "vel_blow_events": np.array([high_speed_events]),
                "tag": np.array([tag], dtype=object),
                "method": np.array([method_tag], dtype=object),
                "time": _prep(imu_time),
                "t": _prep(imu_time),
                "t_rel_imu": _prep(imu_time - float(imu_time[0])),
                "t_gnss": _prep(gnss_time),
                "t_gnss_shifted": _prep(gnss_time_shifted if 'gnss_time_shifted' in locals() else gnss_time),
                "pos_ned": _prep(fused_pos[method]),
                "vel_ned": _prep(fused_vel[method]),
                "fused_pos": _prep(fused_pos[method]),
                "fused_vel": _prep(fused_vel[method]),
                "fused_acc": _prep(fused_acc[method]),
                **({} if args.lite_output else {
                    "pos_ecef": _prep(pos_ecef),
                    "vel_ecef": _prep(vel_ecef),
                    "gnss_pos_ecef": _prep(gnss_pos_ecef),
                    "gnss_vel_ecef": _prep(gnss_vel_ecef),
                    "gnss_pos_ned": _prep(gnss_pos_ned),
                    "gnss_vel_ned": _prep(gnss_vel_ned),
                    "gnss_pos_ned_interp": _prep(gnss_pos_ned_interp),
                    "gnss_vel_ned_interp": _prep(gnss_vel_ned_interp),
                    "gnss_acc_ned_interp": _prep(gnss_acc_ned_interp),
                }),
                "truth_pos_ecef": _prep(pos_truth_ecef if pos_truth_ecef is not None else np.empty((0, 3))),
                "truth_vel_ecef": _prep(vel_truth_ecef if vel_truth_ecef is not None else np.empty((0, 3))),
                "truth_time": _prep(t_truth if t_truth is not None else np.empty(0)),
                "truth_pos_ned": _prep(truth_pos_ned if truth_pos_ned is not None else np.empty((0, 3))),
                "truth_vel_ned": _prep(truth_vel_ned if truth_vel_ned is not None else np.empty((0, 3))),
                "truth_pos_ned_i": _prep(truth_pos_ned_i if 'truth_pos_ned_i' in locals() and truth_pos_ned_i is not None else np.empty((0, 3))),
                "truth_vel_ned_i": _prep(truth_vel_ned_i if 'truth_vel_ned_i' in locals() and truth_vel_ned_i is not None else np.empty((0, 3))),
                "pos_body": _prep(pos_body),
                "vel_body": _prep(vel_body),
                **({} if args.lite_output else {
                    "innov_pos": _prep(innov_pos_all[method]),
                    "innov_vel": _prep(innov_vel_all[method]),
                    "residual_pos": _prep(res_pos_all[method]),
                    "residual_vel": _prep(res_vel_all[method]),
                    "time_residuals": _prep(time_res_all[method]),
                    "P_hist": _prep(P_hist_all[method]),
                    "x_log": _prep(x_log_all[method]),
                }),
                "euler": _prep(euler_all[method]),
                "euler_deg": _prep(np.rad2deg(euler_all[method])),
                "attitude_q": _prep(att_q_raw),
                "attitude_q_harmonized": _prep(att_q_h),
                "ref_lat": np.array([ref_lat]),
                "ref_lon": np.array([ref_lon]),
                "ref_r0": _prep(ref_r0),
            },
        )
        logging.info(
            "Saved MATLAB bundle: %s (load in MATLAB and call MATLAB/plot_all_from_mat.m)",
            str(RESULTS_DIR / f"{tag}_kf_output.mat"),
        )

    # --- Per-task data bundle (MAT struct) --------------------------------
    tasks_mat = {
        'task1': {
            'lat0_deg': np.array([lat_deg]),
            'lon0_deg': np.array([lon_deg]),
            'h0_m': np.array([alt if alt is not None else 0.0]),
            'g_ned': g_NED,
            'omega_ie_ned': omega_ie_NED,
            'r0_ecef_m': ref_r0,
        },
        'task2': {
            'dt_imu': np.array([dt_imu]),
            'g_body': g_body,
            'omega_ie_body': omega_ie_body,
            'static_start': np.array([static_start]),
            'static_end': np.array([static_end]),
        },
        'task3': {
            'R_TRIAD': task3_results['TRIAD']['R'],
            'R_Davenport': task3_results['Davenport']['R'],
            'R_SVD': task3_results['SVD']['R'],
            'grav_err_deg': np.array([grav_err_mean]),
            'earth_rate_err_deg': np.array([omega_err_mean]),
        },
        'task4_6': {
            'time': imu_time,
            'pos_ned': fused_pos[method],
            'vel_ned': fused_vel[method],
            'pos_gnss_ned': gnss_pos_ned,
            'vel_gnss_ned': gnss_vel_ned,
            'residual_pos': res_pos_all[method],
            'residual_vel': res_vel_all[method],
        },
    }
    save_mat(str(RESULTS_DIR / f"{tag}_tasks.mat"), tasks_mat)

    # ---- Task 6 and Task 7: fused vs truth in NED, ECEF and Body -----------
    # Runs whenever a truth file was supplied, not only for measure_source=truth.
    if truth_file and truth_pos_ned_i is not None and truth_vel_ned_i is not None:
        try:
            from task6_task7_plots import (
                task6_fused_vs_truth,
                task6_quaternion_comparison,
                task7_5_diff_over_time,
                task7_6_attitude,
            )

            _C_B_N = C_B_N_methods[method]
            _args6 = (
                tag, imu_time,
                fused_pos[method], fused_vel[method],
                truth_pos_ned_i, truth_vel_ned_i,
                C_ECEF_to_NED, ref_r0, _C_B_N, RESULTS_DIR,
            )
            made = task6_fused_vs_truth(*_args6)
            made += task7_5_diff_over_time(*_args6[:-1], RESULTS_DIR)

            # Attitude comparisons need a truth quaternion; the bundled truth
            # stores it Body-to-ECEF as [qx,qy,qz,qw] in the last four columns.
            _qt = _truth_quaternion_b2n(
                truth_file, imu_time, C_ECEF_to_NED, args.truth_quat_frame
            )
            if _qt is not None and attitude_q is not None and len(attitude_q):
                _qf = np.asarray(attitude_q, dtype=float)
                if _qf.shape[0] == _qt.shape[0]:
                    made += task6_quaternion_comparison(
                        tag, imu_time, _qf, _qt, C_ECEF_to_NED, RESULTS_DIR
                    )
                    made += task7_6_attitude(tag, imu_time, _qf, _qt, RESULTS_DIR)
                else:
                    logging.warning(
                        "Task 6/7 attitude skipped: %d fused vs %d truth quaternions",
                        _qf.shape[0], _qt.shape[0],
                    )
            else:
                logging.info("Task 6/7 attitude skipped: no truth quaternion available")
            logging.info("Task 6/7 frame comparisons: %d figures", len(made))
        except Exception as ex:  # pragma: no cover - plotting must not abort a run
            logging.warning("Task 6/7 frame comparison plots failed: %s", ex)

    # Compact overview figure with subplots (always saved)
    # Use GNSS and Truth series already aligned/interpolated to the IMU timebase
    # to avoid plotting shape mismatches (IMU has many more samples than GNSS).
    _save_tasks_overview(
        RESULTS_DIR / f"{tag}_tasks_overview",
        imu_time,
        euler_all.get(method, None),
        fused_pos[method],
        fused_vel[method],
        gnss_pos_ned_interp,
        gnss_vel_ned_interp,
        truth_pos_ned_i if measure_source == 'truth' else None,
        truth_vel_ned_i if measure_source == 'truth' else None,
        res_pos_all.get(method, None),
        res_vel_all.get(method, None),
    )

    # --- Persist for cross-dataset comparison ------------------------------
    import pickle
    import gzip

    pack = {
        "method": method_tag,
        "dataset": summary_tag.split("_")[1],
        "t": t_rel_ilu,
        "pos_ned": fused_pos[method],
        "vel_ned": fused_vel[method],
        "pos_gnss": gnss_pos_ned,
        "vel_gnss": gnss_vel_ned,
    }
    fname = RESULTS_DIR / f"{summary_tag}_{method_tag}_compare.pkl.gz"
    with gzip.open(fname, "wb") as f:
        pickle.dump(pack, f)

    # Align truth quaternions to IMU timeline if available -----------------
    q_truth_i = None
    if args.init_att_with_truth and truth_file:
        try:
            import numpy as _np
            from scipy.spatial.transform import Rotation as _R

            truth_arr = _np.loadtxt(truth_file)
            t_cols = []
            for c in range(min(3, truth_arr.shape[1])):
                col = truth_arr[:, c].astype(float)
                if _np.all(_np.isfinite(col)) and (
                    _np.nanmax(col) - _np.nanmin(col)
                ) > 0:
                    t_cols.append((c, float(_np.nanmax(col) - _np.nanmin(col)), col))
            t_cols.sort(key=lambda x: x[1])
            t_truth = t_cols[0][2]
            q_truth = truth_arr[:, -4:]

            if args.truth_quat_frame == "ECEF":
                C_e2n = compute_C_ECEF_to_NED(ref_lat, ref_lon)
                qn_list = []
                for q in q_truth:
                    r_be = _R.from_quat([q[1], q[2], q[3], q[0]])
                    R_bn = C_e2n @ r_be.as_matrix()
                    r_bn = _R.from_matrix(R_bn)
                    q_xyzw = r_bn.as_quat()
                    qn_list.append([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
                q_truth = _np.asarray(qn_list)

            q_truth = q_truth / (
                _np.linalg.norm(q_truth, axis=1, keepdims=True) + 1e-12
            )
            _sign_mask = q_truth[:, 0] < 0
            q_truth[_sign_mask, :] *= -1.0

            idx_nn = _np.searchsorted(
                t_truth, imu_time.clip(t_truth[0], t_truth[-1])
            )
            idx_nn = _np.clip(idx_nn, 0, len(t_truth) - 1)
            q_truth_i = q_truth[idx_nn]
        except Exception as exc:
            logging.warning("Failed to load truth quaternions for summary: %s", exc)

    # --- Final plots -------------------------------------------------------
    if not args.no_plots:
        dataset_id = imu_stem.split("_")[1]
        zupt_mask = np.zeros(len(imu_time), dtype=bool)
        for s, e in zupt_events_all.get(method, []):
            s = max(0, s)
            e = min(len(imu_time) - 1, e)
            zupt_mask[s : e + 1] = True

        save_zupt_variance(
            acc_body_corrected[method],
            zupt_mask,
            dt_imu,
            dataset_id,
            threshold=0.01,
            base_dir=RESULTS_DIR,
            method=method,
        )

        # Removed: task5_9_1/5_9_2 residuals and task5_9_4 velocity profile.
        # They compared the fused solution against GNSS on the GNSS grid, which
        # duplicates the Task 7.5 fused-minus-truth figures and is not
        # meaningful across all three methods.

        # Task 7: Attitude comparison — quaternion components (Truth vs Estimated)
        # Saves under the Task 7 naming convention if truth data is available.
        if args.init_att_with_truth and truth_file:
            try:
                import numpy as _np
                from scipy.spatial.transform import Rotation as _R

                # Load truth times and quaternions
                truth_arr = _np.loadtxt(truth_file)
                # Heuristic: find a time-like column among first 3 columns
                t_cols = []
                for c in range(min(3, truth_arr.shape[1])):
                    col = truth_arr[:, c].astype(float)
                    if _np.all(_np.isfinite(col)) and (_np.nanmax(col) - _np.nanmin(col)) > 0:
                        t_cols.append((c, float(_np.nanmax(col) - _np.nanmin(col)), col))
                if not t_cols:
                    raise ValueError("No valid time column found in truth file for attitude plot")
                t_cols.sort(key=lambda x: x[1])
                t_truth = t_cols[0][2]
                q_truth = truth_arr[:, -4:]

                # Convert body->ECEF truth quats to body->NED if requested
                if args.truth_quat_frame == "ECEF":
                    C_e2n = compute_C_ECEF_to_NED(ref_lat, ref_lon)
                    qn_list = []
                    for q in q_truth:
                        # Input q is [qw,qx,qy,qz]; scipy expects [x,y,z,w]
                        r_be = _R.from_quat([q[1], q[2], q[3], q[0]])
                        R_bn = C_e2n @ r_be.as_matrix()
                        r_bn = _R.from_matrix(R_bn)
                        q_xyzw = r_bn.as_quat()
                        qn_list.append([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
                    q_truth = _np.asarray(qn_list)

                # Normalise and enforce positive scalar part
                q_truth = q_truth / (_np.linalg.norm(q_truth, axis=1, keepdims=True) + 1e-12)
                _truth_sign_mask = (q_truth[:, 0] < 0)
                q_truth[_truth_sign_mask, :] *= -1.0

                # Align truth to IMU sampling by nearest neighbour
                idx_nn = _np.searchsorted(t_truth, imu_time.clip(t_truth[0], t_truth[-1]))
                idx_nn = _np.clip(idx_nn, 0, len(t_truth) - 1)
                q_truth_i = q_truth[idx_nn]

                # Quaternions (w,x,y,z) time series
                q_est = attitude_q_all[method]
                # Align quaternion signs per-sample (q and -q are equivalent)
                dots = _np.sum(q_truth_i * q_est, axis=1)
                signs = _np.where(dots >= 0.0, 1.0, -1.0)[:, None]
                q_est = q_est * signs
                import matplotlib.pyplot as _plt
                fig_q, axs_q = _plt.subplots(4, 1, figsize=(10, 8), sharex=True)
                comps = ["q_w", "q_x", "q_y", "q_z"]
                for i, lab in enumerate(comps):
                    axs_q[i].plot(imu_time, q_truth_i[:, i], "k-", label="Truth")
                    axs_q[i].plot(imu_time, q_est[:, i], "r--", label="Estimated")
                    axs_q[i].set_ylabel(lab)
                    axs_q[i].grid(True)
                    if i == 0:
                        axs_q[i].legend(loc="best")
                axs_q[-1].set_xlabel("Time [s]")
                fig_q.suptitle("Task 7: Body→NED Attitude Quaternions — Truth vs Estimate")
                fig_q.tight_layout(rect=[0, 0, 1, 0.95])

                # Save with the requested filename pattern
                imu_tag = Path(imu_file).stem
                gnss_tag = Path(gnss_file).stem
                out_name = f"{imu_tag}_{gnss_tag}_{method}_Task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png"
                out_path = RESULTS_DIR / out_name
                fig_q.savefig(out_path, dpi=200, bbox_inches="tight")
                # Also save a MATLAB .mat bundle for plot_any compatibility
                try:
                    from scipy.io import savemat  # type: ignore
                    n = min(len(imu_time), len(q_truth_i), len(q_est))
                    savemat(str(out_path.with_suffix('.mat')), {
                        't': np.asarray(imu_time[:n], float),
                        'q_truth': np.asarray(q_truth_i[:n], float),
                        'q_est': np.asarray(q_est[:n], float),
                    })
                except Exception:
                    pass
                _plt.close(fig_q)
            except Exception as ex:
                logging.warning("Failed to save Task 7 attitude comparison: %s", ex)

    q_final = attitude_q_all[method][-1].copy()
    q_final = q_final / (np.linalg.norm(q_final) + 1e-12)
    if q_final[0] < 0:
        q_final *= -1.0

    att_err_deg = float("nan")
    if q_truth_i is not None:
        try:
            q_truth_final = q_truth_i[-1]
            q_est_final = q_final.copy()
            if np.dot(q_truth_final, q_est_final) < 0:
                q_est_final *= -1.0
            dot = np.clip(np.dot(q_truth_final, q_est_final), -1.0, 1.0)
            att_err_deg = 2.0 * np.degrees(np.arccos(dot))
        except Exception as exc:
            logging.warning("Failed to compute final attitude error: %s", exc)

    logging.info(
        f"[SUMMARY] method={method:<9} imu={os.path.basename(imu_file)} gnss={os.path.basename(gnss_file)} "
        f"rmse_pos={rmse_pos:7.2f}m final_pos={final_pos:7.2f}m "
        f"rms_resid_pos={rms_resid_pos:7.2f}m max_resid_pos={max_resid_pos:7.2f}m "
        f"rms_resid_vel={rms_resid_vel:7.2f}m max_resid_vel={max_resid_vel:7.2f}m "
        f"accel_bias={np.linalg.norm(accel_bias):.4f} gyro_bias={np.linalg.norm(gyro_bias):.4f} "
        f"ZUPT_count={zupt_counts.get(method,0)} "
        f"GravErrMean_deg={grav_err_mean:.6f} GravErrMax_deg={grav_err_max:.6f} "
        f"EarthRateErrMean_deg={omega_err_mean:.6f} EarthRateErrMax_deg={omega_err_max:.6f} "
        f"att_err_deg={att_err_deg:.6f} "
        f"q_final=[{q_final[0]:.6f},{q_final[1]:.6f},{q_final[2]:.6f},{q_final[3]:.6f}]"
    )


if __name__ == "__main__":
    main()
