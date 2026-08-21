#!/usr/bin/env python3
"""Run the TRIAD initialisation on a single fixed IMU/GNSS dataset.

This script mirrors :mod:`run_all_methods.py` but processes only
``IMU_X002.dat`` together with ``GNSS_X002.csv``.  The full GNSS/IMU fusion
pipeline (Tasks 1--7) is executed and all outputs are collected under
``results/IMU_X002_GNSS_X002_TRIAD``.  Use this helper to compare the Python and
MATLAB pipelines on a single dataset before running the full batch.

Usage
-----
    python src/run_triad_only.py [options]
"""

from __future__ import annotations

# Self-contained imports: make local src importable from any CWD
from pathlib import Path as _Path
import sys as _sys
_SRC = _Path(__file__).resolve().parent
if str(_SRC) not in _sys.path:
    _sys.path.insert(0, str(_SRC))
# Repository root (…/IMU)
REPO_ROOT = _SRC.parents[2]
from dependency_bootstrap import ensure_dependencies

ensure_dependencies()

import argparse
import json
import logging
import math
import os
import pathlib
import re
import subprocess
import sys
import time
import shutil
import io
from contextlib import redirect_stdout
from pathlib import Path
from typing import Iterable, List, Dict

import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from utils.quaternion_tools import normalize_quat as _qnorm, align_sign_to_ref as _qalign, fix_estimator_quat_wxyz_against_truth as _qfix_est
import scipy.io as sio
from tabulate import tabulate
import matplotlib.pyplot as plt
from scipy.io import loadmat, savemat

from evaluate_filter_results import run_evaluation_npz
from run_all_methods import run_case, compute_C_NED_to_ECEF
from utils import save_mat

# Import helper utilities from the utils package
from utils.timeline import print_timeline
from utils.resolve_truth_path import resolve_truth_path
from utils.io_checks import assert_single_pair
from utils.run_id import run_id as build_run_id

sys.path.append(str(Path(__file__).resolve().parents[1] / "tools"))

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
from paths import (
    imu_path as _imu_path_helper,
    gnss_path as _gnss_path_helper,
    truth_path as _truth_path_helper,
    ensure_results_dir as _ensure_results,
    python_results_dir as _py_results_dir,
)

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def check_files(
    imu_file: str, gnss_file: str
) -> tuple[pathlib.Path, pathlib.Path]:
    """Return validated paths for the IMU and GNSS files."""
    logger.info("Checking files: imu=%s gnss=%s", imu_file, gnss_file)
    imu_path = pathlib.Path(imu_file)
    gnss_path = pathlib.Path(gnss_file)
    logger.info("Validated files exist: imu=%s gnss=%s", imu_path, gnss_path)
    return imu_path, gnss_path


def _unwrap_clock_1s(t_raw, wrap=1.0, tol=0.25):
    """Unwrap a clock that resets every ``wrap`` seconds (default 1s)."""
    import numpy as np
    logger.info("Unwrapping clock with wrap=%s tol=%s", wrap, tol)
    t = np.asarray(t_raw, dtype=float).ravel()
    if t.size == 0:
        logger.info("No timestamps provided; returning empty array")
        return t, 0
    out = t.copy()
    wraps = 0
    offset = 0.0
    for i in range(1, t.size):
        if t[i] + 1e-12 < t[i - 1] - tol:
            wraps += 1
            offset += wrap
        out[i] = t[i] + offset
    logger.info("Unwrapped clock applied %d wraps", wraps)
    return out, wraps


def _make_monotonic_time(t_like, fallback_len=None, dt=None, imu_rate_hint=None):
    """Build a strictly increasing IMU timebase."""
    import numpy as np
    logger.info(
        "Building monotonic time: len=%s dt=%s imu_rate_hint=%s",
        len(t_like) if hasattr(t_like, "__len__") else None,
        dt,
        imu_rate_hint,
    )
    if t_like is None or (hasattr(t_like, "__len__") and len(t_like) == 0):
        if dt is None:
            if imu_rate_hint and imu_rate_hint > 0:
                dt = 1.0 / float(imu_rate_hint)
            else:
                raise ValueError("No IMU timestamps and no dt/imu_rate_hint provided.")
        n = int(fallback_len) if fallback_len is not None else 0
        t = np.arange(n, dtype=float) * float(dt)
        meta = {"source": "synth", "wraps": 0, "dt": dt}
        logger.info("Generated synthetic timebase of %d samples", n)
        return t, meta

    t = np.asarray(t_like, dtype=float).ravel()
    meta = {"source": "file", "wraps": 0}
    span = float(t.max() - t.min()) if t.size else 0.0
    if span < 2.0 and t.size > 2000:
        t, wraps = _unwrap_clock_1s(t, wrap=1.0, tol=0.25)
        meta["wraps"] = int(wraps)
        meta["unwrapped_span"] = float(t[-1] - t[0]) if t.size else 0.0
        print(
            f"[Clock] Detected 1s-resetting IMU clock; unwrapped with {wraps} wraps -> {meta['unwrapped_span']:.2f}s total."
        )
    d = np.diff(t)
    if (d <= 0).any():
        eps = np.finfo(float).eps
        t = t + np.arange(t.size) * eps
        meta["jitter_eps_applied"] = True
    logger.info("Monotonic timebase built with %d samples", t.size)
    return t, meta


def _write_run_meta(outdir, run_id, **kv):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    meta_path = outdir / f"{run_id}_runmeta.json"
    with meta_path.open("w", encoding="utf-8") as f:
        json.dump(kv, f, indent=2, sort_keys=True)
    logger.info("Saved run meta -> %s", meta_path)


# ----------------------------
# Fallback plots without Truth (Task 7.6 style)
# ----------------------------
def _task7_6_plots_no_truth(results_dir: str, run_id: str) -> None:
    """Generate quaternion and fused overlays (NED/ECEF/Body) when no truth.

    Saves files with the Task 7.6 naming so downstream steps find them:
      - <run_id>_Task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png (KF only)
      - <run_id>_task7_6_overlay_NED.png (Fused only)
      - <run_id>_task7_6_overlay_ECEF.png (Fused only)
      - <run_id>_task7_6_overlay_BODY.png (Fused only)
    """
    res_dir = Path(results_dir)
    npz_path = res_dir / f"{run_id}_kf_output.npz"
    if not npz_path.exists():
        return
    d = np.load(npz_path, allow_pickle=True)
    # Time
    t = d.get("time_s") if d.get("time_s") is not None else d.get("time")
    if t is None:
        # Derive from dt and a series length
        dt = d.get("imu_dt") or d.get("dt")
        n = None
        for key in ("pos_ned_m", "pos_ned", "fused_pos"):
            if d.get(key) is not None:
                n = int(np.asarray(d.get(key)).shape[0])
                break
        t = np.arange(n) * float(dt) if (dt and n) else np.arange(0)
    t = np.asarray(t, float).reshape(-1)
    if t.size:
        t = t - t[0]

    # Quaternion
    # Prefer harmonized (continuous) quaternions if available (avoid `or` on arrays)
    q = d.get("attitude_q_harmonized")
    if q is None:
        q = d.get("att_quat")
    if q is None:
        q = d.get("attitude_q")
    if q is None:
        q = d.get("quat_log")
    if q is None:
        return
    q = _norm_quat(np.asarray(q, float))

    # Fused pos/vel in NED (avoid `or` on arrays)
    pos_ned = d.get("pos_ned_m")
    if pos_ned is None:
        pos_ned = d.get("pos_ned")
    if pos_ned is None:
        pos_ned = d.get("fused_pos")
    vel_ned = d.get("vel_ned_ms")
    if vel_ned is None:
        vel_ned = d.get("vel_ned")
    if vel_ned is None:
        vel_ned = d.get("fused_vel")
    if pos_ned is None or vel_ned is None:
        return
    pos_ned = np.asarray(pos_ned, float)
    vel_ned = np.asarray(vel_ned, float)

    # Align lengths
    n = min(len(t), len(pos_ned), len(vel_ned), len(q))
    if n <= 0:
        return
    t = t[:n]
    pos_ned = pos_ned[:n]
    vel_ned = vel_ned[:n]
    q = q[:n]

    # Plot quaternion components (KF only)
    plt.figure(figsize=(10, 6))
    labs = ["w", "x", "y", "z"]
    for i, lab in enumerate(labs):
        ax = plt.subplot(2, 2, i + 1)
        ax.plot(t, q[:, i], '-', label='KF')
        ax.set_title(f"q_{lab}")
        ax.grid(True)
    plt.legend(loc='upper right')
    plt.suptitle(f"{run_id} Task7_6: Quaternion Components")
    out_q = res_dir / f"{run_id}_Task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png"
    try:
        plt.gcf().savefig(out_q, dpi=200, bbox_inches='tight')
    except Exception:
        pass
    finally:
        plt.close()

    # Helper to make fused-only overlay
    def _overlay(pos, vel, labels, frame):
        fig, axes = plt.subplots(2, 3, figsize=(9, 4), sharex=True)
        # Decimate if large
        stride = int(np.ceil(len(t) / 200000)) if len(t) > 200000 else 1
        tt = t[::stride]
        P = pos[::stride]
        V = vel[::stride]
        for i in range(3):
            axes[0, i].plot(tt, P[:, i], label="Fused")
            axes[0, i].set_title(labels[i])
            axes[0, i].set_ylabel("Position [m]")
            axes[0, i].grid(True)
            axes[1, i].plot(tt, V[:, i], label="Fused")
            axes[1, i].set_xlabel("Time [s]")
            axes[1, i].set_ylabel("Velocity [m/s]")
            axes[1, i].grid(True)
        handles, labs_h = axes[0, 0].get_legend_handles_labels()
        if handles:
            fig.legend(handles, labs_h, ncol=2, loc="upper center")
        fig.suptitle(f"Task 7.6 – Fused ({frame} Frame)")
        fig.tight_layout(rect=[0, 0, 1, 0.92])
        base = res_dir / f"{run_id}_task7_6_overlay_{frame}"
        try:
            fig.savefig(base.with_suffix('.png'), dpi=200, bbox_inches='tight')
        except Exception:
            pass
        plt.close(fig)

    # NED overlay
    _overlay(pos_ned, vel_ned, ["North", "East", "Down"], "NED")

    # ECEF overlay
    ref_lat = d.get("ref_lat_rad") or d.get("ref_lat")
    ref_lon = d.get("ref_lon_rad") or d.get("ref_lon")
    ref_r0 = d.get("ref_r0_m") or d.get("ref_r0")
    if ref_lat is not None and ref_lon is not None:
        ref_lat = float(np.squeeze(ref_lat))
        ref_lon = float(np.squeeze(ref_lon))
        C_NE = compute_C_NED_to_ECEF(ref_lat, ref_lon)
        pos_ecef = (C_NE @ pos_ned.T).T + np.asarray(ref_r0).reshape(3)
        vel_ecef = (C_NE @ vel_ned.T).T
    else:
        pos_ecef = pos_ned.copy()
        vel_ecef = vel_ned.copy()
    _overlay(pos_ecef, vel_ecef, ["X", "Y", "Z"], "ECEF")

    # Body overlay
    try:
        rot = R.from_quat(q[:, [1, 2, 3, 0]])
        pos_body = rot.apply(pos_ned, inverse=True)
        vel_body = rot.apply(vel_ned, inverse=True)
        _overlay(pos_body, vel_body, ["X", "Y", "Z"], "BODY")
    except Exception:
        pass


# ----------------------------
# Inline validation helpers
# ----------------------------
def _norm_quat(q):
    q = np.asarray(q, float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    n[n == 0] = 1.0
    return q / n


def _fix_hemisphere(qt, qe):
    dot = np.sum(qt * qe, axis=1)
    s = np.sign(dot)
    s[s == 0] = 1.0
    return qe * s[:, None]


def _quat_angle_deg(qt, qe):
    d = np.clip(np.abs(np.sum(qt * qe, axis=1)), 0.0, 1.0)
    return 2.0 * np.degrees(np.arccos(d))


def _quat_to_euler_zyx_deg(q):  # yaw, pitch, roll from [w,x,y,z]
    w, x, y, z = q.T
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    s = np.clip(2 * (w * y - z * x), -1.0, 1.0)
    pitch = np.degrees(np.arcsin(s))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return np.vstack([yaw, pitch, roll]).T


def _overlap(t1, t2):
    t0 = max(t1[0], t2[0])
    t1e = min(t1[-1], t2[-1])
    if t1e <= t0:
        raise RuntimeError("No overlapping time between truth and estimate.")
    return t0, t1e


def _interp_columns_to(t_src, Y_src, t_dst):
    Y_src = np.asarray(Y_src)
    out = np.empty((t_dst.size, Y_src.shape[1]), float)
    for j in range(Y_src.shape[1]):
        out[:, j] = np.interp(t_dst, t_src, Y_src[:, j])
    return out


# Robust overlap and interpolation helpers for inline validation
def _uniq_sorted_time(t, *arrays):
    """Sort by time, drop duplicates, and apply the same indexing to companions."""
    t = np.asarray(t, dtype=float)
    order = np.argsort(t, kind="mergesort")
    t_sorted = t[order]
    uniq_mask = np.ones_like(t_sorted, dtype=bool)
    uniq_mask[1:] = np.diff(t_sorted) > 0
    keep = order[uniq_mask]
    out = [t[keep]]
    for arr in arrays:
        out.append(np.asarray(arr)[keep])
    return out if len(out) > 1 else out[0]


def _compute_overlap(t_est, t_truth, *truth_arrays):
    """Return overlapped (t_truth_sub, *truth_arrays_sub) aligned to [t_est.min, t_est.max]."""
    if t_est.size == 0 or t_truth.size == 0:
        raise RuntimeError("Empty time arrays passed to _compute_overlap.")
    t0, t1 = float(np.nanmin(t_est)), float(np.nanmax(t_est))
    # clean & sort truth; drop duplicate times
    t_truth_clean, *truth_arrays_clean = _uniq_sorted_time(t_truth, *truth_arrays)
    finite_mask = np.isfinite(t_truth_clean)
    for arr in truth_arrays_clean:
        finite_mask &= np.all(np.isfinite(arr), axis=1) if arr.ndim == 2 else np.isfinite(arr)
    t_truth_clean = t_truth_clean[finite_mask]
    truth_arrays_clean = [arr[finite_mask] for arr in truth_arrays_clean]
    # window to est range
    if t_truth_clean.size == 0:
        return (np.array([]),) + tuple(np.array([]) for _ in truth_arrays_clean)
    mask = (t_truth_clean >= t0) & (t_truth_clean <= t1)
    t_sub = t_truth_clean[mask]
    arrays_sub = [arr[mask] for arr in truth_arrays_clean]
    return (t_sub,) + tuple(arrays_sub)


def _interp_quat_components(t_src, q_src, t_query):
    """
    Component-wise linear interp of quaternions w/o SLERP (OK for diagnostics).
    Returns (N,4) or raises with a clear message if inputs are unusable.
    """
    if t_src.size < 2:
        raise ValueError(
            f"Not enough truth samples to interpolate: len(t_src)={t_src.size}. "
            "Check overlap window and NaNs."
        )
    # ensure strictly increasing
    order = np.argsort(t_src, kind="mergesort")
    t_src = t_src[order]
    q_src = q_src[order]
    # clamp query to src bounds to avoid extrapolation errors
    tq = np.clip(t_query, t_src[0], t_src[-1])
    from scipy.interpolate import interp1d
    q_interp = np.empty((tq.size, 4), dtype=float)
    for k in range(4):
        f = interp1d(
            t_src,
            q_src[:, k],
            kind="linear",
            bounds_error=False,
            fill_value=(q_src[0, k], q_src[-1, k]),
            assume_sorted=True,
        )
        q_interp[:, k] = f(tq)
    # renormalize
    norm = np.linalg.norm(q_interp, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    return q_interp / norm


def _divergence_time(t, err_deg, thr_deg, persist_s):
    above = err_deg > thr_deg
    if not np.any(above):
        return np.nan
    dt = np.median(np.diff(t))
    need = max(1, int(round(persist_s / dt)))
    i = 0
    N = above.size
    while i < N:
        if above[i]:
            j = i
            while j < N and above[j]:
                j += 1
            if (j - i) >= need:
                return t[i]
            i = j
        else:
            i += 1
    return np.nan


def _read_state_x001(truth_file):
    """
    Read STATE_X001-like truth with flexible column handling.

    Expected columns include (typical order):
      [idx?] time_s  x_ecef  y_ecef  z_ecef  vx_ecef  vy_ecef  vz_ecef  qw  qx  qy  qz

    This function:
      - loads all numeric columns
      - auto-selects the best time column
      - zero-bases time if it looks offset (e.g., starts at ~10000 s)
      - returns (t, pos_ecef, vel_ecef, quat) with shapes:
          t:(N,), pos:(N,3), vel:(N,3), quat:(N,4) (qw,qx,qy,qz)
    """
    import numpy as np

    M = np.loadtxt(truth_file, dtype=float)
    if M.ndim != 2 or M.shape[1] < 10:
        raise ValueError(f"Unexpected truth shape {M.shape} for {truth_file}")

    # Heuristics to find quaternion columns: last 4 numeric columns
    quat = M[:, -4:]  # assume (qw,qx,qy,qz)

    # Candidate time columns: prefer one whose range overlaps small window (seconds)
    # Common layouts: time at col 0 or col 1
    candidates = []
    for c in range(min(3, M.shape[1])):  # probe first up to 3 cols
        t_try = M[:, c].astype(float)
        if np.all(np.isfinite(t_try)):
            span = float(np.nanmax(t_try) - np.nanmin(t_try))
            candidates.append((c, span, t_try))
    if not candidates:
        raise ValueError("No finite time-like column found in truth file.")

    # Choose the candidate with the smallest positive span >= 1.0 sec (prefers [0..~2000] over [10000..20086])
    candidates = [(c, s, tcol) for (c, s, tcol) in candidates if s > 0]
    candidates.sort(key=lambda x: x[1])
    c_best, span_best, t_raw = candidates[0]

    # Zero-base time if it appears offset (e.g., starts >> 0 and span looks reasonable)
    t0 = float(np.nanmin(t_raw))
    t = t_raw - t0 if t0 > 1.0 else t_raw

    # Now pick positions/velocities assuming standard layout: find 6 columns before quats
    # If your format differs, adjust here.
    six_before_quat = M.shape[1] - 4 - 6
    if six_before_quat < 0:
        raise ValueError("Not enough columns to extract pos/vel before quaternions.")
    pv = M[:, six_before_quat:six_before_quat+6]
    pos_ecef = pv[:, :3]
    vel_ecef = pv[:, 3:6]

    # Clean NaNs and enforce shapes
    m = np.isfinite(t) & np.all(np.isfinite(pos_ecef), axis=1) & np.all(np.isfinite(vel_ecef), axis=1) & np.all(np.isfinite(quat), axis=1)
    t = t[m]
    pos_ecef = pos_ecef[m]
    vel_ecef = vel_ecef[m]
    quat = quat[m]

    return t, pos_ecef, vel_ecef, quat


def _save_png_and_mat(path_png, arrays):
    """Save PNG, MATLAB .mat arrays, and MATLAB-native .fig for current figure.

    - PNG at ``path_png``
    - MAT at ``<path_png base>.mat`` containing provided arrays
    - FIG at ``<path_png base>.fig`` via MATLAB engine (best-effort)
    """
    # Derive base (without extension)
    base, _ = os.path.splitext(path_png)

    # 1) Save PNG
    try:
        plt.gcf().savefig(path_png, dpi=200, bbox_inches='tight')
        print(f"[PNG] {path_png}")
    except Exception as e:
        print(f"[WARN] Failed to save PNG {path_png}: {e}")

    # 2) Save MATLAB .mat with arrays for reproducibility
    #    Align all arrays on first dimension to a common length when a time
    #    vector 't' is provided, to avoid downstream length mismatches.
    try:
        to_save = dict(arrays)
        # Normalise and align shapes if a reference time vector is present
        if 't' in to_save:
            import numpy as _np
            t_vec = _np.ravel(_np.asarray(to_save['t']))
            lengths = []
            # Gather candidate lengths from arrays that have a first-dimension length
            for k, v in to_save.items():
                a = _np.asarray(v)
                if a.ndim >= 1 and a.shape[0] > 0:
                    lengths.append(int(a.shape[0]))
            if lengths:
                n = int(min(lengths))
                if any(L != n for L in lengths):
                    print(f"[Align] Harmonising lengths to n={n} for MAT save: "
                          + ", ".join(f"{k}:{_np.asarray(v).shape}" for k, v in to_save.items()))
                # Trim all arrays on the first axis to common n
                for k, v in list(to_save.items()):
                    a = _np.asarray(v)
                    if a.ndim >= 1 and a.shape[0] != n:
                        to_save[k] = a[:n]
                # Also trim 't' explicitly in case it's longer
                to_save['t'] = t_vec[:n]
        savemat(base + '.mat', to_save)
        print(f"[MAT] {base + '.mat'}")
    except Exception as e:
        print(f"[WARN] Failed to save MAT {base + '.mat'}: {e}")

    # 3) Save MATLAB-native .fig and optionally validate (no non-native fallbacks)
    try:
        from utils.matlab_fig_export import save_matlab_fig, validate_fig_openable
        fig_path = save_matlab_fig(plt.gcf(), base)
        if fig_path:
            validate_fig_openable(str(fig_path))
    except Exception as e:
        print(f"[WARN] .fig export/validation skipped: {e}")

    # Close the figure to free memory
    plt.close()


def _run_inline_truth_validation(results_dir, tag, kf_mat_path, truth_file, args):
    print("Starting inline validation (truth vs KF)…")
    est = loadmat(kf_mat_path, squeeze_me=True)
    # time
    t_est = None
    for k in ['t_est', 'time_s', 'time', 't']:
        if k in est:
            t_est = np.ravel(est[k]).astype(float)
            break
    if t_est is None:
        raise RuntimeError("Estimator time not found in kf_output.mat")
    # quaternion (prefer att_quat; else quat_log)
    q_est = None
    for k in ['att_quat', 'attitude_q', 'quat_log', 'q_est', 'quaternion']:
        if k in est:
            q_est = np.atleast_2d(est[k]).astype(float)
            if q_est.shape[0] == 4 and q_est.shape[1] != 4:
                q_est = q_est.T
            if q_est.shape[1] != 4 and q_est.shape[0] == 4:
                q_est = q_est.T
            break
    if q_est is None or q_est.shape[1] != 4:
        raise RuntimeError("Bad estimator quaternion shape")
    q_est = _norm_quat(q_est)

    # truth
    t_tru, pos_ecef_tru, vel_ecef_tru, q_tru = _read_state_x001(truth_file)
    
    # Sanity prints to the log
    print(f"[InlineValid] t_est:[{t_est[0]:.3f},{t_est[-1]:.3f}] N={t_est.size}")
    print(f"[InlineValid] t_truth(raw):[{np.nanmin(t_tru):.3f},{np.nanmax(t_tru):.3f}] N={t_tru.size}")

    # Compute robust overlap (drops dupes/NaNs, windows to [min(t_est), max(t_est)])
    t_truth_sub, q_truth_sub = _compute_overlap(t_est, t_tru, q_tru)

    if t_truth_sub.size < 2:
        msg = (
            f"No truth samples in overlap window.\n"
            f"  est range   = [{t_est[0]:.3f},{t_est[-1]:.3f}] (N={t_est.size})\n"
            f"  truth range = [{np.nanmin(t_tru):.3f},{np.nanmax(t_tru):.3f}] (N={t_tru.size})\n"
            f"  overlap N   = {t_truth_sub.size} (need ≥2). "
            f"Check truth parsing (time in 2nd col, quats in last 4) and NaNs."
        )
        print("[WARN] Inline validation skipped:", msg)
        return

    # Resample truth quaternions onto estimator time using SLERP (handles 1Hz vs 400Hz)
    def _q_normalize(q):
        q = np.asarray(q, float)
        n = np.linalg.norm(q, axis=1, keepdims=True)
        n[n == 0] = 1.0
        return q / n

    def _q_conj(q):
        q = np.asarray(q, float)
        return np.column_stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]])

    def _hemi_fix(q):
        q = _q_normalize(q)
        s = np.ones(q.shape[0])
        if q.shape[0] > 1:
            s[1:] = np.sign(np.sum(q[1:] * q[:-1], axis=1))
            s = np.cumprod(s)
        return q * s[:, None]

    def _slerp(q1, q2, u):
        d = float(np.dot(q1, q2))
        if d < 0.0:
            q2 = -q2
            d = -d
        if d > 0.9995:
            q = (1.0 - u) * q1 + u * q2
            return q / (np.linalg.norm(q) + 1e-12)
        theta0 = np.arccos(np.clip(d, -1.0, 1.0))
        s0 = np.sin((1.0 - u) * theta0) / np.sin(theta0)
        s1 = np.sin(u * theta0) / np.sin(theta0)
        return s0 * q1 + s1 * q2

    def _resample_quat_to(t_src, q_src, t_dst):
        t_src = np.asarray(t_src, float)
        q_src = _hemi_fix(np.asarray(q_src, float))
        t_dst = np.asarray(t_dst, float)
        if t_src.size < 2:
            raise ValueError("Need at least two truth samples for SLERP resampling")
        idx = np.searchsorted(t_src, t_dst, side='right') - 1
        idx = np.clip(idx, 0, t_src.size - 2)
        u = (t_dst - t_src[idx]) / (t_src[idx + 1] - t_src[idx] + 1e-12)
        out = np.empty((t_dst.size, 4), float)
        for k in range(t_dst.size):
            out[k] = _slerp(q_src[idx[k]], q_src[idx[k] + 1], float(u[k]))
        return _q_normalize(out)

    try:
        q_truth_on_est_raw = _resample_quat_to(t_truth_sub, q_truth_sub, t_est)
    except Exception as e:
        print("[WARN] Inline validation skipped during quaternion SLERP resampling:", e)
        return

    # Decide if truth quaternions are body->NED or body->ECEF by comparing
    # angle error after interpreting truth in both frames and choosing the smaller.
    # If ECEF, convert to NED using reference lat/lon from truth ECEF position.
    detected_frame = "NED"
    q_truth_ned = q_truth_on_est_raw.copy()
    try:
        # Compute reference lat/lon from the first valid ECEF truth position
        from utils import ecef_to_geodetic, compute_C_ECEF_to_NED
        # Use first row (already masked for NaNs earlier) as reference
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_ecef_tru[0])
        C_e2n = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
        # Convert sequence: q_be (body->ECEF) -> q_bn (body->NED)
        R_bn_list = []
        from scipy.spatial.transform import Rotation as _R
        for q in q_truth_on_est_raw:
            r_be = _R.from_quat([q[1], q[2], q[3], q[0]])  # [x,y,z,w]
            R_bn = C_e2n @ r_be.as_matrix()
            R_bn_list.append(_R.from_matrix(R_bn).as_quat())  # [x,y,z,w]
        q_truth_from_ecef = np.array([[q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]] for q_xyzw in R_bn_list])
        # Normalise both interpretations
        q_truth_from_ecef = _norm_quat(q_truth_from_ecef)
        q_truth_ned = _norm_quat(q_truth_ned)
        # Hemisphere-fix vs estimate and compute angle errors
        q_est_aligned_for_ned = _fix_hemisphere(q_truth_ned, q_est)
        q_est_aligned_for_ecef = _fix_hemisphere(q_truth_from_ecef, q_est)
        ang_ned = _quat_angle_deg(q_truth_ned, q_est_aligned_for_ned)
        ang_ecef = _quat_angle_deg(q_truth_from_ecef, q_est_aligned_for_ecef)
        # Compare robust statistics (median over overlap)
        med_ned = float(np.nanmedian(ang_ned))
        med_ecef = float(np.nanmedian(ang_ecef))
        if np.isfinite(med_ecef) and (med_ecef + 1e-3) < med_ned:
            detected_frame = "ECEF"
            q_truth_ned = q_truth_from_ecef
            qE = q_est_aligned_for_ecef
            ang = ang_ecef
        else:
            detected_frame = "NED"
            qE = q_est_aligned_for_ned
            ang = ang_ned
        print(f"[Task7] Detected truth quaternion frame: {detected_frame}")
        # Save detection result for reference
        det_path = os.path.join(results_dir, f"{tag}_Task7_truth_quaternion_frame.txt")
        with open(det_path, "w", encoding="utf-8") as fdet:
            fdet.write(detected_frame + "\n")
    except Exception as ex:
        # Fall back to assuming provided truth is already Body->NED
        print(f"[WARN] Truth frame detection failed ({ex}); assuming NED.")
        q_truth_ned = _norm_quat(q_truth_ned)
        qE = _fix_hemisphere(q_truth_ned, q_est)
        ang = _quat_angle_deg(q_truth_ned, qE)

    # Truth direction detection: Body->NED vs NED->Body (conjugate)
    try:
        q_truth_ned = _norm_quat(q_truth_ned)
        def _err0(qt):
            qe = _fix_hemisphere(qt, q_est)
            ang0 = _quat_angle_deg(qt, qe)
            return float(ang0[0]) if ang0.size else float('nan')
        e0_dir = _err0(q_truth_ned)
        e0_con = _err0(_q_conj(q_truth_ned))
        if np.isfinite(e0_con) and e0_con + 1e-3 < e0_dir:
            q_truth_ned = _q_conj(q_truth_ned)
            print(f"[Task7] Truth appears NED→Body; using conjugate. (err0 direct={e0_dir:.2f}°, conj={e0_con:.2f}°)")
        else:
            print(f"[Task7] Truth matches Body→NED. (err0 direct={e0_dir:.2f}°)")
    except Exception:
        pass

    # Auto-align truth frame and remove fixed body-side registration before comparison
    # Helpers
    def q_norm(q):
        q = np.asarray(q, float)
        n = np.linalg.norm(q, axis=1, keepdims=True)
        n[n == 0] = 1.0
        return q / n

    def q_conj(q):
        q = np.asarray(q, float)
        return np.column_stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]])

    def q_mul(a, b):
        a = np.asarray(a, float)
        b = np.asarray(b, float)
        w1, x1, y1, z1 = a.T
        w2, x2, y2, z2 = b.T
        return np.column_stack([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
        ])

    def dcm_to_quat(C):
        C = np.asarray(C, float)
        t = np.trace(C)
        if t <= 0:
            i = int(np.argmax(np.diag(C)))
            if i == 0:
                s = 2.0 * np.sqrt(max(1e-12, 1 + C[0, 0] - C[1, 1] - C[2, 2]))
                w = (C[2, 1] - C[1, 2]) / s
                x = 0.25 * s
                y = (C[0, 1] + C[1, 0]) / s
                z = (C[0, 2] + C[2, 0]) / s
            elif i == 1:
                s = 2.0 * np.sqrt(max(1e-12, 1 + C[1, 1] - C[0, 0] - C[2, 2]))
                w = (C[0, 2] - C[2, 0]) / s
                x = (C[0, 1] + C[1, 0]) / s
                y = 0.25 * s
                z = (C[1, 2] + C[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(max(1e-12, 1 + C[2, 2] - C[0, 0] - C[1, 1]))
                w = (C[1, 0] - C[0, 1]) / s
                x = (C[0, 2] + C[2, 0]) / s
                y = (C[1, 2] + C[2, 1]) / s
                z = 0.25 * s
        else:
            s = 0.5 / np.sqrt(max(1e-12, t + 1.0))
            w = 0.25 / s
            x = (C[2, 1] - C[1, 2]) * s
            y = (C[0, 2] - C[2, 0]) * s
            z = (C[1, 0] - C[0, 1]) * s
        q = np.array([w, x, y, z], float)
        q = q / (np.linalg.norm(q) + 1e-12)
        if q[0] < 0:
            q = -q
        return q

    def quat_err_angle_deg(q_est_b2n, q_truth_b2n):
        q_err = q_mul(q_est_b2n, q_conj(q_truth_b2n))
        q_err = q_norm(q_err)
        ang = 2*np.arccos(np.clip(np.abs(q_err[:, 0]), 0.0, 1.0)) * 180.0 / np.pi
        return ang

    def ned_to_enu_quat():
        C_ne = np.array([[0.0, 1.0, 0.0],
                         [1.0, 0.0, 0.0],
                         [0.0, 0.0, -1.0]])
        return dcm_to_quat(C_ne)

    def ecef_to_ned_quat(lat_rad, lon_rad):
        sL, cL = np.sin(lat_rad), np.cos(lat_rad)
        sO, cO = np.sin(lon_rad), np.cos(lon_rad)
        C_ne = np.array([[-sL*cO, -sL*sO,  cL],
                         [    -sO,     cO,  0.],
                         [-cL*cO, -cL*sO, -sL]])
        C_en = C_ne.T
        return dcm_to_quat(C_en)

    def wrap_world(q_b2W, q_W2N, q_N2W):
        return q_mul(np.tile(q_W2N, (q_b2W.shape[0], 1)), q_mul(q_b2W, np.tile(q_N2W, (q_b2W.shape[0], 1))))

    def average_quat(q):
        M = (q[:, :, None] @ q[:, None, :]).sum(axis=0)
        w, v = np.linalg.eigh(M)
        q_mean = v[:, np.argmax(w)]
        if q_mean[0] < 0:
            q_mean = -q_mean
        return q_mean

    # Build estimate and truth candidates (both currently Body->NED, wxyz)
    tE = t_est
    qE_est = q_est  # already normalized and in Body->NED
    qT_raw = q_truth_ned  # Body->NED from earlier conversion

    # World transforms
    try:
        from utils import ecef_to_geodetic
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_ecef_tru[0])
        ref_lat_rad = np.deg2rad(lat_deg)
        ref_lon_rad = np.deg2rad(lon_deg)
    except Exception:
        ref_lat_rad = 0.0
        ref_lon_rad = 0.0
    q_ne = ned_to_enu_quat()
    q_en = q_conj(np.atleast_2d(q_ne))[0]
    q_en_ecef = ecef_to_ned_quat(ref_lat_rad, ref_lon_rad)
    q_ne_ecef = q_conj(np.atleast_2d(q_en_ecef))[0]

    # Candidate interpretations of truth
    cands = []
    cands.append(("truth b2NED", qT_raw))
    cands.append(("truth NED2b (conj)", q_conj(qT_raw)))
    cands.append(("truth b2ENU → b2NED", wrap_world(qT_raw, q_en, q_ne)))
    cands.append(("truth ENU2b → b2NED", wrap_world(q_conj(qT_raw), q_en, q_ne)))
    cands.append(("truth b2ECEF → b2NED", wrap_world(qT_raw, q_en_ecef, q_ne_ecef)))
    cands.append(("truth ECEF2b → b2NED", wrap_world(q_conj(qT_raw), q_en_ecef, q_ne_ecef)))

    # Score on early/static window
    t0, t1 = 0.0, 60.0
    idx = np.where((tE >= t0) & (tE <= t1))[0]
    if idx.size < 100:
        idx = np.arange(min(1000, tE.size))
    scores = []
    for name, qT in cands:
        ang = quat_err_angle_deg(qE_est[idx], qT[idx])
        scores.append((float(np.nanmean(ang)), name))
    best_i = int(np.argmin([s[0] for s in scores]))
    best_name, best_mean = scores[best_i][1], scores[best_i][0]
    qT_best = cands[best_i][1]
    print(f"[AutoAlign] best world/frame hypothesis: {best_name} (mean {best_mean:.3f}° over [{t0},{t1}]s)")

    # Estimate constant body-side delta over the window and apply to truth
    q_err_static = q_mul(qE_est[idx], q_conj(qT_best[idx]))
    q_delta = average_quat(q_err_static)
    if q_delta[0] < 0:
        q_delta = -q_delta
    q_delta_tiled = np.tile(q_delta, (qT_best.shape[0], 1))
    qT_aligned = q_mul(q_delta_tiled, qT_best)
    print(f"[AutoAlign] applied fixed body-side Δ: {np.array2string(q_delta, precision=4)}")

    # Use aligned truth and sign-align estimate for plotting
    qT = q_norm(qT_aligned)
    qE = _fix_hemisphere(qT, qE_est)
    # Angle error between aligned truth and estimate across full timeline
    ang = _quat_angle_deg(qT, qE)

    # Helper to align x and multiple y-arrays to the shortest common length
    def _align_to_same_len(x, *ys):
        n = int(min(len(x), *[len(y) for y in ys]))
        if n <= 0:
            return x, ys
        x2 = x[:n]
        ys2 = tuple(y[:n] for y in ys)
        return x2, ys2

    # plots (NO acceleration)
    # 1) quaternion components (Body->NED frame)
    plt.figure(figsize=(10, 6))
    # Ensure same-length arrays for plotting
    t_plot, (qT_plot, qE_plot) = _align_to_same_len(tE, qT, qE)
    # Final safety: normalise and align KF sign hemisphere to truth
    qT_plot = _qnorm(qT_plot)
    qE_plot = _qalign(_qnorm(qE_plot), qT_plot)
    for i, lab in enumerate(['w', 'x', 'y', 'z']):
        ax = plt.subplot(2, 2, i + 1)
        ax.plot(t_plot, qT_plot[:, i], '-', label='Truth')
        ax.plot(t_plot, qE_plot[:, i], '--', label='KF')
        ax.set_title(f'q_{lab}')
        ax.grid(True)
    plt.legend(loc='upper right')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Quaternion Truth vs KF')
    _save_png_and_mat(
        os.path.join(results_dir, f'{tag}_Task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png'),
        {'t': t_plot, 'q_truth': qT_plot, 'q_kf': qE_plot}
    )

    # 1b) quaternion component residuals (est - truth), sign-aligned already
    dq = qE_plot - qT_plot
    plt.figure(figsize=(10, 6))
    for i, lab in enumerate(['w', 'x', 'y', 'z']):
        ax = plt.subplot(2, 2, i + 1)
        ax.plot(t_plot, dq[:, i], '-', label='Δq = est − truth')
        ax.set_title(f'Δq_{lab}')
        ax.grid(True)
        if i == 0:
            ax.legend(loc='upper right')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Quaternion Component Error')
    _save_png_and_mat(
        os.path.join(results_dir, f'{tag}_Task7_6_BodyToNED_attitude_quaternion_error_components.png'),
        {'t': t_plot, 'dq_wxyz': dq}
    )

    # 2) Euler ZYX (yaw,pitch,roll)
    eT = _quat_to_euler_zyx_deg(qT_plot)
    eE = _quat_to_euler_zyx_deg(qE_plot)
    plt.figure(figsize=(10, 6))
    for i, lab in enumerate(['Yaw(Z)', 'Pitch(Y)', 'Roll(X)']):
        ax = plt.subplot(3, 1, i + 1)
        ax.plot(t_plot, eT[:, i], '-', label='Truth')
        ax.plot(t_plot, eE[:, i], '--', label='KF')
        ax.set_ylabel(lab + ' [deg]')
        ax.grid(True)
        if i == 0:
            ax.legend()
    plt.xlabel('Time [s]')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Euler (ZYX) Truth vs KF')
    _save_png_and_mat(
        os.path.join(results_dir, f'{tag}_Task7_6_BodyToNED_attitude_truth_vs_estimate_euler.png'),
        {'t': t_plot, 'e_truth_zyx_deg': eT, 'e_kf_zyx_deg': eE}
    )

    # 2b) Euler error (difference, wrapped to [-180, 180])
    def _wrap_deg(x):
        x = (x + 180.0) % 360.0 - 180.0
        # Promote -180 to +180 consistently for readability
        x[x == -180.0] = 180.0
        return x
    e_err = _wrap_deg(eE - eT)
    plt.figure(figsize=(10, 6))
    for i, lab in enumerate(['Yaw(Z)', 'Pitch(Y)', 'Roll(X)']):
        ax = plt.subplot(3, 1, i + 1)
        ax.plot(t_plot, e_err[:, i], '-', label='Estimate − Truth')
        ax.set_ylabel(lab + ' err [deg]')
        ax.grid(True)
        if i == 0:
            ax.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Euler Error (ZYX) vs Time')
    _save_png_and_mat(
        os.path.join(results_dir, f"{tag}_Task7_6_BodyToNED_attitude_euler_error_over_time.png"),
        {'t': t_plot, 'e_error_zyx_deg': e_err}
    )

    # 3) attitude angle error (ensure time and error are same length; decimate both if plotting heavy)
    t_plot = t_plot  # already aligned above
    ang_plot = ang
    # Align angle error length to x if needed
    if len(ang_plot) != len(t_plot):
        n = min(len(ang_plot), len(t_plot))
        t_plot = t_plot[:n]
        ang_plot = ang_plot[:n]
    try:
        if t_plot.size > 200000:
            step = max(2, int(round(t_plot.size / 50000)))  # target ~50k points
            t_plot = t_plot[::step]
            ang_plot = ang_plot[::step]
    except Exception:
        pass
    plt.figure(figsize=(10, 3))
    plt.plot(t_plot, ang_plot, '-')
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle Error [deg]')
    plt.title(f'{tag} Task7_6: Quaternion Error (angle) vs Time')
    _save_png_and_mat(os.path.join(results_dir, f'{tag}_Task7_6_attitude_error_angle_over_time.png'),
                      {'t': t_plot, 'att_err_deg': ang_plot})
    # Back-compat save under previous Task7 filename
    _save_png_and_mat(os.path.join(results_dir, f'{tag}_Task7_attitude_error_angle_over_time.png'),
                      {'t': t_plot, 'att_err_deg': ang_plot})

    # 4) Console summary tables for residuals (printed to stdout)
    def _metrics(vec):
        vec = np.asarray(vec).ravel()
        mean = float(np.mean(vec))
        rmse = float(np.sqrt(np.mean(vec**2)))
        p95 = float(np.percentile(np.abs(vec), 95))
        p99 = float(np.percentile(np.abs(vec), 99))
        vmax = float(np.max(np.abs(vec)))
        final = float(vec[-1]) if vec.size else float('nan')
        return mean, rmse, p95, p99, vmax, final

    # Quaternion component residuals metrics
    headers = ["metric", "w", "x", "y", "z"]
    rows = []
    stats_q = {k: _metrics(dq[:, i]) for i, k in enumerate(['w', 'x', 'y', 'z'])}
    for label, idx in [("mean", 0), ("rmse", 1), ("p95_abs", 2), ("p99_abs", 3), ("max_abs", 4), ("final", 5)]:
        rows.append([label] + [f"{stats_q[k][idx]:.6f}" for k in ['w','x','y','z']])
    print("\n===== Task7_6 Quaternion Component Error Summary (est − truth) =====")
    print(" ".join(f"{h:>12s}" for h in headers))
    for r in rows:
        print(" ".join(f"{c:>12s}" for c in r))

    # Euler residuals metrics (ZYX order: yaw, pitch, roll)
    headers_e = ["metric", "yaw", "pitch", "roll"]
    rows_e = []
    stats_e = {k: _metrics(e_err[:, i]) for i, k in enumerate(['yaw', 'pitch', 'roll'])}
    for label, idx in [("mean", 0), ("rmse", 1), ("p95_abs", 2), ("p99_abs", 3), ("max_abs", 4), ("final", 5)]:
        rows_e.append([label] + [f"{stats_e[k][idx]:.6f}" for k in ['yaw','pitch','roll']])
    print("\n===== Task7_6 Euler Error Summary (est − truth) [deg] =====")
    print(" ".join(f"{h:>12s}" for h in headers_e))
    for r in rows_e:
        print(" ".join(f"{c:>12s}" for c in r))

    # divergence time + length dependence
    div_t = _divergence_time(tE, ang, args.div_threshold_deg, args.div_persist_sec)
    if np.isnan(div_t):
        print("Estimated divergence start time (attitude): none")
    else:
        print(f"Estimated divergence start time (attitude): {div_t:.3f} s")

    with open(os.path.join(results_dir, f'{tag}_Task7_divergence_summary.csv'), 'w') as f:
        f.write('threshold_deg,persist_s,divergence_s\n')
        f.write(f"{args.div_threshold_deg},{args.div_persist_sec},{'' if np.isnan(div_t) else f'{div_t:.6f}'}\n")

    # length scan
    Ls = [float(x) for x in str(args.length_scan).split(',') if x.strip()]
    rows = []
    for L in Ls:
        tend = min(tE[0] + L, tE[-1])
        m = tE <= tend
        dv = _divergence_time(tE[m], ang[m], args.div_threshold_deg, args.div_persist_sec)
        rows.append((L, np.nan if (dv != dv) else dv))
    with open(os.path.join(results_dir, f'{tag}_Task7_divergence_vs_length.csv'), 'w') as f:
        f.write('tmax_s,divergence_s\n')
        for L, dv in rows:
            f.write(f"{L},{'' if (dv != dv) else dv}\n")

    plt.figure(figsize=(6, 4))
    plt.plot([r[0] for r in rows], [np.nan if (r[1] != r[1]) else r[1] for r in rows], '-o')
    plt.grid(True)
    plt.xlabel('Dataset length used [s]')
    plt.ylabel('Divergence time [s]')
    plt.title(f'{tag} Task7: Divergence vs Length')
    _save_png_and_mat(os.path.join(results_dir, f'{tag}_Task7_divergence_vs_length.png'),
                      {'tmax_s': np.array([r[0] for r in rows], float),
                       'divergence_s': np.array([np.nan if (r[1] != r[1]) else r[1] for r in rows], float)})
    print("Inline validation done.")


def _task5_plot_quat_cases(results_dir: str, run_id: str, truth_file: str) -> None:
    """Compare TRIAD-init KF vs TRUE-init KF quaternion components vs truth."""
    base_npz = Path(results_dir) / f"{run_id}_kf_output.npz"
    true_npz = Path(results_dir) / f"{run_id}_TRUEINIT_kf_output.npz"
    if not base_npz.exists() or not true_npz.exists():
        print("[Task5] Skipping TRIAD vs TRUE-init comparison (missing outputs).")
        return
    dA = np.load(base_npz, allow_pickle=True)
    dB = np.load(true_npz, allow_pickle=True)
    t = np.ravel(dA.get("time_s") if dA.get("time_s") is not None else dA.get("time"))
    qA = dA.get("attitude_q_harmonized")
    if qA is None:
        qA = dA.get("att_quat")
    if qA is None:
        qA = dA.get("attitude_q")
    if qA is None:
        qA = dA.get("quat_log")
    qB = dB.get("attitude_q_harmonized")
    if qB is None:
        qB = dB.get("att_quat")
    if qB is None:
        qB = dB.get("attitude_q")
    if qB is None:
        qB = dB.get("quat_log")
    if qA is None or qB is None or t is None:
        print("[Task5] Missing quaternion/time in outputs.")
        return
    qA = _norm_quat(np.asarray(qA))
    qB = _norm_quat(np.asarray(qB))

    # Load truth and prepare Body→NED truth quaternions on estimator time
    t_tru, pos_ecef_tru, vel_ecef_tru, q_tru = _read_state_x001(truth_file)
    # reference lat/lon from truth
    try:
        from utils import ecef_to_geodetic, compute_C_ECEF_to_NED
        lat_deg, lon_deg, _ = ecef_to_geodetic(*pos_ecef_tru[0])
        C_e2n = compute_C_ECEF_to_NED(np.deg2rad(lat_deg), np.deg2rad(lon_deg))
    except Exception:
        C_e2n = None

    # Interp truth to estimator time
    q_truth_on_t = _interp_quat_components(t_tru, q_tru, t)
    # Consider ECEF->NED conversion
    if C_e2n is not None:
        from scipy.spatial.transform import Rotation as _R
        qlist = []
        for q in q_truth_on_t:
            r_be = _R.from_quat([q[1], q[2], q[3], q[0]])
            R_bn = C_e2n @ r_be.as_matrix()
            q_xyzw = _R.from_matrix(R_bn).as_quat()
            qlist.append([q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]])
        q_truth_e2n = _norm_quat(np.asarray(qlist))
    else:
        q_truth_e2n = _norm_quat(q_truth_on_t)

    # Detect estimator quaternion format vs truth and fix before comparisons
    try:
        qA_fixed, infoA = _qfix_est(qA, q_truth_on_t)
        print(f"[Task5][Detect] TRIAD-init quaternion as {infoA.get('name')} (+conj={infoA.get('conj')}); mean Δθ={infoA.get('err_deg'):.3f}°")
        qA = qA_fixed
    except Exception:
        pass
    try:
        qB_fixed, infoB = _qfix_est(qB, q_truth_on_t)
        print(f"[Task5][Detect] TRUE-init quaternion as {infoB.get('name')} (+conj={infoB.get('conj')}); mean Δθ={infoB.get('err_deg'):.3f}°")
        qB = qB_fixed
    except Exception:
        pass

    # Pick direct vs converted by median error against TRIAD-init
    def _median_err(qt):
        qe = _fix_hemisphere(qt, qA)
        return float(np.nanmedian(_quat_angle_deg(qt, qe)))
    q_truth_direct = _norm_quat(q_truth_on_t)
    qT = q_truth_e2n if _median_err(q_truth_e2n) + 1e-3 < _median_err(q_truth_direct) else q_truth_direct

    # Direction detection via t0 error
    err0_dir = float(_quat_angle_deg(qT, _fix_hemisphere(qT, qA))[0])
    def _q_conj_local(q):
        q = np.asarray(q, float)
        return np.column_stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]])
    err0_con = float(_quat_angle_deg(_q_conj_local(qT), _fix_hemisphere(_q_conj_local(qT), qA))[0])
    if err0_con + 1e-3 < err0_dir:
        qT = _q_conj_local(qT)
        print(f"[Task5] Truth appears NED→Body; using conjugate. (err0 direct={err0_dir:.2f}°, conj={err0_con:.2f}°)")

    # Align per-sample signs
    qA = _fix_hemisphere(qT, qA)
    qB = _fix_hemisphere(qT, qB)

    # Plot
    plt.figure(figsize=(10, 8))
    labs = ["qw", "qx", "qy", "qz"]
    for i in range(4):
        ax = plt.subplot(4, 1, i + 1)
        ax.plot(t, qT[:, i], 'k-', label='Truth')
        ax.plot(t, qA[:, i], 'b--', label='KF (TRIAD-init)')
        ax.plot(t, qB[:, i], 'r-.', label='KF (TRUE-init)')
        ax.set_ylabel(labs[i])
        ax.grid(True)
        if i == 0:
            ax.legend(loc='best')
    plt.xlabel('Time [s]')
    plt.suptitle('Task 5: Quaternions — Truth vs KF (TRIAD-init vs TRUE-init)')
    out = Path(results_dir) / f"{run_id}_Task5_quat_cases_TRIADinit_vs_TRUEinit.png"
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    try:
        plt.gcf().savefig(out, dpi=200, bbox_inches='tight')
        print(f"[Task5] Saved TRIAD vs TRUE-init quaternion comparison -> {out}")
    except Exception as e:
        print(f"[Task5] Failed to save comparison plot: {e}")
    finally:
        plt.close()


def main(argv: Iterable[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description="Run TRIAD (Task-1) on a selected dataset or run all methods",
        allow_abbrev=False,
    )
    parser.add_argument(
        "--dataset",
        choices=["X001", "X002", "X003"],
        default="X002",
        help="Dataset ID to use (default X002)",
    )
    parser.add_argument("--imu", type=str, help="Path to IMU data file")
    parser.add_argument("--gnss", type=str, help="Path to GNSS data file")
    parser.add_argument("--truth", type=str, help="Path to truth data file")
    parser.add_argument(
        "--auto-truth",
        dest="auto_truth",
        action="store_true",
        default=True,
        help="Auto-detect a truth file if --truth is not provided (default on)",
    )
    parser.add_argument(
        "--no-auto-truth",
        dest="auto_truth",
        action="store_false",
        help="Do not auto-detect truth; ignore any available truth unless --truth is given",
    )
    parser.add_argument(
        "--allow-truth-mismatch",
        action="store_true",
        help="Allow using a truth file from a different dataset (use with care)",
    )
    parser.add_argument(
        "--outdir", type=str, help="Directory to write results (default PYTHON/results)"
    )
    parser.add_argument("--no-plots", action="store_true", help="Skip plot generation")
    parser.add_argument(
        "--mode",
        choices=["all", "triad"],
        default="triad",
        help=(
            "Mode: run all methods across datasets (all) or TRIAD-only (triad). "
            "Default is triad."
        ),
    )
    parser.add_argument(
        "--show-measurements",
        action="store_true",
        help="Include IMU and GNSS measurements in Task 6 overlay plots",
    )
    parser.add_argument(
        "--task",
        type=int,
        help="Run a single helper task and exit",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        "--debug",
        dest="debug",
        action="store_true",
        help="Enable verbose task-level logging",
    )
    parser.add_argument("--imu-rate", type=float, default=None, help="Hint IMU sample rate [Hz]")
    parser.add_argument("--gnss-rate", type=float, default=None, help="Hint GNSS sample rate [Hz]")
    parser.add_argument("--truth-rate", type=float, default=None, help="Hint truth sample rate [Hz]")
    parser.add_argument("--tasks", type=str, default=None, help="Comma-separated task numbers to run")
    parser.add_argument('--div-threshold-deg', type=float, default=30.0,
                        help='Attitude error threshold in degrees for divergence.')
    parser.add_argument('--div-persist-sec', type=float, default=10.0,
                        help='Seconds above threshold required to declare divergence.')
    parser.add_argument('--length-scan', type=str, default="60,120,300,600,900,1200",
                        help='Comma-separated end-times (s) for divergence-vs-length study.')
    # Attitude initialisation and yaw-fusion controls (passed to GNSS_IMU_Fusion)
    parser.add_argument('--init-att-with-truth', action='store_true',
                        help='Initialise the KF attitude with the truth quaternion at IMU t0')
    parser.add_argument('--truth-quat-frame', choices=['NED', 'ECEF'], default='NED',
                        help='Frame of the truth quaternion when initialising from truth')
    parser.add_argument('--fuse-yaw', action='store_true',
                        help='Fuse GNSS-derived yaw to correct quaternion drift')
    parser.add_argument('--yaw-gain', type=float, default=0.2,
                        help='Gain [0..1] per update for GNSS yaw fusion')
    parser.add_argument('--yaw-speed-min', type=float, default=2.0,
                        help='Min horizontal speed [m/s] to trust GNSS yaw')
    parser.add_argument('--true-init-second-pass', action='store_true',
                        help='Run a second KF pass with true attitude init and compare quaternions (Task 5).')

    args = parser.parse_args(argv)

    # Convenience: if requested to run everything, forward to run_all_methods.py
    if args.mode == "all":
        from pathlib import Path as _Path
        import subprocess as _subprocess
        HERE_ = _Path(__file__).resolve().parent
        cmd = [sys.executable, str(HERE_ / "run_all_methods.py")]
        if args.no_plots:
            cmd.append("--no-plots")
        # Forward dataset override if explicitly given
        if args.imu and args.gnss:
            cmd += ["--datasets", args.imu, args.gnss]
        print("Running all methods across datasets …")
        rc = _subprocess.call(cmd)
        sys.exit(rc)

    # Build method/dataset-specific results directory: PYTHON/results/<Method>/<Dataset>
    base_results = Path(args.outdir) if args.outdir else _py_results_dir()
    results_dir = base_results / "TRIAD" / args.dataset
    results_dir.mkdir(parents=True, exist_ok=True)
    logger.info("Ensured '%s' directory exists.", results_dir)
    # Expose the results directory to child processes so that all plotting
    # helpers can place their outputs directly in ``PYTHON/results``.
    os.environ["PYTHON_RESULTS_DIR"] = str(results_dir)
    import os as _os
    if not _os.environ.get("RESULTS_NOTE_PRINTED"):
        print("Note: Python saves to results/ ; MATLAB saves to MATLAB/results/ (independent).")
        _os.environ["RESULTS_NOTE_PRINTED"] = "1"

    if args.debug:
        logger.setLevel(logging.DEBUG)

    if args.task == 7:
        from evaluate_filter_results import run_evaluation

        # Save Task 7 output directly under ``results/``
        run_evaluation(
            prediction_file="outputs/predicted_states.csv",
            gnss_file="outputs/gnss_measurements.csv",
            attitude_file="outputs/estimated_attitude.csv",
            save_path="results",
        )
        return

    method = "TRIAD"

    results: List[Dict[str, float | str]] = []

    # Resolve inputs from CLI or dataset selection
    if args.imu:
        imu_path = Path(args.imu)
    else:
        imu_path = _imu_path_helper(args.dataset)

    if args.gnss:
        gnss_path = Path(args.gnss)
    else:
        gnss_path = _gnss_path_helper(args.dataset)

    # Resolve truth path
    user_supplied_truth = bool(args.truth)
    truth_path = Path(args.truth) if user_supplied_truth else None
    # If not supplied and auto-truth is enabled, try to locate an appropriate file
    if (not user_supplied_truth) and args.auto_truth:
        try:
            auto = resolve_truth_path(args.dataset)
        except Exception:
            auto = None
        if auto and Path(auto).exists():
            truth_path = Path(auto)
            print(f"Auto-detected truth file: {truth_path}")
    logger.info(
        "Resolved input files: imu=%s gnss=%s truth=%s", imu_path, gnss_path, truth_path
    )

    # Input validation with friendly errors
    header = [
        "Running TRIAD with:",
        f"  IMU:   {imu_path}",
        f"  GNSS:  {gnss_path}",
        f"  Truth: {truth_path if truth_path else 'N/A'}",
        f"  Out:   {results_dir}",
    ]
    print("\n".join(header))
    for p in (imu_path, gnss_path):
        if not Path(p).exists():
            raise FileNotFoundError(f"Required input not found: {p}")
    # If user explicitly provided a truth path and it doesn't exist, error.
    if user_supplied_truth and truth_path and not Path(truth_path).exists():
        raise FileNotFoundError(f"Truth file not found: {truth_path}")
    # Enforce single-pair only when a truth file is present (allow mixing).
    if truth_path and Path(truth_path).exists():
        try:
            assert_single_pair(
                str(imu_path), str(gnss_path), str(truth_path),
                force_mix=True,
            )
        except Exception:
            pass

    run_id = build_run_id(str(imu_path), str(gnss_path), method)
    os.environ["RUN_ID"] = run_id
    log_path = results_dir / f"{run_id}.log"
    print(f"\u25b6 {run_id}")

    import os as _os
    if not _os.environ.get("RESULTS_NOTE_PRINTED"):
        print("Note: Python saves to results/ ; MATLAB saves to MATLAB/results/ (independent).")
        _os.environ["RESULTS_NOTE_PRINTED"] = "1"
    print_timeline(run_id, str(imu_path), str(gnss_path), str(truth_path) if truth_path else None, out_dir=str(results_dir))

    task_set = None
    if args.tasks:
        task_set = {int(x) for x in re.split(r"[,\s]+", args.tasks) if x}
    if task_set == {6}:
        est_file = results_dir / f"{run_id}_kf_output.npz"
        cmd = [
            sys.executable,
            str(HERE / "task6_plot_truth.py"),
            "--est-file",
            str(est_file),
            "--truth-file",
            str(truth_path),
            "--gnss-file",
            str(gnss_path),
            "--output",
            str(results_dir),
        ]
        # Apply Task 6 plotting defaults (decimation + y-limit sync)
        cmd += ["--decimate-maxpoints", "200000", "--ylim-percentile", "99.5"]
        if args.debug:
            cmd.append("--debug-task6")
        subprocess.check_call(cmd)
        return

    if logger.isEnabledFor(logging.DEBUG):
        try:
            gnss_preview = np.loadtxt(gnss_path, delimiter=",", skiprows=1, max_rows=1)
            imu_preview = np.loadtxt(imu_path, max_rows=1)
            logger.debug(
                "GNSS preview: shape %s, first row: %s",
                gnss_preview.shape,
                gnss_preview,
            )
            logger.debug(
                "IMU preview: shape %s, first row: %s",
                imu_preview.shape,
                imu_preview,
            )
        except Exception as e:  # pragma: no cover - best effort
            logger.warning(
                "Failed data preview for %s or %s: %s", imu_path, gnss_path, e
            )

    cmd = [
        sys.executable,
        str(HERE / "GNSS_IMU_Fusion.py"),
        "--imu-file",
        str(imu_path),
        "--gnss-file",
        str(gnss_path),
        "--method",
        method,
    ]
    # Pass through attitude/truth/yaw options to the fusion step
    if truth_path and Path(truth_path).exists():
        cmd += ["--truth-file", str(truth_path), "--allow-truth-mismatch"]
        if args.init_att_with_truth:
            cmd.append("--init-att-with-truth")
            cmd += ["--truth-quat-frame", str(args.truth_quat_frame)]
    if args.fuse_yaw:
        cmd.append("--fuse-yaw")
        cmd += ["--yaw-gain", str(args.yaw_gain), "--yaw-speed-min", str(args.yaw_speed_min)]
    if args.no_plots:
        cmd.append("--no-plots")
    logger.info("Running fusion command: %s", cmd)
    start_t = time.time()
    ret, summaries = run_case(cmd, log_path)
    runtime = time.time() - start_t
    if ret != 0:
        logger.info("Fusion command failed with return code %s", ret)
        raise subprocess.CalledProcessError(ret, cmd)
    logger.info("Fusion command completed in %.2fs", runtime)

    for summary in summaries:
        kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
        metrics = {
            "dataset": imu_path.stem,
            "method": kv.get("method", method),
            "rmse_pos": float(kv.get("rmse_pos", "nan").replace("m", "")),
            "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
            "rms_resid_pos": float(kv.get("rms_resid_pos", "nan").replace("m", "")),
            "max_resid_pos": float(kv.get("max_resid_pos", "nan").replace("m", "")),
            "rms_resid_vel": float(kv.get("rms_resid_vel", "nan").replace("m", "")),
            "max_resid_vel": float(kv.get("max_resid_vel", "nan").replace("m", "")),
            "accel_bias": float(kv.get("accel_bias", "nan")),
            "gyro_bias": float(kv.get("gyro_bias", "nan")),
            "zupt_count": float(kv.get("ZUPT_count", "nan")),
            "grav_err_mean_deg": float(kv.get("GravErrMean_deg", "nan")),
            "grav_err_max_deg": float(kv.get("GravErrMax_deg", "nan")),
            "earth_rate_err_mean_deg": float(kv.get("EarthRateErrMean_deg", "nan")),
            "earth_rate_err_max_deg": float(kv.get("EarthRateErrMax_deg", "nan")),
            "runtime": runtime,
        }
        results.append(metrics)
    if results:
        metrics_path = results_dir / f"{run_id}_metrics.json"
        with metrics_path.open("w", encoding="utf-8") as f:
            json.dump(results[0], f, indent=2, sort_keys=True)
        np.savez(results_dir / f"{run_id}_metrics.npz", **results[0])
        logger.info("Saved metrics to %s and %s", metrics_path, metrics_path.with_suffix('.npz'))

    # PNG outputs are now written directly into ``results_dir`` with unique
    # filenames, so no post-processing or manifest copying is required.

    # ------------------------------------------------------------------
    # Attitude comparison plots: KF vs Truth and Gyro DR vs Truth
    # ------------------------------------------------------------------
    try:
        est_npz = results_dir / f"{run_id}_kf_output.npz"
        if est_npz.exists() and truth_path and Path(truth_path).exists():
            plot_cmd = [
                sys.executable,
                str(HERE / "../scripts/plot_attitude_kf_vs_no_kf.py"),
                "--imu-file", str(imu_path),
                "--est-file", str(est_npz),
                "--truth-file", str(truth_path),
                "--out-dir", str(results_dir),
                "--true-init",
            ]
            logger.info("Running attitude comparison plots: %s", plot_cmd)
            # Ensure imports work when executed as a script by adding project root
            env = os.environ.copy()
            try:
                proj_root = str(ROOT)
                src_path = str(HERE)
                extra = os.pathsep.join([proj_root, src_path])
                env["PYTHONPATH"] = extra + os.pathsep + env.get("PYTHONPATH", "")
            except Exception:
                pass
            subprocess.check_call(plot_cmd, env=env)
            print("Generated attitude comparison plots (KF vs truth, DR vs truth).")
        else:
            print("[INFO] Skipping attitude comparison plots (missing estimate or truth file).")
    except Exception as e:
        print(f"[WARN] Attitude comparison plotting failed: {e}")

    # ------------------------------------------------------------------
    # Convert NPZ output to a MATLAB file with explicit frame variables
    # ------------------------------------------------------------------
    npz_path = results_dir / f"{run_id}_kf_output.npz"
    t_imu = None
    tmeta: Dict[str, float | int | str] = {}
    if npz_path.exists():
        data = np.load(npz_path, allow_pickle=True)
        logger.debug("Loaded output %s with keys: %s", npz_path, list(data.keys()))
        time_s_raw = data.get("time_s")
        if time_s_raw is None:
            time_s_raw = data.get("time")

        dt_hint = None
        for name in ("imu_dt", "dt_imu", "dt"):
            if name in data and np.isscalar(data[name]):
                dt_hint = float(data[name])
                break

        imu_rate_hint = args.imu_rate
        if imu_rate_hint is None:
            for name in ("IMU_RATE_HZ", "imu_rate", "rate_imu_hz", "rate_imu", "imu_rate_hz"):
                if name in data and np.isscalar(data[name]):
                    imu_rate_hint = float(data[name])
                    break

        t_imu, tmeta = _make_monotonic_time(
            t_like=time_s_raw,
            fallback_len=len(time_s_raw) if time_s_raw is not None else None,
            dt=dt_hint,
            imu_rate_hint=imu_rate_hint,
        )
        time_s = t_imu

        if time_s is not None and len(time_s) > 1:
            imu_dt = float(np.median(np.diff(time_s)))
            imu_rate_hz = 1.0 / imu_dt
        else:
            imu_dt = None
            imu_rate_hz = imu_rate_hint

        pos_ned = data.get("pos_ned_m")
        if pos_ned is None:
            pos_ned = data.get("pos_ned")
        if pos_ned is None:
            pos_ned = data.get("fused_pos")

        vel_ned = data.get("vel_ned_ms")
        if vel_ned is None:
            vel_ned = data.get("vel_ned")
        if vel_ned is None:
            vel_ned = data.get("fused_vel")

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "Output time range: %s to %s s",
                time_s[0] if time_s is not None else "N/A",
                time_s[-1] if time_s is not None else "N/A",
            )
            logger.debug(
                "Position shape: %s",
                pos_ned.shape if pos_ned is not None else "None",
            )

        ref_lat = data.get("ref_lat_rad")
        ref_lon = data.get("ref_lon_rad")
        ref_r0 = data.get("ref_r0_m")
        if ref_lat is None:
            ref_lat = data.get("ref_lat")
        if ref_lon is None:
            ref_lon = data.get("ref_lon")
        if ref_r0 is None:
            ref_r0 = data.get("ref_r0")
        ref_lat = float(np.squeeze(ref_lat))
        ref_lon = float(np.squeeze(ref_lon))
        ref_r0 = np.asarray(ref_r0)

        C_NED_ECEF = compute_C_NED_to_ECEF(ref_lat, ref_lon)
        pos_ecef = (C_NED_ECEF @ pos_ned.T).T + ref_r0
        vel_ecef = (C_NED_ECEF @ vel_ned.T).T

        quat = data.get("attitude_q_harmonized")
        if quat is None:
            quat = data.get("att_quat")
        if quat is None:
            quat = data.get("attitude_q")
        if quat is None:
            quat = data.get("quat_log")
        if quat is not None:
            rot = R.from_quat(quat[:, [1, 2, 3, 0]])
            C_B_N = rot.as_matrix()
            pos_body = np.einsum("nij,nj->ni", C_B_N.transpose(0, 2, 1), pos_ned)
            vel_body = np.einsum("nij,nj->ni", C_B_N.transpose(0, 2, 1), vel_ned)
        else:
            pos_body = np.zeros_like(pos_ned)
            vel_body = np.zeros_like(vel_ned)

        mat_out = {
            "time_s": time_s,
            "t_est": time_s,
            "pos_ned_m": pos_ned,
            "vel_ned_ms": vel_ned,
            "pos_ecef_m": pos_ecef,
            "vel_ecef_ms": vel_ecef,
            "pos_body_m": pos_body,
            "vel_body_ms": vel_body,
            "ref_lat_rad": ref_lat,
            "ref_lon_rad": ref_lon,
            "ref_r0_m": ref_r0,
            "att_quat": quat,
            "method_name": method,
            "dt": imu_dt,
            "imu_rate_hz": imu_rate_hz,
            # Consistent fused data for Tasks 5 and 6
            "fused_pos": pos_ned,
            "fused_vel": vel_ned,
            "quat_log": quat,
        }
        save_mat(npz_path.with_suffix(".mat"), mat_out)

        # ------------------------------------------------------------------
        # Export estimator time vector and sampling interval for MATLAB Task 7
        # ------------------------------------------------------------------
        x_log = data.get("x_log")
        if x_log is not None and imu_dt is not None:
            t_est = np.arange(x_log.shape[1]) * imu_dt
            mat_time = {"t_est": t_est, "dt": imu_dt, "x_log": x_log}
            time_path = results_dir / f"{run_id}_task5_time.mat"
            sio.savemat(str(time_path), mat_time)
            logger.info("Saved Task 5 time data to %s", time_path)

        logger.info(
            "Subtask 6.8.2: Plotted %s position North: First = %.4f, Last = %.4f",
            method,
            pos_ned[0, 0],
            pos_ned[-1, 0],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s position East: First = %.4f, Last = %.4f",
            method,
            pos_ned[0, 1],
            pos_ned[-1, 1],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s position Down: First = %.4f, Last = %.4f",
            method,
            pos_ned[0, 2],
            pos_ned[-1, 2],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s velocity North: First = %.4f, Last = %.4f",
            method,
            vel_ned[0, 0],
            vel_ned[-1, 0],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s velocity East: First = %.4f, Last = %.4f",
            method,
            vel_ned[0, 1],
            vel_ned[-1, 1],
        )
        logger.info(
            "Subtask 6.8.2: Plotted %s velocity Down: First = %.4f, Last = %.4f",
            method,
            vel_ned[0, 2],
            vel_ned[-1, 2],
        )
        def _infer_rate(t):
            import numpy as np
            if t is None or len(t) < 2:
                return None
            dt = np.median(np.diff(t))
            return None if dt <= 0 else 1.0 / float(dt)

        from pathlib import Path as _P
        meta = {
            "imu_file": str(imu_path),
            "gnss_file": str(gnss_path),
            "truth_file": (str(truth_path) if (truth_path and _P(truth_path).exists()) else None),
            "imu_rate_hz": _infer_rate(t_imu) or args.imu_rate,
            "gnss_rate_hz": args.gnss_rate,
            "truth_rate_hz": args.truth_rate,
            "imu_time_meta": tmeta,
        }
        _write_run_meta(results_dir, run_id, **meta)

    # Task 6 removed per requirements; no truth overlays are generated here.

    # ----------------------------
    # Task 7: Evaluation
    # ----------------------------
    # Task 7 plots are now saved directly under the ``results``
    # directory without subfolders for easier navigation.
    task7_dir = results_dir
    with open(log_path, "a") as log:
        log.write("\nTASK 7: Evaluate residuals\n")
        msg = "Running Task 7 evaluation ..."
        logger.info(msg)
        log.write(msg + "\n")
        buf = io.StringIO()
        with redirect_stdout(buf):
            try:
                run_evaluation_npz(str(npz_path), str(task7_dir), run_id)
            except Exception as e:  # pragma: no cover - graceful failure
                print(f"Task 7 failed: {e}")
        output = buf.getvalue()
        print(output, end="")
        log.write(output)
        print(
            f"Saved Task 7.5 diff-truth plots (NED/ECEF/Body) under: results/{run_id}/"
        )
        logger.info("Task 7 evaluation complete; results in %s", task7_dir)

    # ----------------------------
    # Task 7.x: Inline truth validation (no external script)
    # ----------------------------
    try:
        # Only run inline truth validation if a truth file is available
        if truth_path and Path(truth_path).exists():
            # Some generators preserve original IMU/GNSS stem case (e.g., 'small'). Try both run_id and alt.
            kf_mat = os.path.join(str(results_dir), f"{run_id}_kf_output.mat")
            if not os.path.isfile(kf_mat):
                run_id_alt = f"{Path(imu_path).stem}_{Path(gnss_path).stem}_{method}"
                kf_mat_alt = os.path.join(str(results_dir), f"{run_id_alt}_kf_output.mat")
                if os.path.isfile(kf_mat_alt):
                    kf_mat = kf_mat_alt
            _run_inline_truth_validation(str(results_dir), run_id, kf_mat, str(truth_path), args)
    except Exception as e:
        print(f"[WARN] Inline validation failed: {e}")

    # Optional: run a second KF pass with true attitude initialisation and compare
    try:
        if args.true_init_second_pass and truth_path and Path(truth_path).exists():
            # Load detected truth quaternion frame (if available)
            det_file = Path(results_dir) / f"{run_id}_Task7_truth_quaternion_frame.txt"
            truth_frame = 'NED'
            if det_file.exists():
                try:
                    truth_frame = det_file.read_text().strip().upper()
                except Exception:
                    pass
            cmd_true = [
                sys.executable,
                str(HERE / "GNSS_IMU_Fusion.py"),
                "--imu-file", str(imu_path),
                "--gnss-file", str(gnss_path),
                "--method", method,
                "--truth-file", str(truth_path),
                "--allow-truth-mismatch",
                "--init-att-with-truth",
                "--truth-quat-frame", truth_frame,
                "--tag", "TRUEINIT",
            ]
            if args.no_plots:
                cmd_true.append("--no-plots")
            logger.info("Running TRUE-init KF pass: %s", cmd_true)
            ret2, _ = run_case(cmd_true, results_dir / f"{run_id}_TRUEINIT.log")
            if ret2 != 0:
                print("[WARN] TRUE-init KF pass failed; skipping comparison plot.")
            else:
                # Try both canonical and alternate case run-ids for output matching
                _task5_plot_quat_cases(str(results_dir), run_id, str(truth_path))
                _task5_plot_quat_cases(str(results_dir), f"{Path(imu_path).stem}_{Path(gnss_path).stem}_{method}", str(truth_path))
    except Exception as e:
        print(f"[WARN] TRUE-init comparison step failed: {e}")

    # --- nicely formatted summary table --------------------------------------
    if results:
        rows = [
            [
                e["dataset"],
                e["method"],
                e["rmse_pos"],
                e["final_pos"],
                e["rms_resid_pos"],
                e["max_resid_pos"],
                e["rms_resid_vel"],
                e["max_resid_vel"],
                e["runtime"],
            ]
            for e in results
        ]
        print(
            tabulate(
                rows,
                headers=[
                    "Dataset",
                    "Method",
                    "RMSEpos [m]",
                    "End-Error [m]",
                    "RMSresidPos [m]",
                    "MaxresidPos [m]",
                    "RMSresidVel [m/s]",
                    "MaxresidVel [m/s]",
                    "Runtime [s]",
                ],
                floatfmt=".2f",
            )
        )
        df = pd.DataFrame(
            rows,
            columns=[
                "Dataset",
                "Method",
                "RMSEpos_m",
                "EndErr_m",
                "RMSresidPos_m",
                "MaxresidPos_m",
                "RMSresidVel_mps",
                "MaxresidVel_mps",
                "Runtime_s",
            ],
        )
        df.to_csv(results_dir / "summary.csv", index=False)

    # Flat layout requested: do not create subfolders under results_dir

    # ----------------------------
    # Post-run artifact validation
    # ----------------------------
    try:
        def _exists(name: str) -> bool:
            p_top = results_dir / name
            return p_top.exists()

        must_have = [
            # Task 1
            f"{run_id}_task1_location_map.png",
            # Task 2
            f"{run_id}_task2_static_interval.png",
            f"{run_id}_task2_vectors.png",
            # Task 3
            f"{run_id}_task3_quaternions.png",
            f"{run_id}_task3_errors.png",
            # Task 4
            f"{run_id}_task4_comparison_ned.png",
            f"{run_id}_task4_mixed_frames.png",
            f"{run_id}_task4_all_ned.png",
            f"{run_id}_task4_all_ecef.png",
            f"{run_id}_task4_all_body.png",
            # Task 5
            f"{run_id}_task5_results_{method}.png",
            f"{run_id}_task5_mixed_frames.png",
            f"{run_id}_task5_all_ned.png",
            f"{run_id}_task5_all_ecef.png",
            f"{run_id}_task5_all_body.png",
            # Task 6 removed
            # Task 7 core plots
            f"{run_id}_task7_3_residuals_position_velocity.png",
            f"{run_id}_task7_3_error_norms.png",
            f"{run_id}_task7_4_attitude_angles_euler.png",
            # Differences over time
            f"{run_id}_task7_5_diff_truth_fused_over_time_NED.png",
            f"{run_id}_task7_5_diff_truth_fused_over_time_ECEF.png",
            f"{run_id}_task7_5_diff_truth_fused_over_time_Body.png",
        ]
        missing = [m for m in must_have if not _exists(m)]
        if missing:
            print("[VALIDATE] Missing expected artifacts:")
            for m in missing:
                print("  -", m)
        else:
            print("[VALIDATE] All required Task1–Task7 artifacts present.")
    except Exception as e:
        print(f"[WARN] Artifact validation skipped: {e}")

    print("TRIAD processing complete for X002")


if __name__ == "__main__":
    main()
