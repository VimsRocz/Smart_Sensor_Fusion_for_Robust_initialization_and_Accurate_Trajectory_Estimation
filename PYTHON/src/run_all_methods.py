#!/usr/bin/env python3
"""Run every (IMU, GNSS, method) combo and log the output.

The script processes each IMU/GNSS pair with all selected methods and writes
the console output to ``results/<IMU>_<GNSS>_<method>.log``.  By default the
bundled ``IMU_X`` data sets are used, but a YAML configuration file can
override the data files and the list of methods.

Example ``config.yml``::

    datasets:
      - imu: IMU_X001.dat
        gnss: GNSS_X001.csv
      - imu: IMU_X002.dat
        gnss: GNSS_X002.csv
    methods: [TRIAD, Davenport, SVD]

Run the script with ``--config config.yml`` to process those files.
"""

import argparse
from pathlib import Path as _Path
import sys as _sys
_SRC = _Path(__file__).resolve().parent
if str(_SRC) not in _sys.path:
    _sys.path.insert(0, str(_SRC))
REPO_ROOT = _SRC.parents[2]
from dependency_bootstrap import ensure_dependencies

ensure_dependencies()

import itertools
import os
import pathlib
import subprocess
import sys
from typing import Iterable, Tuple
import logging
import re
import time
import zipfile
import pandas as pd
from tabulate import tabulate
import numpy as np
from scipy.spatial.transform import Rotation as R
from utils import save_mat
from contextlib import redirect_stdout
import io
from evaluate_filter_results import run_evaluation_npz

from utils import compute_C_ECEF_to_NED

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
from paths import (
    ensure_results_dir as _ensure_results,
    imu_path as _imu_path_helper,
    gnss_path as _gnss_path_helper,
)

# Import helper utilities from the utils package
from utils.timeline import print_timeline_summary  # type: ignore
from utils.resolve_truth_path import resolve_truth_path  # type: ignore

try:
    import yaml
except ModuleNotFoundError:  # allow running without PyYAML installed
    yaml = None

DEFAULT_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
    ("IMU_X002.dat", "GNSS_X002.csv"),
    ("IMU_X003.dat", "GNSS_X002.csv"),  # dataset X003 shares GNSS_X002
]

SMALL_DATASETS: Iterable[Tuple[str, str]] = [
    ("IMU_X001_small.dat", "GNSS_X001_small.csv"),
    ("IMU_X002_small.dat", "GNSS_X002_small.csv"),
    ("IMU_X003_small.dat", "GNSS_X002_small.csv"),
]

DEFAULT_METHODS = ["TRIAD", "SVD", "Davenport"]

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def load_config(path: str):
    """Return (datasets, methods) from a YAML config file."""
    if yaml is None:
        raise RuntimeError("PyYAML is required to use --config")
    with open(path) as fh:
        data = yaml.safe_load(fh) or {}
    datasets = [
        (item["imu"], item["gnss"]) for item in data.get("datasets", [])
    ] or list(DEFAULT_DATASETS)
    methods = data.get("methods", DEFAULT_METHODS)
    return datasets, methods


def compute_C_NED_to_ECEF(lat: float, lon: float) -> np.ndarray:
    """Return rotation matrix from NED to ECEF frame."""
    return compute_C_ECEF_to_NED(lat, lon).T


def run_case(cmd, log_path):
    """Run a single fusion command and log output live to console and file."""
    logger.info("Executing fusion command: %s", cmd)
    # Ensure child Python processes flush their output so logs appear in real time
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    # ``bufsize=1`` requests line-buffered behaviour when possible
    with open(log_path, "w") as log:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=env,
        )
        summary_lines = []
        for line in proc.stdout:
            print(line, end="")
            log.write(line)
            m = SUMMARY_RE.search(line)
            if m:
                summary_lines.append(m.group(1))
        proc.wait()
    logger.info("Fusion command completed with return code %s", proc.returncode)
    return proc.returncode, summary_lines


# ---------------------------------------------------------------------------
# Task7 Attitude plotting helpers (mirrors logic from run_triad_only.py)
# ---------------------------------------------------------------------------
from typing import Dict, Any, List, Tuple, Optional


def _save_png_and_mat(base_png_path: str, data_dict: Dict[str, Any]):
    """Save current matplotlib figure to PNG and accompanying MAT file.

    The MAT filename mirrors the PNG but with .mat extension. Failures are
    logged but non-fatal.
    """
    try:
        import matplotlib.pyplot as plt  # local import to avoid global dependency if never used
        png_path = pathlib.Path(base_png_path)
        png_path.parent.mkdir(parents=True, exist_ok=True)
        plt.gcf().savefig(png_path, dpi=150, bbox_inches="tight")
        print(f"[PNG] {png_path}")
    except Exception as e:  # pragma: no cover - best-effort
        logger.warning(f"Failed to save PNG {base_png_path}: {e}")
    try:
        from scipy.io import savemat
        mat_path = str(pathlib.Path(base_png_path).with_suffix(".mat"))
        savemat(mat_path, {k: (np.asarray(v) if not isinstance(v, (int, float)) else v) for k, v in data_dict.items()})
        print(f"[MAT] {mat_path}")
    except Exception as e:  # pragma: no cover
        logger.warning(f"Failed to save MAT for {base_png_path}: {e}")
    return pathlib.Path(base_png_path)


def _task7_attitude_plots(est_npz: pathlib.Path, truth_file: Optional[pathlib.Path], tag: str,
                          results_dir: pathlib.Path, div_threshold_deg: float = 30.0,
                          div_persist_sec: float = 10.0, length_scan: str = "60,120,300,600,900,1200",
                          subdir: str = "task7_attitude") -> None:
    """Replicate Task7 attitude plots produced by run_triad_only for batch runs.

    Generates quaternion/euler truth-vs-estimate plots, component residuals,
    angle error over time, divergence summary and divergence-vs-length study.
    Safe to call even if inputs are incomplete (will silently skip).
    """
    try:
        import matplotlib.pyplot as plt  # local import
    except Exception as e:  # pragma: no cover
        logger.warning(f"Matplotlib unavailable; skipping Task7 attitude plots: {e}")
        return

    if not est_npz.exists():
        return
    try:
        data = np.load(est_npz, allow_pickle=True)
    except Exception as e:
        logger.warning(f"Failed loading estimate NPZ for attitude plots: {e}")
        return

    # Extract time + quaternion (wxyz) from estimator
    time_s = data.get("time_s") or data.get("time")
    quat = data.get("att_quat") or data.get("attitude_q") or data.get("quat_log") or data.get("quat")
    if quat is None or time_s is None:
        logger.info("Estimator NPZ missing quaternion/time; skipping Task7 attitude plots")
        return
    time_s = np.asarray(time_s).ravel()
    quat = np.asarray(quat)
    if quat.shape[0] != time_s.shape[0]:
        n = min(len(time_s), len(quat))
        time_s = time_s[:n]
        quat = quat[:n]

    truth_available = bool(truth_file and truth_file.exists())
    qT = None
    t_truth = None
    if truth_available:
        # Load truth (STATE_X...) flexible parsing similar to run_triad_only _read_state_x001
        try:
            M = np.loadtxt(truth_file, dtype=float)
            if M.ndim != 2 or M.shape[1] < 10:
                raise ValueError("truth file unexpected shape")
            q_truth = M[:, -4:]  # assume qw qx qy qz
            # Determine time column (prefer one whose span < 2e4s)
            cand_cols = [0, 1] if M.shape[1] > 1 else [0]
            t_candidates = []
            for c in cand_cols:
                tcol = M[:, c]
                span = tcol.max() - tcol.min()
                t_candidates.append((abs(span), c, tcol))
            t_candidates.sort(key=lambda x: x[0])
            t_truth = t_candidates[0][2]
            # Zero-base if starts at a large offset
            if t_truth[0] > 1000:
                t_truth = t_truth - t_truth[0]
        except Exception as e:
            logger.warning(f"Failed loading truth for attitude: {e}")
            truth_available = False


    # Helper functions (minimal subset)
    def q_norm(q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, float)
        n = np.linalg.norm(q, axis=1, keepdims=True)
        n[n == 0] = 1.0
        return q / n

    def q_conj(q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, float)
        return np.column_stack([q[:, 0], -q[:, 1], -q[:, 2], -q[:, 3]])

    def quat_angle_deg(qA: np.ndarray, qB: np.ndarray) -> np.ndarray:
        # angle between orientations represented by qA (estimate) and qB (truth)
        qA = q_norm(qA)
        qB = q_norm(qB)
        dots = np.sum(qA * qB, axis=1)
        dots = np.clip(np.abs(dots), 0.0, 1.0)
        return 2 * np.arccos(dots) * 180.0 / np.pi

    def fix_hemisphere(q_ref: np.ndarray, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, float)
        d = np.sum(q * q_ref, axis=1) < 0
        q[d] = -q[d]
        return q


    # Align truth / estimate to common timebase (nearest neighbor for simplicity)
    # Build index mapping estimate times to truth (assume truth has >= coverage)
    try:
        # Use numpy searchsorted for speed
        idx = np.searchsorted(t_truth, time_s)
        idx[idx >= len(t_truth)] = len(t_truth) - 1
        qT = q_truth[idx]
    except Exception:
        # Fallback: truncate to min length
        n = min(len(q_truth), len(quat))
        qT = q_truth[:n]
        quat = quat[:n]
        time_s = time_s[:n]

    qE = q_norm(quat)
    if truth_available and t_truth is not None:
        try:
            idx = np.searchsorted(t_truth, time_s)
            idx[idx >= len(t_truth)] = len(t_truth) - 1
            qT = q_truth[idx]
        except Exception:
            n = min(len(q_truth), len(quat))
            qT = q_truth[:n]
            quat = quat[:n]
            time_s = time_s[:n]
        qT = q_norm(qT)
        # Align hemisphere only; avoid component reordering/conjugation tweaks
        qE = fix_hemisphere(qT, qE)
        ang = quat_angle_deg(qE, qT)
    else:
        ang = None

    generated: List[pathlib.Path] = []

    # Quaternion components plot (always generated)
    plt.figure(figsize=(10, 6))
    labels = ['w', 'x', 'y', 'z']
    for i, lab in enumerate(labels):
        ax = plt.subplot(2, 2, i + 1)
        if truth_available and qT is not None:
            ax.plot(time_s, qT[:, i], '-', label='Truth')
            ax.plot(time_s, qE[:, i], '--', label='KF')
        else:
            ax.plot(time_s, qE[:, i], '-', label='KF')
        ax.set_title(f'q_{lab}')
        ax.grid(True)
    plt.legend(loc='upper right')
    title = f'{tag} Task7_6 (Body→NED): Quaternion Truth vs KF' if truth_available else f'{tag} Task7_6: Quaternion Components'
    plt.suptitle(title)
    data_dict = {'t': time_s, 'q_kf': qE}
    if truth_available and qT is not None:
        data_dict['q_truth'] = qT
    generated.append(
        _save_png_and_mat(
            str(results_dir / f'{tag}_Task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png'),
            data_dict,
        )
    )

    if not truth_available or qT is None:
        logger.info('Generated Task7 attitude plot for %s (no truth available)', tag)
        return

    # Quaternion component residuals
    dq = qE - qT
    plt.figure(figsize=(10, 6))
    for i, lab in enumerate(labels):
        ax = plt.subplot(2, 2, i + 1)
        ax.plot(time_s, dq[:, i], '-', label='Δq est − truth')
        ax.set_title(f'Δq_{lab}')
        ax.grid(True)
        if i == 0:
            ax.legend(loc='upper right')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Quaternion Component Error')
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_BodyToNED_attitude_quaternion_error_components.png'),
                                       {'t': time_s, 'dq_wxyz': dq}))

    # Euler angles (ZYX) using scipy for consistency
    try:
        rotT = R.from_quat(qT[:, [1, 2, 3, 0]])
        rotE = R.from_quat(qE[:, [1, 2, 3, 0]])
        eT = rotT.as_euler('zyx', degrees=True)
        eE = rotE.as_euler('zyx', degrees=True)
    except Exception as e:
        logger.warning(f"Failed Euler conversion: {e}")
        return
    plt.figure(figsize=(10, 6))
    e_labels = ['Yaw(Z)', 'Pitch(Y)', 'Roll(X)']
    for i, lab in enumerate(e_labels):
        ax = plt.subplot(3, 1, i + 1)
        ax.plot(time_s, eT[:, i], '-', label='Truth')
        ax.plot(time_s, eE[:, i], '--', label='KF')
        ax.set_ylabel(lab + ' [deg]')
        ax.grid(True)
        if i == 0:
            ax.legend()
    plt.xlabel('Time [s]')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Euler (ZYX) Truth vs KF')
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_BodyToNED_attitude_truth_vs_estimate_euler.png'),
                                       {'t': time_s, 'e_truth_zyx_deg': eT, 'e_kf_zyx_deg': eE}))

    def _wrap_deg(x: np.ndarray) -> np.ndarray:
        x = (x + 180.0) % 360.0 - 180.0
        x[x == -180.0] = 180.0
        return x
    e_err = _wrap_deg(eE - eT)
    plt.figure(figsize=(10, 6))
    for i, lab in enumerate(e_labels):
        ax = plt.subplot(3, 1, i + 1)
        ax.plot(time_s, e_err[:, i], '-', label='Estimate − Truth')
        ax.set_ylabel(lab + ' err [deg]')
        ax.grid(True)
        if i == 0:
            ax.legend(loc='upper right')
    plt.xlabel('Time [s]')
    plt.suptitle(f'{tag} Task7_6 (Body→NED): Euler Error (ZYX) vs Time')
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_BodyToNED_attitude_euler_error_over_time.png'),
                                       {'t': time_s, 'e_error_zyx_deg': e_err}))

    # Quaternion angle error over time (Task7_6 + back-compat Task7)
    plt.figure(figsize=(10, 3))
    plt.plot(time_s, ang, '-')
    plt.grid(True)
    plt.xlabel('Time [s]')
    plt.ylabel('Angle Error [deg]')
    plt.title(f'{tag} Task7_6: Quaternion Error (angle) vs Time')
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_6_attitude_error_angle_over_time.png'),
                                       {'t': time_s, 'att_err_deg': ang}))
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_attitude_error_angle_over_time.png'),
                                       {'t': time_s, 'att_err_deg': ang}))

    # Console summary tables matching run_triad_only.py
    def _metrics(vec: np.ndarray):
        vec = np.asarray(vec).ravel()
        mean = float(np.mean(vec))
        rmse = float(np.sqrt(np.mean(vec ** 2)))
        p95 = float(np.percentile(np.abs(vec), 95))
        p99 = float(np.percentile(np.abs(vec), 99))
        vmax = float(np.max(np.abs(vec)))
        final = float(vec[-1]) if vec.size else float('nan')
        return mean, rmse, p95, p99, vmax, final

    headers = ["metric", "w", "x", "y", "z"]
    stats_q = {k: _metrics(dq[:, i]) for i, k in enumerate(['w', 'x', 'y', 'z'])}
    rows = []
    for label, idx in [("mean", 0), ("rmse", 1), ("p95_abs", 2), ("p99_abs", 3), ("max_abs", 4), ("final", 5)]:
        rows.append([label] + [f"{stats_q[k][idx]:.6f}" for k in ['w','x','y','z']])
    print("\n===== Task7_6 Quaternion Component Error Summary (est − truth) =====")
    print(" ".join(f"{h:>12s}" for h in headers))
    for r in rows:
        print(" ".join(f"{c:>12s}" for c in r))

    headers_e = ["metric", "yaw", "pitch", "roll"]
    stats_e = {k: _metrics(e_err[:, i]) for i, k in enumerate(['yaw', 'pitch', 'roll'])}
    rows_e = []
    for label, idx in [("mean", 0), ("rmse", 1), ("p95_abs", 2), ("p99_abs", 3), ("max_abs", 4), ("final", 5)]:
        rows_e.append([label] + [f"{stats_e[k][idx]:.6f}" for k in ['yaw','pitch','roll']])
    print("\n===== Task7_6 Euler Error Summary (est − truth) [deg] =====")
    print(" ".join(f"{h:>12s}" for h in headers_e))
    for r in rows_e:
        print(" ".join(f"{c:>12s}" for c in r))

    # Divergence detection utilities
    def _divergence_time(t: np.ndarray, err: np.ndarray, thresh_deg: float, persist_s: float) -> float:
        t = np.asarray(t)
        err = np.asarray(err)
        dt = np.median(np.diff(t)) if t.size > 1 else 0.0
        if dt <= 0:
            return np.nan
        above = err >= thresh_deg
        need = max(1, int(round(persist_s / dt)))
        i = 0
        while i < above.size:
            if above[i]:
                j = i
                while j < above.size and above[j]:
                    j += 1
                if (j - i) >= need:
                    return t[i]
                i = j
            else:
                i += 1
        return np.nan

    div_t = _divergence_time(time_s, ang, div_threshold_deg, div_persist_sec)
    if np.isnan(div_t):
        print("Estimated divergence start time (attitude): none")
    else:
        print(f"Estimated divergence start time (attitude): {div_t:.3f} s")
    with open(results_dir / f'{tag}_Task7_divergence_summary.csv', 'w') as f:
        f.write('threshold_deg,persist_s,divergence_s\n')
        f.write(f"{div_threshold_deg},{div_persist_sec},{'' if np.isnan(div_t) else f'{div_t:.6f}'}\n")

    # Divergence vs length scan
    Ls = [float(x) for x in str(length_scan).split(',') if x.strip()]
    rows = []
    for L in Ls:
        tend = min(time_s[0] + L, time_s[-1])
        m = time_s <= tend
        dv = _divergence_time(time_s[m], ang[m], div_threshold_deg, div_persist_sec)
        rows.append((L, np.nan if (dv != dv) else dv))
    with open(results_dir / f'{tag}_Task7_divergence_vs_length.csv', 'w') as f:
        f.write('tmax_s,divergence_s\n')
        for L, dv in rows:
            f.write(f"{L},{'' if (dv != dv) else dv}\n")
    plt.figure(figsize=(6, 4))
    plt.plot([r[0] for r in rows], [np.nan if (r[1] != r[1]) else r[1] for r in rows], '-o')
    plt.grid(True)
    plt.xlabel('Dataset length used [s]')
    plt.ylabel('Divergence time [s]')
    plt.title(f'{tag} Task7: Divergence vs Length')
    generated.append(_save_png_and_mat(str(results_dir / f'{tag}_Task7_divergence_vs_length.png'),
                                       {'tmax_s': np.array([r[0] for r in rows], float),
                                        'divergence_s': np.array([np.nan if (r[1] != r[1]) else r[1] for r in rows], float)}))
    logger.info("Generated Task7 attitude plots for %s", tag)
    # Copy into per-run subdirectory and create manifest
    try:
        import json, shutil
        subdir_path = results_dir / tag / subdir
        subdir_path.mkdir(parents=True, exist_ok=True)
        unified_task7 = results_dir / tag / 'task7'
        unified_task7.mkdir(parents=True, exist_ok=True)
        manifest = []
        for p in generated:
            for ext in ('.png', '.mat'):
                src = p.with_suffix(ext)
                if src.exists():
                    dst = subdir_path / src.name
                    try:
                        shutil.copy2(src, dst)
                    except Exception:
                        pass
                    # also copy into unified task7 folder
                    try:
                        shutil.copy2(src, unified_task7 / src.name)
                    except Exception:
                        pass
            manifest.append(p.name)
        manifest_path = subdir_path / 'task7_attitude_manifest.json'
        with open(manifest_path, 'w') as f:
            json.dump({
                'tag': tag,
                'generated_pngs': [p.with_suffix('.png').name for p in generated],
                'generated_mats': [p.with_suffix('.mat').name for p in generated],
                'count': len(generated)
            }, f, indent=2)
        logger.info('[TASK 7] Attitude plots (truth vs estimate + errors):')
        for p in generated:
            logger.info('  %s', p.with_suffix('.png').name)
        logger.info('[TASK 7] Attitude manifest: %s', manifest_path)
    except Exception as e:  # pragma: no cover
        logger.warning('Failed to create Task7 attitude manifest: %s', e)
    # close any excess figures to avoid memory leak
    try:
        import matplotlib.pyplot as plt
        plt.close('all')
    except Exception:
        pass


def main(argv=None):
    results_dir = _ensure_results()
    logger.info("Ensured '%s' directory exists.", results_dir)
    parser = argparse.ArgumentParser(
        description="Run GNSS_IMU_Fusion with multiple datasets and methods",
    )
    parser.add_argument(
        "--config",
        help="YAML file specifying datasets and methods",
    )
    parser.add_argument(
        "--include-small",
        action="store_true",
        help="Include *_small dataset variants",
    )
    parser.add_argument(
        "--no-plots",
        action="store_true",
        help="Skip plot generation for faster execution",
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
        help="Enable verbose debug output",
    )
    args = parser.parse_args(argv)

    if args.debug:
        logger.setLevel(logging.DEBUG)

    if args.task == 7:
        from evaluate_filter_results import run_evaluation

    # Task 7 results must be stored directly in ``results/``
        # according to the updated project requirements.
        run_evaluation(
            prediction_file="outputs/predicted_states.csv",
            gnss_file="outputs/gnss_measurements.csv",
            attitude_file="outputs/estimated_attitude.csv",
            save_path=str(results_dir),
        )
        return

    if args.config:
        cases, methods = load_config(args.config)
    else:
        cases, methods = list(DEFAULT_DATASETS), list(DEFAULT_METHODS)
        if args.include_small:
            cases.extend(SMALL_DATASETS)

    logger.debug(f"Datasets: {cases}")
    logger.debug(f"Methods: {methods}")

    results = []
    for (imu, gnss), m in itertools.product(cases, methods):
        imu_path = _imu_path_helper(imu)
        gnss_path = _gnss_path_helper(gnss)
        tag = f"{imu_path.stem}_{gnss_path.stem}_{m}"
        log_path = results_dir / f"{tag}.log"
        print(f"\u25b6 {tag}")

        # Print and save a concise timeline summary for the current dataset.
        try:
            truth_path = resolve_truth_path()
        except Exception:
            truth_path = None
        try:
            print_timeline_summary(tag, str(imu_path), str(gnss_path), truth_path, out_dir=results_dir)
        except Exception:
            # Timeline is best-effort; continue even if it fails
            pass

        if logger.isEnabledFor(logging.DEBUG):
            try:
                gnss_preview = np.loadtxt(
                    gnss_path, delimiter=",", skiprows=1, max_rows=1
                )
                imu_preview = np.loadtxt(imu_path, max_rows=1)
                logger.debug(
                    f"GNSS preview: shape {gnss_preview.shape}, first row: {gnss_preview}"
                )
                logger.debug(
                    f"IMU preview: shape {imu_preview.shape}, first row: {imu_preview}"
                )
            except Exception as e:
                logger.warning(f"Failed data preview for {imu} or {gnss}: {e}")
        cmd = [
            sys.executable,
            str(HERE / "GNSS_IMU_Fusion.py"),
            "--imu-file",
            str(imu_path),
            "--gnss-file",
            str(gnss_path),
            "--method",
            m,
        ]
        # Pass truth to fusion when available and allow dataset mismatches
        try:
            if truth_path and pathlib.Path(truth_path).exists():
                cmd += ["--truth-file", str(truth_path), "--allow-truth-mismatch"]
        except Exception:
            pass
        if args.no_plots:
            cmd.append("--no-plots")
        start_t = time.time()
        ret, summaries = run_case(cmd, log_path)
        runtime = time.time() - start_t
        if ret != 0:
            raise subprocess.CalledProcessError(ret, cmd)
        for summary in summaries:
            kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
            results.append(
                {
                    "dataset": pathlib.Path(imu).stem,
                    "method": kv.get("method", m),
                    "rmse_pos": float(kv.get("rmse_pos", "nan").replace("m", "")),
                    "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                    "rms_resid_pos": float(
                        kv.get("rms_resid_pos", "nan").replace("m", "")
                    ),
                    "max_resid_pos": float(
                        kv.get("max_resid_pos", "nan").replace("m", "")
                    ),
                    "rms_resid_vel": float(
                        kv.get("rms_resid_vel", "nan").replace("m", "")
                    ),
                    "max_resid_vel": float(
                        kv.get("max_resid_vel", "nan").replace("m", "")
                    ),
                    "runtime": runtime,
                }
            )
        # ------------------------------------------------------------------
        # Convert NPZ output to a MATLAB file with explicit frame variables
        # ------------------------------------------------------------------
        npz_path = results_dir / f"{tag}_kf_output.npz"
        if npz_path.exists():
            try:
                data = np.load(npz_path, allow_pickle=True)
            except (OSError, zipfile.BadZipFile) as exc:
                logger.error("Failed to load %s: %s", npz_path, exc)
                continue
            logger.debug(f"Loaded output {npz_path} with keys: {list(data.keys())}")
            time_s = data.get("time_s")
            if time_s is None:
                time_s = data.get("time")

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
                    f"Output time range: {time_s[0] if time_s is not None else 'N/A'}"
                    f" to {time_s[-1] if time_s is not None else 'N/A'} s"
                )
                logger.debug(
                    f"Position shape: {pos_ned.shape if pos_ned is not None else 'None'}"
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
                "method_name": m,
                # Consistent fused data for Tasks 5 and 6
                "fused_pos": pos_ned,
                "fused_vel": vel_ned,
                "quat_log": quat,
            }
            save_mat(npz_path.with_suffix(".mat"), mat_out)

            logger.info(
                "Subtask 6.8.2: Plotted %s position North: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 0],
                pos_ned[-1, 0],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s position East: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 1],
                pos_ned[-1, 1],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s position Down: First = %.4f, Last = %.4f",
                m,
                pos_ned[0, 2],
                pos_ned[-1, 2],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity North: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 0],
                vel_ned[-1, 0],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity East: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 1],
                vel_ned[-1, 1],
            )
            logger.info(
                "Subtask 6.8.2: Plotted %s velocity Down: First = %.4f, Last = %.4f",
                m,
                vel_ned[0, 2],
                vel_ned[-1, 2],
            )

            # ----------------------------
            # Task 6: Truth overlay plots
            # ----------------------------
            truth_file = None
            m_ds = re.search(r"IMU_(X\d+)", imu_path.stem)
            if m_ds:
                ds_id = m_ds.group(1)
                for cand in [
                    ROOT / f"STATE_{ds_id}.txt",
                    ROOT / f"STATE_{ds_id}_small.txt",
                ]:
                    if cand.is_file():
                        truth_file = cand
                        break

            overlay_cmd = [
                sys.executable,
                str(HERE / "task6_plot_truth.py"),
                "--est-file",
                str(npz_path.with_suffix(".mat")),
                "--gnss-file",
                str(gnss_path),
                "--output",
                "results",
            ]
            # Apply Task 6 plotting defaults (decimation + y-limit sync)
            overlay_cmd += ["--decimate-maxpoints", "200000", "--ylim-percentile", "99.5"]
            if truth_file is not None:
                overlay_cmd.extend(["--truth-file", str(truth_file.resolve())])
            if args.show_measurements:
                overlay_cmd.append("--show-measurements")
            with open(log_path, "a") as log:
                log.write("\nTASK 6: Overlay fused output with truth\n")
                msg = "Starting Task 6 overlay ..."
                logger.info(msg)
                log.write(msg + "\n")
                proc = subprocess.Popen(
                    overlay_cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                for line in proc.stdout:
                    print(line, end="")
                    log.write(line)
                proc.wait()

            # ----------------------------
            # Task 7: Evaluation
            # ----------------------------
            # Updated: Task 7 outputs are saved directly in ``results/``
            task7_dir = results_dir
            with open(log_path, "a") as log:
                log.write("\nTASK 7: Evaluate residuals\n")
                msg = "Running Task 7 evaluation ..."
                logger.info(msg)
                log.write(msg + "\n")
                buf = io.StringIO()
                with redirect_stdout(buf):
                    try:
                        run_evaluation_npz(str(npz_path), str(task7_dir), tag)
                    except Exception as e:
                        print(f"Task 7 failed: {e}")
                output = buf.getvalue()
                print(output, end="")
                log.write(output)
                print(
                    f"Saved Task 7.5 diff-truth plots (NED/ECEF/Body) under: results/{tag}/"
                )

            # ----------------------------
            # Task 7.6 Attitude plots (match run_triad_only output set)
            # ----------------------------
            try:
                truth_path_obj = pathlib.Path(truth_file) if 'truth_file' in locals() and truth_file else None
                _task7_attitude_plots(npz_path, truth_path_obj, tag, results_dir)
            except Exception as ex:
                logger.info(f"[WARN] Task7 attitude plots failed for {tag}: {ex}")

            # ----------------------------
            # Attitude comparison (KF vs truth, DR true-init)
            # ----------------------------
            try:
                est_npz = results_dir / f"{tag}_kf_output.npz"
                if est_npz.exists() and truth_path and pathlib.Path(truth_path).exists():
                    att_cmd = [
                        sys.executable,
                        str(HERE / "../scripts/plot_attitude_kf_vs_no_kf.py"),
                        "--imu-file", str(imu_path),
                        "--est-file", str(est_npz),
                        "--truth-file", str(truth_path),
                        "--out-dir", str(results_dir),
                        "--true-init",
                    ]
                    env = os.environ.copy()
                    # Ensure 'src' is importable from the script
                    try:
                        env["PYTHONPATH"] = str(ROOT) + os.pathsep + str(HERE) + os.pathsep + env.get("PYTHONPATH", "")
                    except Exception:
                        pass
                    with open(log_path, "a") as log:
                        logger.info("Running attitude comparison plots: %s", att_cmd)
                        proc = subprocess.Popen(att_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
                        for line in proc.stdout:
                            print(line, end="")
                            log.write(line)
                        proc.wait()
                        print("Generated attitude comparison plots (KF vs truth, DR vs truth).")
                        log.write("Generated attitude comparison plots (KF vs truth, DR vs truth).\n")
            except Exception as ex:
                logger.info(f"[WARN] Attitude comparison plotting failed: {ex}")

    # --- nicely formatted summary table --------------------------------------
    if results:
        key_order = {m: i for i, m in enumerate(methods)}
        results.sort(key=lambda r: (r["dataset"], key_order.get(r["method"], 0)))
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


if __name__ == "__main__":
    main()
