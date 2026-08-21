#!/usr/bin/env python3
"""Run all datasets using a selectable attitude initialisation method.

This script generalises ``run_triad_only.py``. It forwards the chosen
method to ``run_all_datasets.py`` and validates the resulting trajectory
against the bundled ground truth when available. Task 6 overlay plots and
Task 7 residual evaluation are executed automatically when the truth data
is present. The printed summary table matches the one produced by
``run_triad_only.py``.

Usage
-----
    python src/run_method_only.py --method Davenport
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

import subprocess
import sys
from pathlib import Path
import re
import logging
import os

import numpy as np
import pandas as pd
from tabulate import tabulate
from scipy.spatial.transform import Rotation as R
from plot_overlay import plot_overlay
from validate_with_truth import load_estimate, assemble_frames
from evaluate_filter_results import run_evaluation_npz
from pyproj import Transformer
import os
import subprocess as _subprocess

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
from paths import truth_path as _truth_path_helper

logging.basicConfig(level=logging.DEBUG, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# Use the project root for locating the common truth file so the script works
# regardless of the current working directory.
TRUTH_PATH = _truth_path_helper()
EXPECTED_LAT = -32.026554


def load_truth_data(truth_file: Path):
    """Load the common truth trajectory.

    Accept both whitespace- and comma-separated formats. Return
    (time_vector, full_matrix) on success or (None, None) on failure.
    """
    logger.debug(f"Loading truth file: {truth_file}")
    if not truth_file.exists():
        logger.error(f"Truth file {truth_file} not found")
        return None, None
    # Try robust load without forcing a delimiter; fall back to comma if needed
    for kwargs in ({"comments": "#"}, {"delimiter": ",", "comments": "#"}):
        try:
            truth = np.loadtxt(truth_file, **kwargs)
            logger.debug(f"Truth data shape: {truth.shape}")
            return truth[:, 1], truth
        except Exception as e:  # pragma: no cover - try next
            last_err = e
            continue
    logger.error(f"Failed to load truth file: {last_err}")
    return None, None


def trim_truth_to_estimate(truth_time, truth_data, est_time):
    logger.debug("Truth time range: %.3f-%.3f s", truth_time[0], truth_time[-1])
    logger.debug("Estimate time range: %.3f-%.3f s", est_time[0], est_time[-1])
    valid = truth_time <= est_time[-1]
    if not np.any(valid):
        logger.error("No overlapping time range between truth and estimate data")
        return None, None
    t_trim = truth_time[valid]
    d_trim = truth_data[valid]
    logger.debug("Trimmed truth time range: %.3f-%.3f s", t_trim[0], t_trim[-1])
    return t_trim, d_trim


def ecef_to_ned(r0):
    transformer = Transformer.from_crs("EPSG:4978", "EPSG:4326", always_xy=True)
    try:
        lon, lat, h = transformer.transform(*r0)
        logger.debug("Computed lat=%.6f° lon=%.6f° h=%.2f m", lat, lon, h)
        if abs(lat - EXPECTED_LAT) > 0.1:
            logger.warning("Computed latitude %.6f° differs from expected %.6f°", lat, EXPECTED_LAT)
        return lat, lon, h
    except Exception as e:  # pragma: no cover - optional error
        logger.error(f"ECEF-to-NED conversion failed: {e}")
        return None, None, None


def compute_errors(truth_data, est_pos, est_vel, est_eul, truth_time, est_time):
    logger.debug("Computing errors against truth data")
    try:
        pos_i = np.vstack([np.interp(truth_time, est_time, est_pos[:, i]) for i in range(3)]).T
        vel_i = np.vstack([np.interp(truth_time, est_time, est_vel[:, i]) for i in range(3)]).T
        eul_i = np.vstack([np.interp(truth_time, est_time, est_eul[:, i]) for i in range(3)]).T

        pos_err = np.linalg.norm(truth_data[:, 2:5] - pos_i, axis=1)
        vel_err = np.linalg.norm(truth_data[:, 5:8] - vel_i, axis=1)
        truth_eul = R.from_quat(truth_data[:, 8:12][:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
        eul_err = np.linalg.norm(truth_eul - eul_i, axis=1)

        dt = np.diff(truth_time)
        acc_truth = np.diff(truth_data[:, 5:8], axis=0) / dt[:, None]
        acc_est = np.diff(vel_i, axis=0) / dt[:, None]
        acc_err = np.linalg.norm(acc_truth - acc_est, axis=1)

        rmse_pos = float(np.sqrt(np.mean(pos_err**2)))
        final_pos = float(pos_err[-1])
        rmse_vel = float(np.sqrt(np.mean(vel_err**2)))
        final_vel = float(vel_err[-1])
        rmse_acc = float(np.sqrt(np.mean(acc_err**2)))
        final_acc = float(acc_err[-1])
        max_acc = float(np.max(acc_err))
        rmse_eul = float(np.sqrt(np.mean(eul_err**2)))
        final_eul = float(eul_err[-1])

        logger.debug(
            "RMSE pos=%.3f m, Final pos=%.3f m, RMSE vel=%.3f m/s, Final vel=%.3f m/s, "
            "RMSE acc=%.3f m/s^2, Final acc=%.3f m/s^2, Max acc=%.3f m/s^2, "
            "RMSE eul=%.3f deg, Final eul=%.3f deg",
            rmse_pos, final_pos, rmse_vel, final_vel, rmse_acc, final_acc, max_acc, rmse_eul, final_eul,
        )

        return (
            rmse_pos,
            final_pos,
            rmse_vel,
            final_vel,
            rmse_acc,
            final_acc,
            max_acc,
            rmse_eul,
            final_eul,
        )
    except Exception as e:  # pragma: no cover - safety
        logger.error(f"Error computation failed: {e}")
        return (None, None, None, None, None, None, None, None, None)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Run all datasets with one method", allow_abbrev=False)
    parser.add_argument("--method", choices=["TRIAD", "Davenport", "SVD"], default="TRIAD")
    args, remaining = parser.parse_known_args(argv)

    cmd = [sys.executable, str(HERE / "run_all_datasets.py"), "--method", args.method, *remaining]
    subprocess.run(cmd, check=True)

    summary = []

    truth_time, truth_data = load_truth_data(TRUTH_PATH)
    if truth_data is None:
        sys.exit(1)

    est_ref_time = np.arange(0, 1250, 0.0025)
    trimmed_time, trimmed_data = trim_truth_to_estimate(truth_time, truth_data, est_ref_time)
    trimmed_truth_file = Path.cwd() / "results" / "STATE_X001_trimmed.txt"
    np.savetxt(trimmed_truth_file, trimmed_data, fmt="%0.8f")
    logger.debug(f"Trimmed truth saved to {trimmed_truth_file}")

    results = Path.cwd() / "results"
    pattern = f"*_{args.method}_kf_output.mat"
    for mat in results.glob(pattern):
        m = re.match(r"IMU_(X\d+)_.*_" + re.escape(args.method) + r"_kf_output\.mat", mat.name)
        if not m:
            continue
        dataset = m.group(1)
        truth = trimmed_truth_file
        first = np.loadtxt(truth, comments="#", max_rows=1)
        r0 = first[2:5]
        if dataset == "X001":
            r0 = np.array([-3729050.8173, 3935675.6126, -3348394.2576])
        lat_deg, lon_deg, _ = ecef_to_ned(r0)

        vcmd = [
            sys.executable,
            str(HERE / "validate_with_truth.py"),
            "--est-file", str(mat),
            "--truth-file", str(truth),
            "--output", str(results),
            "--ref-lat", str(lat_deg),
            "--ref-lon", str(lon_deg),
            "--ref-r0", str(r0[0]), str(r0[1]), str(r0[2]),
        ]
        proc = subprocess.run(vcmd, check=True, capture_output=True, text=True)
        print(proc.stdout)
        metrics = {}
        for line in proc.stdout.splitlines():
            m_val = re.search(r"Final position error:\s*([0-9.eE+-]+)", line)
            if m_val:
                metrics["final_pos"] = float(m_val.group(1))
            m_val = re.search(r"RMSE position error:\s*([0-9.eE+-]+)", line)
            if m_val:
                metrics["rmse_pos"] = float(m_val.group(1))
            m_val = re.search(r"Final velocity error:\s*([0-9.eE+-]+)", line)
            if m_val:
                metrics["final_vel"] = float(m_val.group(1))
            m_val = re.search(r"RMSE velocity error:\s*([0-9.eE+-]+)", line)
            if m_val:
                metrics["rmse_vel"] = float(m_val.group(1))
            m_val = re.search(r"Final attitude error:\s*([0-9.eE+-]+)", line)
            if m_val:
                metrics["final_att"] = float(m_val.group(1))
            m_val = re.search(r"RMSE attitude error:\s*([0-9.eE+-]+)", line)
            if m_val:
                metrics["rmse_att"] = float(m_val.group(1))

        est_interp = load_estimate(str(mat), times=trimmed_time)
        est_eul = (
            R.from_quat(np.asarray(est_interp["quat"])[:, [1, 2, 3, 0]]).as_euler("xyz", degrees=True)
            if est_interp.get("quat") is not None
            else np.zeros_like(est_interp["pos"])
        )

        (
            rmse_pos, final_pos, rmse_vel, final_vel,
            rmse_acc, final_acc, max_acc, rmse_eul, final_eul,
        ) = compute_errors(
            trimmed_data,
            np.asarray(est_interp["pos"]),
            np.asarray(est_interp["vel"]),
            est_eul,
            trimmed_time,
            trimmed_time,
        )
        logger.debug("%s - interp RMSEpos=%.3f m, final=%.3f m", dataset, rmse_pos, final_pos)
        # Acceleration metrics intentionally omitted from final report
        try:
            t_truth = trimmed_time
            est = load_estimate(str(mat), times=t_truth)
            m2 = re.match(r"(IMU_\w+)_((?:GNSS_)?\w+)_" + re.escape(args.method) + r"_kf_output", mat.stem)
            if m2:
                imu_file = ROOT / f"{m2.group(1)}.dat"
                gnss_file = ROOT / f"{m2.group(2)}.csv"
                frames = assemble_frames(est, imu_file, gnss_file, str(truth))
                for frame_name, data in frames.items():
                    t_i, p_i, v_i, a_i = data["imu"]
                    t_g, p_g, v_g, a_g = data["gnss"]
                    t_f, p_f, v_f, a_f = data["fused"]
                    truth_data = data.get("truth")
                    if truth_data is not None:
                        t_t, p_t, v_t, a_t = truth_data
                    else:
                        t_t = p_t = v_t = a_t = None
                    plot_overlay(
                        frame_name, args.method, t_i, p_i, v_i, a_i,
                        t_g, p_g, v_g, a_g,
                        t_f, p_f, v_f, a_f,
                        results,
                        t_truth=t_t, pos_truth=p_t, vel_truth=v_t, acc_truth=a_t,
                    )

                npz_path = mat.with_suffix('.npz')
                q0 = [float('nan')]*4
                P0 = [float('nan')]*3
                try:
                    npz = np.load(npz_path, allow_pickle=True)
                    if 'attitude_q' in npz:
                        q0 = npz['attitude_q'][0]
                    if 'P_hist' in npz:
                        P0 = np.diagonal(npz['P_hist'][0])[:3]
                except Exception:
                    pass

                # -------- Task 7: Residual evaluation --------
                tag = f"{m2.group(1)}_{m2.group(2)}_{args.method}"
                # Store Task 7 plots directly in ``results/``
                task7_dir = results
                print("Running Task 7 evaluation ...")
                try:
                    run_evaluation_npz(str(npz_path), str(task7_dir), tag)
                except Exception as e:
                    print(f"Task 7 failed: {e}")

                summary.append({
                    'dataset': m.group(1),
                    **metrics,
                    'q0_w': q0[0],
                    'q0_x': q0[1],
                    'q0_y': q0[2],
                    'q0_z': q0[3],
                    'Pxx': P0[0],
                    'Pyy': P0[1],
                    'Pzz': P0[2],
                })
            # --- Task 6 overlay (match TRIAD file convention) ---
            try:
                # Build run_id like TRIAD helper uses
                run_id = f"{pathlib.Path(mat).stem.replace('_kf_output','')}"
                est_npz = mat.with_suffix('.npz')
                if est_npz.exists() and truth and pathlib.Path(truth).exists():
                    cmd_t6 = [
                        sys.executable,
                        str(HERE / "task6_plot_truth.py"),
                        "--est-file", str(mat),  # task6 accepts .mat too
                        "--gnss-file", str(gnss_file),
                        "--truth-file", str(truth),
                        "--output", str(results),
                        "--decimate-maxpoints", "200000",
                        "--ylim-percentile", "99.5",
                    ]
                    print("Starting Task 6 overlay:", cmd_t6)
                    _subprocess.run(cmd_t6, check=True)
            except Exception as ex:
                print(f"Task 6 overlay failed: {ex}")

            # --- Attitude comparison plots (KF vs truth, DR vs truth) ---
            try:
                est_npz = mat.with_suffix('.npz')
                if est_npz.exists() and truth and pathlib.Path(truth).exists():
                    cmd_att = [
                        sys.executable,
                        str(HERE / "../scripts/plot_attitude_kf_vs_no_kf.py"),
                        "--imu-file", str(imu_file),
                        "--est-file", str(est_npz),
                        "--truth-file", str(truth),
                        "--out-dir", str(results),
                        "--true-init",
                    ]
                    env = os.environ.copy()
                    # Ensure 'src' is importable when running the script directly
                    try:
                        env["PYTHONPATH"] = str(ROOT) + os.pathsep + str(HERE) + os.pathsep + env.get("PYTHONPATH", "")
                    except Exception:
                        pass
                    print("Running attitude comparison plots:", cmd_att)
                    _subprocess.run(cmd_att, check=True, env=env)
                    print("Generated attitude comparison plots (KF vs truth, DR vs truth).")
            except Exception as ex:
                print(f"Attitude comparison plotting failed: {ex}")

        except Exception as e:
            print(f"Overlay plot failed: {e}")

    if summary:
        rows = [
            [s.get('dataset'), s.get('rmse_pos'), s.get('final_pos'), s.get('rmse_vel'),
             s.get('final_vel'),
             s.get('rmse_att'), s.get('final_att'),
             s.get('q0_w'), s.get('q0_x'), s.get('q0_y'), s.get('q0_z'),
             s.get('Pxx'), s.get('Pyy'), s.get('Pzz')]
            for s in summary
        ]
        headers = [
            'Dataset', 'RMSEpos[m]', 'FinalPos[m]', 'RMSEvel[m/s]', 'FinalVel[m/s]',
            'RMSEatt[deg]', 'FinalAtt[deg]', 'q0_w', 'q0_x', 'q0_y', 'q0_z',
            'Pxx', 'Pyy', 'Pzz'
        ]
        print(tabulate(rows, headers=headers, floatfmt='.3f'))
        df = pd.DataFrame(rows, columns=headers)
        df.to_csv(results / 'summary_truth.csv', index=False)


if __name__ == "__main__":
    main()
