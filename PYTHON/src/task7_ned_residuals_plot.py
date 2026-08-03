"""Task 7 -- Plot residuals in the NED frame.

Usage:
    python src/task7_ned_residuals_plot.py --est-file fused.npz \
        --truth-file STATE_X001.txt --dataset IMU_GNSS_X001 \
        --output-dir results

This implements the functionality of ``task7_ned_residuals_plot.m`` from the
MATLAB code base. The estimator time vector is shifted to start at zero so that
Task 6 and Task 7 plots share the same x-axis. Figures are written under
``results/<dataset>/``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from src.utils import compute_C_ECEF_to_NED


def load_est_ned(
    file: Path,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float, np.ndarray]:
    """Load estimator output and convert to NED if required."""
    data = np.load(file)
    t = data.get("time_s")
    if t is None:
        t = data.get("time")
    if t is None:
        raise KeyError("Missing time vector in estimator file")
    # GNSS_IMU_Fusion.py writes the unsuffixed names; accept both spellings.
    pos_ned = data.get("pos_ned_m")
    if pos_ned is None:
        pos_ned = data.get("pos_ned")
    vel_ned = data.get("vel_ned_ms")
    if vel_ned is None:
        vel_ned = data.get("vel_ned")
    acc_ned = data.get("acc_ned_ms2")
    if acc_ned is None:
        acc_ned = data.get("fused_acc")

    lat = data.get("ref_lat_rad")
    if lat is None:
        lat = data.get("ref_lat")
    lon = data.get("ref_lon_rad")
    if lon is None:
        lon = data.get("ref_lon")
    r0 = data.get("ref_r0_m")
    if r0 is None:
        r0 = data.get("ref_r0")

    if lat is None or lon is None or r0 is None:
        raise KeyError("Estimator file missing reference location")

    # ref_lat/ref_lon are stored as 1-element arrays; NumPy 2 refuses float()
    # on those, so flatten before converting.
    lat = float(np.asarray(lat).reshape(-1)[0])
    lon = float(np.asarray(lon).reshape(-1)[0])
    r0 = np.asarray(r0).reshape(3)

    if pos_ned is None or vel_ned is None:
        pos_ecef = data["pos_ecef_m"]
        vel_ecef = data["vel_ecef_ms"]
        C = compute_C_ECEF_to_NED(lat, lon)
        pos_ned = np.array([C @ (p - r0) for p in pos_ecef])
        vel_ned = np.array([C @ v for v in vel_ecef])

    if acc_ned is None:
        acc_ned = np.gradient(vel_ned, t, axis=0)

    return t, pos_ned, vel_ned, acc_ned, lat, lon, r0


def load_truth_ned(
    file: Path, lat: float, lon: float, r0: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load truth trajectory and convert to NED."""
    if file.suffix == ".npz":
        d = np.load(file)
        t = d["time_s"]
        pos_ecef = d["pos_ecef_m"]
        vel_ecef = d["vel_ecef_ms"]
        acc_ecef = d.get("acc_ecef_ms2")
        if acc_ecef is None:
            acc_ecef = np.gradient(vel_ecef, t, axis=0)
    else:
        raw = np.loadtxt(file)
        t = raw[:, 0]
        pos_ecef = raw[:, 1:4]
        vel_ecef = raw[:, 4:7]
        acc_ecef = np.vstack(
            (np.zeros(3), np.diff(vel_ecef, axis=0) / np.diff(t)[:, None])
        )

    C = compute_C_ECEF_to_NED(lat, lon)
    pos_ned = np.array([C @ (p - r0) for p in pos_ecef])
    vel_ned = np.array([C @ v for v in vel_ecef])
    acc_ned = np.array([C @ a for a in acc_ecef])
    return t, pos_ned, vel_ned, acc_ned


def compute_residuals(
    t: np.ndarray,
    est_pos: np.ndarray,
    est_vel: np.ndarray,
    truth_pos: np.ndarray,
    truth_vel: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return position, velocity and acceleration residuals."""
    res_pos = est_pos - truth_pos
    res_vel = est_vel - truth_vel
    acc_est = np.gradient(est_vel, t, axis=0)
    acc_truth = np.gradient(truth_vel, t, axis=0)
    res_acc = acc_est - acc_truth
    return res_pos, res_vel, res_acc


def plot_residuals(
    t: np.ndarray,
    res_pos: np.ndarray,
    res_vel: np.ndarray,
    res_acc: np.ndarray,
    dataset: str,
    out_dir: Path,
) -> None:
    """Plot residual components and norms in the NED frame."""
    labels = ["North", "East", "Down"]
    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)

    for i, (arr, ylabel) in enumerate(
        [
            (res_pos, "Position Residual [m]"),
            (res_vel, "Velocity Residual [m/s]"),
            (res_acc, "Acceleration Residual [m/s$^2$]"),
        ]
    ):
        for j in range(3):
            ax = axes[i, j]
            ax.plot(t, arr[:, j])
            if i == 0:
                ax.set_title(labels[j])
            if j == 0:
                ax.set_ylabel(ylabel)
            if i == 2:
                ax.set_xlabel("Time [s]")
            ax.grid(True)

    fig.suptitle(f"{dataset} Task 7.3 — NED Residuals")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    out_dir.mkdir(parents=True, exist_ok=True)
    pdf = out_dir / f"{dataset}_task7_3_ned_residuals.pdf"
    png = out_dir / f"{dataset}_task7_3_ned_residuals.png"
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(Path(pdf).with_suffix("")))
    try:
        from utils import save_plot_mat
        save_plot_mat(fig, str(out_dir / f"{dataset}_task7_3_ned_residuals.mat"))
    except Exception:
        pass
    plt.close(fig)

    fig, ax = plt.subplots()
    ax.plot(t, np.linalg.norm(res_pos, axis=1), label="|pos|")
    ax.plot(t, np.linalg.norm(res_vel, axis=1), label="|vel|")
    ax.plot(t, np.linalg.norm(res_acc, axis=1), label="|acc|")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Residual Norm")
    ax.legend()
    ax.grid(True)
    fig.suptitle(f"{dataset} Task 7.4 — NED Residual Norms")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    norm_pdf = out_dir / f"{dataset}_task7_4_ned_residual_norms.pdf"
    norm_png = out_dir / f"{dataset}_task7_4_ned_residual_norms.png"
    from utils.matlab_fig_export import save_matlab_fig
    save_matlab_fig(fig, str(Path(norm_pdf).with_suffix("")))
    try:
        from utils import save_plot_mat
        save_plot_mat(fig, str(out_dir / f"{dataset}_task7_4_ned_residual_norms.mat"))
    except Exception:
        pass
    plt.close(fig)

    saved = sorted(out_dir.glob(f"{dataset}_task7_*_ned_residual*.pdf"))
    if saved:
        print("Files saved in", out_dir)
        for f in saved:
            print(" -", f.name)


def main() -> None:
    ap = argparse.ArgumentParser(description="Plot NED residuals for Task 7")
    ap.add_argument("--est-file", type=Path, required=True, help="fused estimator npz")
    ap.add_argument("--truth-file", type=Path, required=True, help="ground truth file")
    ap.add_argument("--dataset", required=True, help="dataset identifier")
    ap.add_argument(
        "--output-dir", type=Path, default=Path("results"), help="output base directory"
    )
    args = ap.parse_args()

    t_est, pos_est, vel_est, _acc_est, lat, lon, r0 = load_est_ned(args.est_file)
    t_truth, pos_truth, vel_truth, _ = load_truth_ned(args.truth_file, lat, lon, r0)

    # Use relative time for plotting to align with Task 6
    t_rel = t_est - t_est[0]

    pos_truth_i = np.vstack(
        [np.interp(t_est, t_truth, pos_truth[:, i]) for i in range(3)]
    ).T
    vel_truth_i = np.vstack(
        [np.interp(t_est, t_truth, vel_truth[:, i]) for i in range(3)]
    ).T

    res_pos, res_vel, res_acc = compute_residuals(
        t_rel, pos_est, vel_est, pos_truth_i, vel_truth_i
    )

    out_dir = args.output_dir
    plot_residuals(t_rel, res_pos, res_vel, res_acc, args.dataset, out_dir)
    saved = sorted(out_dir.glob(f"{args.dataset}_task7_*_ned_residual*.pdf"))
    if saved:
        print("Files saved in", out_dir)
        for f in saved:
            print(" -", f.name)


if __name__ == "__main__":
    main()
