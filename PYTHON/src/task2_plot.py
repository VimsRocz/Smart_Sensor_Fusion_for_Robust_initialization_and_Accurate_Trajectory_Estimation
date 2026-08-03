from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import uuid
import os
from utils.plot_save import save_plot, task_summary
from utils.plot_saver import save_png_and_mat


def save_task2_summary_png(
    imu_data: np.ndarray,
    static_start: int,
    static_end: int,
    g_body: np.ndarray,
    omega_ie_body: np.ndarray,
    run_id: str,
    out_dir: str | Path,
) -> Path:
    """Save a Task 2 summary figure.

    The figure contains the accelerometer and gyroscope norms over time with the
    detected static interval highlighted as well as bar charts of the measured
    gravity and Earth rotation vectors in the body frame.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    time = imu_data[:, 1]
    gyro = imu_data[:, 2:5]
    acc = imu_data[:, 5:8]

    if len(time) > 1:
        dt_candidates = np.diff(time[: min(400, len(time))])
        dt = float(np.mean(dt_candidates[dt_candidates > 0]))
    else:
        dt = 1.0 / 400.0

    t = np.arange(len(time)) * dt
    gyro = gyro / dt
    acc = acc / dt
    acc_norm = np.linalg.norm(acc, axis=1)
    gyro_norm = np.linalg.norm(gyro, axis=1)

    fig, axes = plt.subplots(2, 2, figsize=(10, 6))

    ax = axes[0, 0]
    keep = t > 1.0
    if np.any(keep):
        p99 = np.nanpercentile(acc_norm[keep], 99.5)
    else:
        p99 = np.nanpercentile(acc_norm, 99.5)
    ax.plot(t, acc_norm)
    ax.set_ylim(0, max(20, p99))
    ax.axvspan(t[static_start], t[static_end], color="red", alpha=0.3)
    ax.set_ylabel("|acc| [m/s²]")
    ax.set_title("Accelerometer norm")

    ax = axes[1, 0]
    ax.plot(t, gyro_norm)
    ax.axvspan(t[static_start], t[static_end], color="red", alpha=0.3)
    ax.set_ylabel("|gyro| [rad/s]")
    ax.set_xlabel("Time [s]")
    ax.set_title("Gyroscope norm")

    axes[0, 1].bar(["x", "y", "z"], g_body)
    axes[0, 1].set_title("g_body [m/s²]")

    axes[1, 1].bar(["x", "y", "z"], omega_ie_body)
    axes[1, 1].set_title("omega_ie_body [rad/s]")

    fig.suptitle("Task 2 – Body-frame vector summary")
    fig.tight_layout(rect=[0, 0, 1, 0.95])

    base = out_dir / f"{run_id}_task2_2_static_interval"
    save_png_and_mat(
        fig,
        str(base),
        arrays=dict(
            t=t,
            acc_norm=acc_norm,
            gyro_norm=gyro_norm,
            static_start=np.array([static_start]),
            static_end=np.array([static_end]),
            g_body=g_body,
            omega_ie_body=omega_ie_body,
        ),
    )
    out_png = base.with_suffix('.png')
    plt.close(fig)
    task_summary("task2")
    return out_png


def task2_measure_body_vectors(
    imu_data,
    static_indices: tuple[int, int],
    output_dir: str | Path,
    run_id: str,
) -> Path:
    """Plot measured gravity and Earth rotation vectors with error bars."""

    start, end = static_indices
    plot_id = uuid.uuid4().hex
    print(f"Task 2 plot ID: {plot_id}")
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        acc_cols = ["accel_x", "accel_y", "accel_z"]
        gyro_cols = ["gyro_x", "gyro_y", "gyro_z"]
        static_df = imu_data.iloc[start:end]
        time = imu_data["time"].to_numpy()
        if len(time) > 1:
            dt_candidates = np.diff(time[: min(400, len(time))])
            dt = float(np.mean(dt_candidates[dt_candidates > 0]))
        else:
            dt = 1.0 / 400.0
        acc = static_df[acc_cols].to_numpy() / dt
        gyro = static_df[gyro_cols].to_numpy() / dt
        g_body = -np.mean(acc, axis=0)
        omega_ie_body = np.mean(gyro, axis=0)
        acc_err = np.std(acc, axis=0)
        gyro_err = np.std(gyro, axis=0)
        errors = np.concatenate([acc_err, gyro_err])
    except Exception:
        g_body = np.zeros(3)
        omega_ie_body = np.zeros(3)
        errors = np.full(6, 1e-6)
        print("Error data not available; using defaults")
    else:
        if np.all(errors == 0):
            errors = np.full(6, 1e-6)
            print("Error data not available; using defaults")

    labels = [
        "Gravity X",
        "Gravity Y",
        "Gravity Z",
        "Earth Rot X",
        "Earth Rot Y",
        "Earth Rot Z",
    ]
    values = np.concatenate([g_body, omega_ie_body])

    fig, ax = plt.subplots(figsize=(8, 4))
    x = np.arange(len(labels))
    bars = ax.bar(x, values, yerr=errors, capsize=5)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right")
    ax.set_ylabel("Value")
    ax.set_title(
        f"Task 2: Measured vectors in body frame\nID: {plot_id}"
    )

    for bar, val in zip(bars, values):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height(),
            f"{val:.2e}",
            ha="center",
            va="bottom",
            fontsize=8,
        )

    fig.tight_layout()
    base = out_dir / f"{run_id}_task2_3_vectors"
    save_png_and_mat(
        fig,
        str(base),
        arrays=dict(g_body=g_body, omega_ie_body=omega_ie_body, errors=errors),
    )
    out_png = base.with_suffix('.png')
    plt.close(fig)
    task_summary("task2")
    return out_png
