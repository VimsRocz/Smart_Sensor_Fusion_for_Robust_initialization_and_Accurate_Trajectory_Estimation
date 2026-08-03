"""Plotting helpers extracted from the original script."""

import matplotlib.pyplot as plt
from utils.matlab_fig_export import save_matlab_fig
from utils.plot_saver import save_png_and_mat
import numpy as np
from typing import Optional, List, Dict
from naming import plot_path
from pathlib import Path


def save_zupt_variance(
    accel: np.ndarray,
    zupt_mask: np.ndarray,
    dt: float,
    dataset_id: str,
    threshold: float,
    window_size: int = 100,
    base_dir: str | Path = "results",
) -> None:
    """Plot ZUPT-detected intervals and accelerometer variance."""
    t = np.arange(accel.shape[0]) * dt
    accel_norm = np.linalg.norm(accel, axis=1)
    mean_conv = np.ones(window_size) / window_size
    var = np.convolve(accel_norm ** 2, mean_conv, mode="same") - np.convolve(
        accel_norm, mean_conv, mode="same"
    ) ** 2
    plt.figure(figsize=(12, 4))
    plt.plot(t, var, label="Accel Norm Variance", color="tab:blue")
    plt.axhline(threshold, color="gray", linestyle="--", label="ZUPT threshold")
    plt.fill_between(
        t,
        0,
        np.max(var),
        where=zupt_mask,
        color="tab:orange",
        alpha=0.3,
        label="ZUPT Detected",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Variance")
    plt.tight_layout()
    plt.title("Task 2 — ZUPT Detection and Accelerometer Variance")
    base = Path(base_dir) / f"IMU_{dataset_id}_task2_2_zupt_variance"
    save_png_and_mat(plt.gcf(), str(base), arrays=dict(t=t, var=var, zupt=zupt_mask.astype(int)))
    save_matlab_fig(plt.gcf(), str(base))
    plt.close()


def save_euler_angles(
    t: np.ndarray,
    euler_angles: np.ndarray,
    dataset_id: str,
    method: str,
    base_dir: str | Path = "results",
) -> None:
    """Plot roll, pitch and yaw over time.

    Parameters
    ----------
    t : np.ndarray
        Time vector corresponding to ``euler_angles``.
    euler_angles : np.ndarray
        Array of roll, pitch and yaw angles in degrees.
    dataset_id : str
        Identifier of the processed dataset.
    method : str
        Name of the attitude initialisation method.
    """
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.title("Task 3 — Attitude Angles (Roll/Pitch/Yaw) vs. Time")
    base = Path(base_dir) / f"{dataset_id}_{method}_task3_7_3_attitude_angles_over_time"
    save_png_and_mat(plt.gcf(), str(base), arrays=dict(t=t, euler=euler_angles))
    save_matlab_fig(plt.gcf(), str(base))
    plt.close()


def save_residual_plots(
    t: np.ndarray,
    pos_filter: np.ndarray,
    pos_gnss: np.ndarray,
    vel_filter: np.ndarray,
    vel_gnss: np.ndarray,
    tag: str,
    base_dir: str | Path = "results",
) -> None:
    """Plot aggregated position and velocity residuals.

    Parameters
    ----------
    t : np.ndarray
        Time vector for the GNSS measurements.
    pos_filter : np.ndarray
        Filtered position in NED frame.
    pos_gnss : np.ndarray
        GNSS derived position in NED frame.
    vel_filter : np.ndarray
        Filtered velocity in NED frame.
    vel_gnss : np.ndarray
        GNSS derived velocity in NED frame.
    tag : str
        Dataset/method tag used as filename prefix.
    """
    residual_pos = pos_filter - pos_gnss
    residual_vel = vel_filter - vel_gnss
    labels = ["North", "East", "Down"]

    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, residual_pos[:, i], label=label)
    plt.xlabel("Time [s]")
    plt.ylabel("Position Residual [m]")
    plt.title("Task 5 — Position Residuals vs. Time")
    plt.legend(loc="best")
    plt.tight_layout()
    base = Path(plot_path(base_dir, tag, 5, "9_1", "position_residuals")).with_suffix("")
    save_png_and_mat(plt.gcf(), str(base), arrays=dict(t=t, resid=residual_pos))
    save_matlab_fig(plt.gcf(), str(base))
    plt.close()

    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, residual_vel[:, i], label=label)
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity Residual [m/s]")
    plt.title("Task 5 — Velocity Residuals vs. Time")
    plt.legend(loc="best")
    plt.tight_layout()
    base = Path(plot_path(base_dir, tag, 5, "9_2", "velocity_residuals")).with_suffix("")
    save_png_and_mat(plt.gcf(), str(base), arrays=dict(t=t, resid=residual_vel))
    save_matlab_fig(plt.gcf(), str(base))
    plt.close()


def save_attitude_over_time(
    t: np.ndarray,
    euler_angles: np.ndarray,
    dataset_id: str,
    method: str,
    base_dir: str | Path = "results",
) -> None:
    """Plot roll, pitch and yaw over the entire dataset.

    Parameters
    ----------
    t : np.ndarray
        Time vector corresponding to ``euler_angles``.
    euler_angles : np.ndarray
        Array of roll, pitch and yaw angles in degrees.
    dataset_id : str
        Identifier of the processed dataset.
    method : str
        Name of the attitude initialisation method.
    """
    plt.figure()
    plt.plot(t, euler_angles[:, 0], label="Roll")
    plt.plot(t, euler_angles[:, 1], label="Pitch")
    plt.plot(t, euler_angles[:, 2], label="Yaw")
    plt.xlabel("Time [s]")
    plt.ylabel("Angle [deg]")
    plt.legend(loc="best")
    plt.tight_layout()
    plt.title("Task 6 — Attitude Angles (Roll/Pitch/Yaw) Over Time")
    base = Path(base_dir) / f"{dataset_id}_{method}_task6_2_attitude_angles_over_time"
    save_png_and_mat(plt.gcf(), str(base), arrays=dict(t=t, euler=euler_angles))
    save_matlab_fig(plt.gcf(), str(base))
    plt.close()


def save_velocity_profile(t: np.ndarray, vel_filter: np.ndarray, vel_gnss: np.ndarray, base_dir: str | Path = "results") -> None:
    """Plot filter and GNSS velocity over time."""
    labels = ["North", "East", "Down"]
    plt.figure(figsize=(10, 5))
    for i, label in enumerate(labels):
        plt.plot(t, vel_gnss[:, i], linestyle="--", label=f"GNSS {label}")
        plt.plot(t, vel_filter[:, i], label=f"Filter {label}")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Task 5 — Velocity Profile")
    plt.legend(loc="best")
    plt.tight_layout()
    save_matlab_fig(plt.gcf(), str(Path(base_dir) / "task5_9_4_velocity_profile"))
    plt.close()


def plot_all_methods(
    imu_time: np.ndarray,
    gnss_pos_ned_interp: np.ndarray,
    gnss_vel_ned_interp: np.ndarray,
    gnss_acc_ned_interp: np.ndarray,
    fused_pos: dict,
    fused_vel: dict,
    fused_acc: dict,
    methods: Optional[List[str]] = None,
    colors: Optional[Dict[str, str]] = None,
    savefile: str = "task5_results_all_methods.png",
) -> None:
    """Plot position, velocity and acceleration for all methods in one figure."""

    if methods is None:
        methods = ["TRIAD", "Davenport", "SVD"]
    if colors is None:
        colors = {"TRIAD": "r", "Davenport": "g", "SVD": "b"}

    directions = ["North", "East", "Down"]
    names = ["Position", "Velocity", "Acceleration"]
    ylabels = ["Position (m)", "Velocity (m/s)", "Acceleration (m/s²)"]

    fig, axes = plt.subplots(3, 3, figsize=(18, 10))

    for row, (truth, data_dict) in enumerate(
        zip(
            [gnss_pos_ned_interp, gnss_vel_ned_interp, gnss_acc_ned_interp],
            [fused_pos, fused_vel, fused_acc],
        )
    ):
        for col in range(3):
            ax = axes[row, col]
            ax.plot(imu_time, truth[:, col], "k-", label="GNSS")
            for m in methods:
                if m not in data_dict:
                    continue
                c = colors.get(m, None)
                ax.plot(imu_time, data_dict[m][:, col], c, alpha=0.8, label=m)
            ax.set_title(f"{names[row]} {directions[col]}")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel(ylabels[row])
            if row == 0 and col == 0:
                ax.legend()

    plt.tight_layout()
    base = Path(savefile).with_suffix("")
    # Flatten dicts into a MAT-friendly structure
    arrays = {"imu_time": imu_time}
    try:
        arrays.update({"gnss_pos": gnss_pos_ned_interp, "gnss_vel": gnss_vel_ned_interp, "gnss_acc": gnss_acc_ned_interp})
        for name, d in [("pos", fused_pos), ("vel", fused_vel), ("acc", fused_acc)]:
            for k, v in d.items():
                arrays[f"{name}_{k}"] = v
    except Exception:
        pass
    save_png_and_mat(plt.gcf(), str(base), arrays=arrays)
    save_matlab_fig(plt.gcf(), str(base))
    plt.close(fig)
