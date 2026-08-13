"""Plotting helpers extracted from the original script."""

import matplotlib.pyplot as plt
from utils.matlab_fig_export import save_matlab_fig
from utils.plot_saver import save_png_and_mat
import numpy as np
from pathlib import Path


def save_zupt_variance(
    accel: np.ndarray,
    zupt_mask: np.ndarray,
    dt: float,
    dataset_id: str,
    threshold: float,
    window_size: int = 100,
    base_dir: str | Path = "results",
    method: str = "",
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
    base = Path(base_dir) / ((f"{method}_" if method else "") + f"IMU_{dataset_id}_task2_2_zupt_variance")
    save_png_and_mat(plt.gcf(), str(base), arrays=dict(t=t, var=var, zupt=zupt_mask.astype(int)))
    save_matlab_fig(plt.gcf(), str(base))
    plt.close()
