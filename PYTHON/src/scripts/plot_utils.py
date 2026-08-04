import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from typing import List
import numpy as np
from ..utils.plot_save import save_plot as save_and_log
from ..utils.matlab_fig_export import save_matlab_fig


def save_plot(fig, outpath, title):
    ax = fig.axes[0] if fig.axes else fig.add_subplot(111)
    ax.set_title(title, fontsize=14)
    fig.tight_layout()
    save_matlab_fig(fig, str(outpath))
    plt.close(fig)


def plot_attitude(time, quats, results_dir, run_id, method):
    r = R.from_quat(quats)
    euler = r.as_euler('xyz', degrees=True)
    fig, axs = plt.subplots(3, 1, figsize=(6, 8))
    labels = ['Roll', 'Pitch', 'Yaw']
    for i in range(3):
        axs[i].plot(time, euler[:, i])
        axs[i].set_ylabel(f"{labels[i]} (°)")
    axs[-1].set_xlabel("Time (s)")
    fig.suptitle("Task 5: Attitude Angles Over Time")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    save_and_log(fig, results_dir, run_id, "task5_8_1", "attitude_BodyToNED")
    plt.close(fig)



def plot_zupt_detection(accel_data: np.ndarray, gyro_data: np.ndarray,
                        zupt_mask: np.ndarray, sampling_period: float):
    """Plot acceleration and gyro norms with highlighted ZUPT segments."""
    t = np.arange(len(accel_data)) * sampling_period
    accel_norm = np.linalg.norm(accel_data, axis=1)
    gyro_norm = np.linalg.norm(gyro_data, axis=1)

    fig, axs = plt.subplots(2, 1, sharex=True)
    axs[0].plot(t, accel_norm, label="|accel|")
    axs[1].plot(t, gyro_norm, label="|gyro|")
    axs[0].set_ylabel("Accel Norm [m/s²]")
    axs[1].set_ylabel("Gyro Norm [rad/s]")
    axs[1].set_xlabel("Time [s]")

    for ax in axs:
        ax.fill_between(t, ax.get_ylim()[0], ax.get_ylim()[1],
                        where=zupt_mask, color='green', alpha=0.1,
                        label='ZUPT active')
        ax.legend()
        ax.grid(True)

    fig.tight_layout()
    return fig


def plot_initial_attitude(triad_rotation_matrices: dict):
    """Print and plot initial Euler angles from TRIAD results."""
    results = {}
    for ds, R_b2n in triad_rotation_matrices.items():
        rot = R.from_matrix(R_b2n)
        euler_deg = rot.as_euler('zyx', degrees=True)
        results[ds] = euler_deg
        fig, ax = plt.subplots()
        ax.bar(['Yaw', 'Pitch', 'Roll'], euler_deg)
        ax.set_ylabel('Degrees')
        ax.set_title(f'Initial Attitude (TRIAD) for {ds}')
        fig.tight_layout()
        plt.close(fig)
        print(
            f"Dataset {ds}: Initial Yaw={euler_deg[0]:.2f}°, "
            f"Pitch={euler_deg[1]:.2f}°, Roll={euler_deg[2]:.2f}°"
        )
    return results


def plot_zupt_and_variance(accel: np.ndarray, zupt_mask: np.ndarray, dt: float,
                           threshold: float, window_size: int = 100):
    """Plot rolling variance of the acceleration norm with ZUPT intervals.

    Parameters
    ----------
    accel:
        Raw accelerometer samples ``(N,3)``.
    zupt_mask:
        Boolean mask marking detected ZUPT intervals.
    dt:
        Sampling period in seconds.
    threshold:
        Variance threshold used for detection.
    window_size:
        Length of the moving window in samples.

    Returns
    -------
    matplotlib.figure.Figure
        The generated figure.
    """
    t = np.arange(accel.shape[0]) * dt
    accel_norm = np.linalg.norm(accel, axis=1)
    mean_conv = np.ones(window_size) / window_size
    var = (
        np.convolve(accel_norm ** 2, mean_conv, mode="same")
        - np.convolve(accel_norm, mean_conv, mode="same") ** 2
    )

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, var, label="Accel Norm Variance", color="tab:blue")
    ax.axhline(threshold, color="gray", linestyle="--", label="ZUPT threshold")
    ax.fill_between(
        t,
        0,
        np.max(var),
        where=zupt_mask,
        color="tab:orange",
        alpha=0.3,
        label="ZUPT Detected",
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Variance")
    ax.set_title("ZUPT Detected Intervals & Accelerometer Variance")
    ax.legend()
    fig.tight_layout()
    return fig


def plot_triad_euler(triad_rotmats: List[np.ndarray], dataset_names: List[str]):
    """Plot TRIAD initial Euler angles (roll, pitch, yaw) for multiple datasets."""
    eulers = [R.from_matrix(Rm).as_euler("zyx", degrees=True) for Rm in triad_rotmats]
    eulers = np.asarray(eulers)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(dataset_names, eulers[:, 2], marker="o", label="Roll [°]")
    ax.plot(dataset_names, eulers[:, 1], marker="o", label="Pitch [°]")
    ax.plot(dataset_names, eulers[:, 0], marker="o", label="Yaw [°]")
    ax.set_ylabel("Angle [deg]")
    ax.set_title("TRIAD Attitude Initialization (Euler Angles)")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    return fig


def plot_pva_grid(
    time: np.ndarray,
    pos: np.ndarray,
    vel: np.ndarray,
    acc: np.ndarray,
    frame: str = "NED",
) -> plt.Figure:
    """Return a 3×3 figure of position, velocity and acceleration.

    Parameters
    ----------
    time : np.ndarray
        Time stamps for the samples.
    pos, vel, acc : np.ndarray
        Arrays of shape ``(N, 3)`` containing position, velocity and
        acceleration components.
    frame : str, optional
        Coordinate frame name used to label the axes. Defaults to ``"NED"``.
    """

    axis_labels = {
        "NED": ["N", "E", "D"],
        "ECEF": ["X", "Y", "Z"],
        "Body": ["X", "Y", "Z"],
    }
    cols = axis_labels.get(frame, ["X", "Y", "Z"])

    def ensure_array(arr: np.ndarray) -> np.ndarray:
        n = len(time)
        if arr is None or arr.size == 0 or arr.ndim != 2 or arr.shape[1] < 3 or arr.shape[0] != n:
            return np.full((n, 3), np.nan)
        return arr

    pos = ensure_array(pos)
    vel = ensure_array(vel)
    acc = ensure_array(acc)

    fig, axes = plt.subplots(3, 3, figsize=(12, 9), sharex=True)
    datasets = [
        (pos, "Position [m]"),
        (vel, "Velocity [m/s]"),
        (acc, "Acceleration [m/s$^2$]"),
    ]

    for row, (arr, ylab) in enumerate(datasets):
        for col, axis in enumerate(cols):
            ax = axes[row, col]
            ax.plot(time, arr[:, col])
            if row == 0:
                ax.set_title(axis)
            if col == 0:
                ax.set_ylabel(ylab)
            if row == 2:
                ax.set_xlabel("Time [s]")
            ax.grid(True)

    fig.tight_layout()
    return fig
