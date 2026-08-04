import numpy as np
import matplotlib.pyplot as plt


def compute_residuals(gnss_times, gnss_pos, filt_times, filt_pos):
    interp = np.vstack([
        np.interp(gnss_times, filt_times, filt_pos[:, i])
        for i in range(3)
    ]).T
    res = interp - gnss_pos
    return res


from utils.plot_save import save_plot


def plot_residuals(gnss_times, res, results_dir, run_id, method):
    fig, axs = plt.subplots(2, 1, figsize=(8, 6))
    axs[0].plot(gnss_times, res[:, 0], label='North')
    axs[0].plot(gnss_times, res[:, 1], label='East')
    axs[0].plot(gnss_times, res[:, 2], label='Down')
    axs[0].set_ylabel('Position residual (m)')
    axs[0].set_title('Position Residuals (N, E, D)')
    axs[0].legend()

    axs[1].plot(
        gnss_times[:-1],
        np.diff(res, axis=0) / np.diff(gnss_times)[:, None]
    )
    axs[1].set_ylabel('Velocity residual (m/s)')
    axs[1].set_title('Velocity Residuals (N, E, D)')
    axs[1].set_xlabel('Time (s)')

    fig.suptitle('Position & Velocity Residuals')
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    save_plot(fig, results_dir, run_id, "task5_9_3", "filter_residuals_NED")
    plt.close(fig)
