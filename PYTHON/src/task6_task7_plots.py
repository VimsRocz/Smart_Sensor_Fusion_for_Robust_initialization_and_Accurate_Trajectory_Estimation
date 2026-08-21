"""Task 6 and Task 7 comparison plots in all three reference frames.

Task 6 overlays the fused solution on truth in NED, ECEF and Body, and compares
the Body-to-NED and Body-to-ECEF attitude quaternions.

Task 7.5 plots the fused-minus-truth difference over time in each frame.
Task 7.6 produces the four attitude comparison figures.

Every figure is written as PNG and PDF through ``save_matlab_fig`` so the naming
and the optional MATLAB ``.fig`` mirror stay consistent with the rest of the
pipeline.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from utils.matlab_fig_export import save_matlab_fig  # noqa: E402

NED_LABELS = ("North", "East", "Down")
ECEF_LABELS = ("X_ECEF", "Y_ECEF", "Z_ECEF")
BODY_LABELS = ("X_body", "Y_body", "Z_body")
QUAT_LABELS = ("qw", "qx", "qy", "qz")

FRAME_LABELS = {"ned": NED_LABELS, "ecef": ECEF_LABELS, "body": BODY_LABELS}


def _ensure_matlab_helper(out_dir: Path) -> None:
    """Copy show_task_plot.m beside the .mat files so MATLAB can find it."""
    try:
        src = Path(__file__).resolve().parents[2] / "MATLAB" / "show_task_plot.m"
        dst = Path(out_dir) / "show_task_plot.m"
        if src.is_file() and (not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime):
            dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    except Exception:
        pass


def _save(fig, out_dir: Path, stem: str, arrays: dict | None = None) -> None:
    """Write PNG + PDF, and a .mat holding the plotted arrays for MATLAB."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if arrays:
        try:
            from scipy.io import savemat

            savemat(
                str(out_dir / f"{stem}.mat"),
                {k: np.asarray(v) for k, v in arrays.items()},
                do_compression=True,
            )
            _ensure_matlab_helper(out_dir)
            print(f"[MAT ] {out_dir / stem}.mat keys={sorted(arrays)}")
        except Exception as exc:  # pragma: no cover
            print(f"[WARN] could not write {stem}.mat: {exc}")
    save_matlab_fig(fig, str(out_dir / stem))
    plt.close(fig)


def _to_frame(pos_ned, vel_ned, frame, C_ECEF_to_NED, ref_r0, C_B_N):
    """Express an NED position/velocity pair in ``frame``."""
    if frame == "ned":
        return pos_ned, vel_ned
    if frame == "ecef":
        C = np.asarray(C_ECEF_to_NED).T          # NED -> ECEF
        return (C @ pos_ned.T).T + np.asarray(ref_r0), (C @ vel_ned.T).T
    if frame == "body":
        C = np.asarray(C_B_N).T                  # NED -> body
        return (C @ pos_ned.T).T, (C @ vel_ned.T).T
    raise ValueError(f"unknown frame {frame!r}")


# ---------------------------------------------------------------------------
# Task 6 — fused vs truth in each frame
# ---------------------------------------------------------------------------
def task6_fused_vs_truth(
    tag, t, fused_pos_ned, fused_vel_ned, truth_pos_ned, truth_vel_ned,
    C_ECEF_to_NED, ref_r0, C_B_N, out_dir, subtask_start=1,
):
    """One 3x2 position/velocity overlay per reference frame."""
    written = []
    for offset, frame in enumerate(("ned", "ecef", "body")):
        fp, fv = _to_frame(fused_pos_ned, fused_vel_ned, frame, C_ECEF_to_NED, ref_r0, C_B_N)
        tp, tv = _to_frame(truth_pos_ned, truth_vel_ned, frame, C_ECEF_to_NED, ref_r0, C_B_N)
        labels = FRAME_LABELS[frame]

        fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
        for i in range(3):
            ax = axes[i, 0]
            ax.plot(t, tp[:, i], "m-", label="TRUTH")
            ax.plot(t, fp[:, i], "b--", label="FUSED")
            ax.set_ylabel(f"Pos {labels[i]} [m]")
            ax.grid(True)
            if frame == "ecef":
                ax.ticklabel_format(axis="y", style="plain", useOffset=False)

            ax = axes[i, 1]
            ax.plot(t, tv[:, i], "m-", label="TRUTH")
            ax.plot(t, fv[:, i], "b--", label="FUSED")
            ax.set_ylabel(f"Vel {labels[i]} [m/s]")
            ax.grid(True)
        axes[2, 0].set_xlabel("Time [s]")
        axes[2, 1].set_xlabel("Time [s]")
        axes[0, 0].legend(loc="best")
        sub = subtask_start + offset
        fig.suptitle(f"Task 6.{sub} — {frame.upper()} frame — TRUTH vs FUSED")
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        stem = f"{tag}_task6_{sub}_fused_vs_truth_{frame}"
        _save(fig, out_dir, stem, arrays=dict(
            t=t, pos_fused=fp, pos_truth=tp, vel_fused=fv, vel_truth=tv,
            frame=frame, labels=list(labels),
            plot_title=f"Task 6.{sub} - {frame.upper()} frame - TRUTH vs FUSED",
            x_label="Time [s]", col_names=list(labels),
            y_labels=["Position fused [m]", "Position truth [m]",
                      "Velocity fused [m/s]", "Velocity truth [m/s]"]))
        written.append(stem)
    return written


def task6_quaternion_comparison(
    tag, t, quat_fused_b2n, quat_truth_b2n, C_ECEF_to_NED, out_dir, subtask=4,
):
    """Body-to-NED and Body-to-ECEF quaternion components, truth vs fused."""
    from scipy.spatial.transform import Rotation as R

    written = []
    # NED -> ECEF is a fixed rotation for this run, so composing it gives the
    # Body-to-ECEF attitude from the Body-to-NED one.
    R_e_n = R.from_matrix(np.asarray(C_ECEF_to_NED).T)

    # A quaternion is a rotation BETWEEN frames, so it is named for the pair it
    # maps, not for a single frame.
    for offset, frame in enumerate(("BodyToNED", "BodyToECEF")):
        if frame == "BodyToNED":
            qf, qt = quat_fused_b2n, quat_truth_b2n
        else:
            qf = (R_e_n * R.from_quat(_wxyz_to_xyzw(quat_fused_b2n))).as_quat()
            qt = (R_e_n * R.from_quat(_wxyz_to_xyzw(quat_truth_b2n))).as_quat()
            qf, qt = _xyzw_to_wxyz(qf), _xyzw_to_wxyz(qt)

        qf = _align_sign(qt, qf)
        fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
        for i, ax in enumerate(axes.flat):
            ax.plot(t, qt[:, i], "m-", label="TRUTH")
            ax.plot(t, qf[:, i], "b--", label="FUSED")
            ax.set_ylabel(QUAT_LABELS[i])
            ax.grid(True)
            ax.ticklabel_format(axis="y", style="plain", useOffset=False)
        axes[1, 0].set_xlabel("Time [s]")
        axes[1, 1].set_xlabel("Time [s]")
        axes[0, 0].legend(loc="best")
        sub = subtask + offset
        pretty = "Body->NED" if frame == "BodyToNED" else "Body->ECEF"
        fig.suptitle(f"Task 6.{sub} — {pretty} attitude quaternion — TRUTH vs FUSED")
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        stem = f"{tag}_task6_{sub}_attitude_quaternion_{frame}"
        _save(fig, out_dir, stem, arrays=dict(
            t=t, quat_fused_wxyz=qf, quat_truth_wxyz=qt, rotation=frame,
            plot_title=f"Task 6.{sub} - {pretty} attitude quaternion - TRUTH vs FUSED",
            x_label="Time [s]", col_names=list(QUAT_LABELS),
            y_labels=["Fused quaternion", "Truth quaternion"]))
        written.append(stem)
    return written


# ---------------------------------------------------------------------------
# Task 7.5 — fused minus truth over time, per frame
# ---------------------------------------------------------------------------
def task7_5_diff_over_time(
    tag, t, fused_pos_ned, fused_vel_ned, truth_pos_ned, truth_vel_ned,
    C_ECEF_to_NED, ref_r0, C_B_N, out_dir,
):
    written = []
    for frame in ("ned", "ecef", "body"):
        fp, fv = _to_frame(fused_pos_ned, fused_vel_ned, frame, C_ECEF_to_NED, ref_r0, C_B_N)
        tp, tv = _to_frame(truth_pos_ned, truth_vel_ned, frame, C_ECEF_to_NED, ref_r0, C_B_N)
        dp, dv = fp - tp, fv - tv
        labels = FRAME_LABELS[frame]

        fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
        for j in range(3):
            axes[0, j].plot(t, dp[:, j], "r-", linewidth=0.8)
            axes[0, j].axhline(0, color="k", linewidth=0.6)
            axes[0, j].set_title(labels[j])
            axes[0, j].grid(True)
            axes[1, j].plot(t, dv[:, j], "r-", linewidth=0.8)
            axes[1, j].axhline(0, color="k", linewidth=0.6)
            axes[1, j].set_xlabel("Time [s]")
            axes[1, j].grid(True)
        axes[0, 0].set_ylabel("Position: FUSED − TRUTH [m]")
        axes[1, 0].set_ylabel("Velocity: FUSED − TRUTH [m/s]")
        fig.suptitle(
            f"Task 7.5 — {frame.upper()} frame — difference TRUTH vs FUSED over time"
        )
        fig.tight_layout(rect=[0, 0, 1, 0.95])
        stem = f"{tag}_task7_5_diff_truth_fused_over_time_{frame}"
        _save(fig, out_dir, stem, arrays=dict(
            t=t, diff_pos=dp, diff_vel=dv, frame=frame, labels=list(labels),
            plot_title=f"Task 7.5 - {frame.upper()} frame - FUSED minus TRUTH",
            x_label="Time [s]", col_names=list(labels),
            y_labels=["Position error [m]", "Velocity error [m/s]"]))
        written.append(stem)
    return written


# ---------------------------------------------------------------------------
# Task 7.6 — the four attitude figures
# ---------------------------------------------------------------------------
def task7_6_attitude(tag, t, quat_fused_b2n, quat_truth_b2n, out_dir):
    from scipy.spatial.transform import Rotation as R

    qt = np.asarray(quat_truth_b2n, dtype=float)
    qf = _align_sign(qt, np.asarray(quat_fused_b2n, dtype=float))
    written = []

    # 1) components, truth vs estimate
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for i, ax in enumerate(axes.flat):
        ax.plot(t, qt[:, i], "m-", label="TRUTH")
        ax.plot(t, qf[:, i], "b--", label="ESTIMATE")
        ax.set_ylabel(QUAT_LABELS[i]); ax.grid(True)
        ax.ticklabel_format(axis="y", style="plain", useOffset=False)
    axes[0, 0].legend(loc="best")
    axes[1, 0].set_xlabel("Time [s]"); axes[1, 1].set_xlabel("Time [s]")
    fig.suptitle("Task 7.6 — Body→NED attitude quaternion: TRUTH vs ESTIMATE")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    stem = f"{tag}_task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion"
    _save(fig, out_dir, stem, arrays=dict(
        t=t, quat_truth_wxyz=qt, quat_est_wxyz=qf,
        plot_title="Task 7.6 - Body->NED quaternion: TRUTH vs ESTIMATE",
        x_label="Time [s]", col_names=list(QUAT_LABELS),
        y_labels=["Truth quaternion", "Estimate quaternion"]))
    written.append(stem)

    # 2) component-wise error
    err = qf - qt
    fig, axes = plt.subplots(2, 2, figsize=(12, 7), sharex=True)
    for i, ax in enumerate(axes.flat):
        ax.plot(t, err[:, i], "r-", linewidth=0.8)
        ax.axhline(0, color="k", linewidth=0.6)
        ax.set_ylabel(f"Δ{QUAT_LABELS[i]}"); ax.grid(True)
    axes[1, 0].set_xlabel("Time [s]"); axes[1, 1].set_xlabel("Time [s]")
    fig.suptitle("Task 7.6 — Body→NED quaternion error components (ESTIMATE − TRUTH)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    stem = f"{tag}_task7_6_BodyToNED_attitude_quaternion_error_components"
    _save(fig, out_dir, stem, arrays=dict(
        t=t, quat_error_wxyz=err,
        plot_title="Task 7.6 - Body->NED quaternion error (ESTIMATE - TRUTH)",
        x_label="Time [s]", col_names=list(QUAT_LABELS),
        y_labels=["Quaternion error"]))
    written.append(stem)

    # 3) Euler error over time
    eul_t = R.from_quat(_wxyz_to_xyzw(qt)).as_euler("xyz", degrees=True)
    eul_f = R.from_quat(_wxyz_to_xyzw(qf)).as_euler("xyz", degrees=True)
    deul = _wrap180(eul_f - eul_t)
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    for i, name in enumerate(("Roll", "Pitch", "Yaw")):
        axes[i].plot(t, deul[:, i], "r-", linewidth=0.8)
        axes[i].axhline(0, color="k", linewidth=0.6)
        axes[i].set_ylabel(f"Δ{name} [deg]"); axes[i].grid(True)
    axes[2].set_xlabel("Time [s]")
    fig.suptitle("Task 7.6 — Body→NED Euler angle error over time (ESTIMATE − TRUTH)")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    stem = f"{tag}_task7_6_BodyToNED_attitude_euler_error_over_time"
    _save(fig, out_dir, stem, arrays=dict(
        t=t, euler_error_deg=deul, euler_truth_deg=eul_t, euler_est_deg=eul_f,
        plot_title="Task 7.6 - Body->NED Euler error over time",
        x_label="Time [s]", col_names=["Roll", "Pitch", "Yaw"],
        y_labels=["Euler error [deg]", "Truth [deg]", "Estimate [deg]"]))
    written.append(stem)

    # 4) total attitude error angle (sign invariant)
    dots = np.clip(np.abs(np.sum(qf * qt, axis=1)), 0.0, 1.0)
    ang = 2.0 * np.degrees(np.arccos(dots))
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, ang, "b-", linewidth=0.8)
    ax.axhline(float(np.sqrt(np.mean(ang**2))), color="k", linestyle="--",
               label=f"RMSE = {np.sqrt(np.mean(ang**2)):.4f}°")
    ax.set_xlabel("Time [s]"); ax.set_ylabel("Attitude error [deg]")
    ax.grid(True); ax.legend(loc="best")
    fig.suptitle("Task 7.6 — total attitude error angle over time")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    stem = f"{tag}_task7_6_attitude_error_angle_over_time"
    _save(fig, out_dir, stem, arrays=dict(
        t=t, attitude_error_deg=ang,
        plot_title="Task 7.6 - total attitude error angle",
        x_label="Time [s]", y_labels=["Attitude error [deg]"]))
    written.append(stem)
    return written


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _wxyz_to_xyzw(q):
    q = np.asarray(q, dtype=float)
    return np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])


def _xyzw_to_wxyz(q):
    q = np.asarray(q, dtype=float)
    return np.column_stack([q[:, 3], q[:, 0], q[:, 1], q[:, 2]])


def _align_sign(reference, estimate):
    """A quaternion and its negation are the same rotation; match hemispheres."""
    reference = np.asarray(reference, dtype=float)
    estimate = np.asarray(estimate, dtype=float).copy()
    flip = np.sum(reference * estimate, axis=1) < 0
    estimate[flip] *= -1.0
    return estimate


def _wrap180(angles):
    return (np.asarray(angles, dtype=float) + 180.0) % 360.0 - 180.0
