"""Canonical, configurable implementation of Tasks 1 through 7."""

from __future__ import annotations

import json
import os
import platform
import sys
from dataclasses import asdict, dataclass, fields
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .attitude import METHODS, solve_attitude
from .contracts import (
    GnssData,
    ImuData,
    TruthData,
    load_gnss,
    load_imu,
    load_truth,
    validation_summary,
)
from .math3d import (
    align_quaternion_sign,
    ecef_to_geodetic,
    ecef_to_ned_matrix,
    normalize,
    quaternion_from_rotvec,
    quaternion_multiply,
    quaternion_to_matrix,
)


@dataclass
class PipelineConfig:
    """All user-adjustable numeric settings, with conservative defaults."""

    imu_measurement_type: str = "delta"
    static_samples: int = 400
    gravity_override_mps2: float | None = None
    earth_rate_rps: float = 7.292115e-5
    gravity_weight: float = 0.9999
    earth_rate_weight: float = 0.0001
    process_accel_std_mps2: float = 20.0
    gnss_position_std_m: float = 2.0
    gnss_velocity_std_mps: float = 0.5
    max_specific_force_mps2: float = 100.0
    max_angular_rate_rps: float = 20.0
    truth_quaternion_order: str = "xyzw"
    truth_quaternion_frame: str = "body_to_ecef"
    truth_attitude_time_offset_s: float = -0.05
    max_plot_points: int = 50000
    plots: bool = True

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None) -> "PipelineConfig":
        values = values or {}
        allowed = {field.name for field in fields(cls)}
        unknown = sorted(set(values) - allowed)
        if unknown:
            raise ValueError(f"Unknown pipeline configuration keys: {', '.join(unknown)}")
        cfg = cls(**values)
        cfg.validate()
        return cfg

    def validate(self) -> None:
        if self.imu_measurement_type not in {"delta", "rate"}:
            raise ValueError("imu_measurement_type must be 'delta' or 'rate'")
        if self.truth_quaternion_order not in {"wxyz", "xyzw"}:
            raise ValueError("truth_quaternion_order must be 'wxyz' or 'xyzw'")
        if self.truth_quaternion_frame not in {"body_to_ned", "body_to_ecef"}:
            raise ValueError("truth_quaternion_frame must be 'body_to_ned' or 'body_to_ecef'")
        if not np.isfinite(self.truth_attitude_time_offset_s) or abs(self.truth_attitude_time_offset_s) > 10:
            raise ValueError("truth_attitude_time_offset_s must be finite and within +/-10 s")
        if self.static_samples < 20:
            raise ValueError("static_samples must be at least 20")
        positive = (
            "earth_rate_rps",
            "gravity_weight",
            "earth_rate_weight",
            "process_accel_std_mps2",
            "gnss_position_std_m",
            "gnss_velocity_std_mps",
            "max_specific_force_mps2",
            "max_angular_rate_rps",
            "max_plot_points",
        )
        for key in positive:
            if float(getattr(self, key)) <= 0:
                raise ValueError(f"{key} must be positive")
        if self.gravity_override_mps2 is not None and self.gravity_override_mps2 <= 0:
            raise ValueError("gravity_override_mps2 must be positive when supplied")


def _json_value(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def _write_json(path: Path, payload: dict[str, Any]) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(_json_value(payload), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)
    return path


def _save_npz(path: Path, **arrays: Any) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **arrays)
    return path


def _canonical_method(method: str) -> str:
    match = next((name for name in METHODS if name.lower() == str(method).lower()), None)
    if match is None:
        raise ValueError(f"Unknown method {method!r}; choose {', '.join(METHODS)} or ALL")
    return match


def _safe_name(value: str) -> str:
    return "".join(char if char.isalnum() or char in "-_." else "_" for char in value)


def parse_tasks(specification: str | Iterable[int]) -> tuple[list[int], list[int]]:
    """Return requested tasks and dependency-expanded tasks.

    Task execution is prefix-based because every downstream contract consumes
    upstream artifacts. For example, requesting Task 5 executes Tasks 1-5.
    """
    if isinstance(specification, str):
        requested: set[int] = set()
        for item in specification.replace(" ", "").split(","):
            if not item:
                continue
            if "-" in item:
                start, end = (int(part) for part in item.split("-", 1))
                requested.update(range(min(start, end), max(start, end) + 1))
            else:
                requested.add(int(item))
    else:
        requested = {int(value) for value in specification}
    if not requested or min(requested) < 1 or max(requested) > 7:
        raise ValueError("tasks must select one or more task numbers from 1 through 7")
    expanded = list(range(1, max(requested) + 1))
    return sorted(requested), expanded


def _task_dir(run_dir: Path, number: int, slug: str) -> Path:
    path = run_dir / f"task_{number:02d}_{slug}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def _plot_import():
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt


def _stride(length: int, max_points: int) -> int:
    return max(1, int(np.ceil(length / max_points)))


def _plot_task1(path: Path, lat_deg: float, lon_deg: float) -> None:
    plt = _plot_import()
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.scatter([lon_deg], [lat_deg], color="crimson", s=45, label="Initial GNSS fix")
    ax.set(xlim=(-180, 180), ylim=(-90, 90), xlabel="Longitude [deg]", ylabel="Latitude [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title("Task 1.2 — Reference location")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _plot_task2(path: Path, imu: ImuData, start: int, end: int, max_points: int) -> None:
    plt = _plot_import()
    step = _stride(len(imu.time_s), max_points)
    t = imu.time_s[::step]
    fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    labels = ("x", "y", "z")
    for axis, values, unit in zip(axes, (imu.accel_mps2, imu.gyro_rps), ("m/s²", "rad/s")):
        for index, label in enumerate(labels):
            axis.plot(t, values[::step, index], label=label, linewidth=0.9)
        axis.axvspan(imu.time_s[start], imu.time_s[end - 1], color="gold", alpha=0.2, label="static window")
        axis.set_ylabel(unit)
        axis.grid(True, alpha=0.3)
    axes[0].legend(ncol=4)
    axes[1].set_xlabel("Time [s]")
    fig.suptitle("Task 2 — IMU body vectors and selected static interval")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _plot_task3(path: Path, quaternion: np.ndarray, errors: dict[str, float], method: str) -> None:
    plt = _plot_import()
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    axes[0].bar(("w", "x", "y", "z"), quaternion, color="tab:blue")
    axes[0].set_ylim(-1.05, 1.05)
    axes[0].set_title("Normalized Body→NED quaternion")
    axes[0].grid(True, axis="y", alpha=0.3)
    axes[1].bar(("Gravity", "Earth rate"), (errors["gravity_error_deg"], errors["earth_rate_error_deg"]), color=("tab:green", "tab:orange"))
    axes[1].set_ylabel("Vector alignment error [deg]")
    axes[1].grid(True, axis="y", alpha=0.3)
    fig.suptitle(f"Task 3 — {method} initial attitude")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _plot_state_grid(path: Path, title: str, time_s: np.ndarray, position: np.ndarray, velocity: np.ndarray, max_points: int) -> None:
    plt = _plot_import()
    step = _stride(len(time_s), max_points)
    t, p, v = time_s[::step], position[::step], velocity[::step]
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex=True)
    labels = ("North", "East", "Down")
    for index, label in enumerate(labels):
        axes[0, index].plot(t, p[:, index], color="tab:blue")
        axes[0, index].set_title(label)
        axes[0, index].set_ylabel("Position [m]")
        axes[1, index].plot(t, v[:, index], color="tab:orange")
        axes[1, index].set_ylabel("Velocity [m/s]")
        axes[1, index].set_xlabel("Time [s]")
        axes[0, index].grid(True, alpha=0.3)
        axes[1, index].grid(True, alpha=0.3)
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _normal_gravity(lat_rad: float, altitude_m: float) -> float:
    sin2 = np.sin(lat_rad) ** 2
    surface = 9.7803253359 * (1 + 0.00193185265241 * sin2) / np.sqrt(1 - 0.00669437999013 * sin2)
    return float(surface - 3.086e-6 * altitude_m)


def _task1(gnss: GnssData, imu: ImuData, truth: TruthData | None, cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    origin = gnss.position_ecef_m[0]
    lat, lon, altitude = ecef_to_geodetic(origin)
    c_ecef_to_ned = ecef_to_ned_matrix(lat, lon)
    normal_gravity = _normal_gravity(lat, altitude)
    gravity = (
        normal_gravity
        if cfg.gravity_override_mps2 is None
        else float(cfg.gravity_override_mps2)
    )
    omega_ned = cfg.earth_rate_rps * np.array([np.cos(lat), 0.0, -np.sin(lat)])
    summary = {
        "task": 1,
        "name": "Input validation and reference navigation vectors",
        "subtasks": {
            "1.1": "Validate and normalize IMU/GNSS/truth inputs",
            "1.2": "Derive WGS-84 origin from first GNSS ECEF fix",
            "1.3": "Compute local gravity and Earth-rate vectors in NED",
        },
        "validation": validation_summary(imu, gnss, truth),
        "origin_ecef_m": origin,
        "latitude_deg": np.degrees(lat),
        "longitude_deg": np.degrees(lon),
        "altitude_m": altitude,
        "gravity_mps2": gravity,
        "normal_gravity_mps2": normal_gravity,
        "gravity_was_overridden": cfg.gravity_override_mps2 is not None,
        "specific_force_reference_ned_mps2": [0.0, 0.0, -gravity],
        "earth_rate_reference_ned_rps": omega_ned,
        "c_ecef_to_ned": c_ecef_to_ned,
    }
    _write_json(directory / "reference.json", summary)
    if cfg.plots:
        _plot_task1(directory / "reference_location.png", np.degrees(lat), np.degrees(lon))
    return {**summary, "lat_rad": lat, "lon_rad": lon}


def _rolling_static_window(imu: ImuData, requested: int) -> tuple[int, int, np.ndarray]:
    window = min(int(requested), len(imu.time_s))
    if window < 20:
        raise ValueError("IMU file is too short for the minimum 20-sample static interval")
    features = np.linalg.norm(imu.accel_mps2, axis=1) + 20.0 * np.linalg.norm(imu.gyro_rps, axis=1)
    kernel = np.ones(window)
    sums = np.convolve(features, kernel, mode="valid")
    sums2 = np.convolve(features * features, kernel, mode="valid")
    variance = np.maximum(0.0, sums2 / window - (sums / window) ** 2)
    start = int(np.argmin(variance))
    return start, start + window, variance


def _task2(imu: ImuData, task1: dict[str, Any], cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    start, end, variances = _rolling_static_window(imu, cfg.static_samples)
    mean_accel = np.mean(imu.accel_mps2[start:end], axis=0)
    mean_gyro = np.mean(imu.gyro_rps[start:end], axis=0)
    body_vectors = np.vstack([normalize(mean_accel, "static specific force"), normalize(mean_gyro, "static angular rate")])
    reference_vectors = np.vstack(
        [normalize(np.asarray(task1["specific_force_reference_ned_mps2"])), normalize(np.asarray(task1["earth_rate_reference_ned_rps"]))]
    )
    summary = {
        "task": 2,
        "name": "Static interval and measured body vectors",
        "subtasks": {
            "2.1": "Convert delta IMU measurements to SI rates when configured",
            "2.2": "Select the minimum-variance static window",
            "2.3": "Average and normalize specific-force/Earth-rate body vectors",
        },
        "static_start_index": start,
        "static_end_index_exclusive": end,
        "static_duration_s": float(imu.time_s[end - 1] - imu.time_s[start] + imu.dt_s),
        "static_feature_variance": float(variances[start]),
        "mean_accel_body_mps2": mean_accel,
        "mean_gyro_body_rps": mean_gyro,
        "body_vectors_unit": body_vectors,
        "reference_vectors_unit": reference_vectors,
    }
    _write_json(directory / "body_vectors.json", summary)
    if cfg.plots:
        _plot_task2(directory / "static_interval.png", imu, start, end, cfg.max_plot_points)
    return summary


def _task3(method: str, task1: dict[str, Any], task2: dict[str, Any], cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    c_b2n, q_wxyz, errors = solve_attitude(
        method,
        np.asarray(task2["body_vectors_unit"]),
        np.asarray(task2["reference_vectors_unit"]),
        np.array([cfg.gravity_weight, cfg.earth_rate_weight]),
    )
    expected_accel_body = c_b2n.T @ np.asarray(task1["specific_force_reference_ned_mps2"])
    expected_gyro_body = c_b2n.T @ np.asarray(task1["earth_rate_reference_ned_rps"])
    accel_bias = np.asarray(task2["mean_accel_body_mps2"]) - expected_accel_body
    gyro_bias = np.asarray(task2["mean_gyro_body_rps"]) - expected_gyro_body
    summary = {
        "task": 3,
        "name": "Initial attitude and IMU biases",
        "method": method,
        "subtasks": {
            "3.1": "Solve Body-to-NED Wahba alignment",
            "3.2": "Normalize scalar-first quaternion [w,x,y,z]",
            "3.3": "Estimate accelerometer and gyro bias in the chosen attitude",
        },
        "c_body_to_ned": c_b2n,
        "quaternion_wxyz_body_to_ned": q_wxyz,
        "quaternion_norm": float(np.linalg.norm(q_wxyz)),
        "accel_bias_body_mps2": accel_bias,
        "gyro_bias_body_rps": gyro_bias,
        **errors,
    }
    _write_json(directory / "initial_attitude.json", summary)
    if cfg.plots:
        _plot_task3(directory / "initial_attitude.png", q_wxyz, errors, method)
    return summary


def _interpolate_range_outliers(time_s: np.ndarray, values: np.ndarray, limit: float) -> tuple[np.ndarray, int]:
    valid = np.all(np.isfinite(values), axis=1) & (np.linalg.norm(values, axis=1) <= limit)
    rejected = int(np.count_nonzero(~valid))
    if rejected == 0:
        return values, 0
    if np.count_nonzero(valid) < 2:
        raise ValueError(f"Fewer than two IMU samples remain below physical limit {limit:g}")
    repaired = np.asarray(values, dtype=float).copy()
    for column in range(repaired.shape[1]):
        repaired[:, column] = np.interp(time_s, time_s[valid], repaired[valid, column])
    return repaired, rejected


def _propagate_attitude_and_acceleration(
    imu: ImuData,
    gnss: GnssData,
    task1: dict[str, Any],
    task3: dict[str, Any],
    cfg: PipelineConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, dict[str, int]]:
    q = np.asarray(task3["quaternion_wxyz_body_to_ned"], dtype=float)
    accel_bias = np.asarray(task3["accel_bias_body_mps2"], dtype=float)
    gyro_bias = np.asarray(task3["gyro_bias_body_rps"], dtype=float)
    gravity_n = np.array([0.0, 0.0, float(task1["gravity_mps2"])])
    quaternions = np.empty((len(imu.time_s), 4))
    acceleration_ned = np.empty((len(imu.time_s), 3))
    position = np.zeros((len(imu.time_s), 3))
    velocity = np.zeros_like(position)
    velocity[0] = np.asarray(task1["c_ecef_to_ned"]) @ gnss.velocity_ecef_mps[0]
    accel, accel_rejected = _interpolate_range_outliers(
        imu.time_s, imu.accel_mps2, cfg.max_specific_force_mps2
    )
    gyro, gyro_rejected = _interpolate_range_outliers(
        imu.time_s, imu.gyro_rps, cfg.max_angular_rate_rps
    )
    for index in range(len(imu.time_s)):
        dt = imu.dt_s if index == 0 else imu.time_s[index] - imu.time_s[index - 1]
        state_index = max(0, index - 1)
        latitude = float(task1["lat_rad"]) + position[state_index, 0] / 6.35e6
        altitude = float(task1["altitude_m"]) - position[state_index, 2]
        sin_lat = np.sin(latitude)
        denom = np.sqrt(1.0 - 6.69437999014e-3 * sin_lat * sin_lat)
        radius_east = 6378137.0 / denom
        radius_north = 6378137.0 * (1.0 - 6.69437999014e-3) / denom**3
        velocity_previous = velocity[state_index]
        omega_ie_n = cfg.earth_rate_rps * np.array(
            [np.cos(latitude), 0.0, -np.sin(latitude)]
        )
        omega_en_n = np.array(
            [
                velocity_previous[1] / (radius_east + altitude),
                -velocity_previous[0] / (radius_north + altitude),
                -velocity_previous[1] * np.tan(latitude) / (radius_east + altitude),
            ]
        )
        c_b2n = quaternion_to_matrix(q)
        omega_nb_b = gyro[index] - gyro_bias - c_b2n.T @ (omega_ie_n + omega_en_n)
        q = quaternion_multiply(q, quaternion_from_rotvec(omega_nb_b * dt))
        q /= np.linalg.norm(q)
        if index and np.dot(q, quaternions[index - 1]) < 0:
            q = -q
        c_b2n = quaternion_to_matrix(q)
        coriolis = -np.cross(2.0 * omega_ie_n + omega_en_n, velocity_previous)
        acceleration_ned[index] = c_b2n @ (accel[index] - accel_bias) + gravity_n + coriolis
        quaternions[index] = q
        if index > 0:
            velocity[index] = velocity[index - 1] + 0.5 * (
                acceleration_ned[index - 1] + acceleration_ned[index]
            ) * dt
            position[index] = position[index - 1] + 0.5 * (
                velocity[index - 1] + velocity[index]
            ) * dt
    return quaternions, acceleration_ned, position, velocity, {
        "accelerometer_samples_interpolated": accel_rejected,
        "gyroscope_samples_interpolated": gyro_rejected,
    }


def _task4(imu: ImuData, gnss: GnssData, task1: dict[str, Any], task3: dict[str, Any], cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    quaternions, acceleration, position, velocity, range_screening = (
        _propagate_attitude_and_acceleration(imu, gnss, task1, task3, cfg)
    )
    artifact = _save_npz(
        directory / "inertial_solution.npz",
        time_s=imu.time_s,
        position_ned_m=position,
        velocity_ned_mps=velocity,
        acceleration_ned_mps2=acceleration,
        quaternion_wxyz=quaternions,
    )
    summary = {
        "task": 4,
        "name": "IMU-only strapdown propagation",
        "subtasks": {
            "4.1": "Screen physical-range outliers and correct IMU using Task 3 biases",
            "4.2": "Propagate Body-to-NED quaternion with Earth and transport rates",
            "4.3": "Apply Coriolis compensation and integrate NED state",
        },
        "samples": len(imu.time_s),
        "duration_s": float(imu.time_s[-1]),
        "range_screening": range_screening,
        "artifact": str(artifact),
        "final_position_ned_m": position[-1],
        "final_velocity_ned_mps": velocity[-1],
    }
    _write_json(directory / "summary.json", summary)
    if cfg.plots:
        _plot_state_grid(directory / "inertial_solution.png", "Task 4 — IMU-only solution", imu.time_s, position, velocity, cfg.max_plot_points)
    return {**summary, "time_s": imu.time_s, "position": position, "velocity": velocity, "acceleration": acceleration, "quaternion": quaternions}


def _gnss_ned(gnss: GnssData, task1: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    c = np.asarray(task1["c_ecef_to_ned"])
    origin = np.asarray(task1["origin_ecef_m"])
    return (c @ (gnss.position_ecef_m - origin).T).T, (c @ gnss.velocity_ecef_mps.T).T


def _task5(imu: ImuData, gnss: GnssData, task1: dict[str, Any], task4: dict[str, Any], cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    gnss_position, gnss_velocity = _gnss_ned(gnss, task1)
    state = np.r_[gnss_position[0], gnss_velocity[0]]
    covariance = np.diag([cfg.gnss_position_std_m**2] * 3 + [cfg.gnss_velocity_std_mps**2] * 3)
    measurement_covariance = covariance.copy()
    identity = np.eye(6)
    position = np.empty((len(imu.time_s), 3))
    velocity = np.empty_like(position)
    innovation_times: list[float] = []
    innovations: list[np.ndarray] = []
    gnss_index = 0
    for index, time_s in enumerate(imu.time_s):
        dt = imu.dt_s if index == 0 else time_s - imu.time_s[index - 1]
        transition = np.block([[np.eye(3), dt * np.eye(3)], [np.zeros((3, 3)), np.eye(3)]])
        control = np.vstack([0.5 * dt * dt * np.eye(3), dt * np.eye(3)])
        state = transition @ state + control @ task4["acceleration"][index]
        covariance = transition @ covariance @ transition.T + (cfg.process_accel_std_mps2**2) * (control @ control.T)
        while gnss_index < len(gnss.time_s) and gnss.time_s[gnss_index] <= time_s + dt / 2.0:
            measurement = np.r_[gnss_position[gnss_index], gnss_velocity[gnss_index]]
            innovation = measurement - state
            innovation_covariance = covariance + measurement_covariance
            gain = np.linalg.solve(innovation_covariance.T, covariance.T).T
            state = state + gain @ innovation
            covariance = (identity - gain) @ covariance @ (identity - gain).T + gain @ measurement_covariance @ gain.T
            innovation_times.append(float(time_s))
            innovations.append(innovation)
            gnss_index += 1
        position[index], velocity[index] = state[:3], state[3:]
    innovations_array = np.asarray(innovations, dtype=float).reshape(-1, 6)
    artifact = _save_npz(
        directory / "fused_solution.npz",
        time_s=imu.time_s,
        position_ned_m=position,
        velocity_ned_mps=velocity,
        acceleration_ned_mps2=task4["acceleration"],
        quaternion_wxyz=task4["quaternion"],
        innovation_time_s=np.asarray(innovation_times),
        innovations=innovations_array,
    )
    summary = {
        "task": 5,
        "name": "GNSS/IMU Kalman fusion",
        "subtasks": {
            "5.1": "Predict six-state NED position/velocity from IMU acceleration",
            "5.2": "Update with asynchronous GNSS ECEF position/velocity converted to NED",
            "5.3": "Save fused state and innovation history",
        },
        "samples": len(imu.time_s),
        "gnss_updates": len(innovations_array),
        "artifact": str(artifact),
        "final_position_ned_m": position[-1],
        "final_velocity_ned_mps": velocity[-1],
    }
    _write_json(directory / "summary.json", summary)
    if cfg.plots:
        _plot_state_grid(directory / "fused_solution.png", "Task 5 — GNSS/IMU fused solution", imu.time_s, position, velocity, cfg.max_plot_points)
    return {**summary, "time_s": imu.time_s, "position": position, "velocity": velocity, "acceleration": task4["acceleration"], "quaternion": task4["quaternion"], "innovation_times": np.asarray(innovation_times), "innovations": innovations_array}


def _interp_columns(source_time: np.ndarray, values: np.ndarray, target_time: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(target_time, source_time, values[:, index]) for index in range(values.shape[1])])


def _truth_on_fused_grid(
    truth: TruthData,
    task1: dict[str, Any],
    task5: dict[str, Any],
    cfg: PipelineConfig,
) -> dict[str, np.ndarray]:
    end = min(float(truth.time_s[-1]), float(task5["time_s"][-1]))
    mask = task5["time_s"] <= end
    time_s = task5["time_s"][mask]
    if time_s.size < 2:
        raise ValueError("Truth and fused solution do not have at least two overlapping samples")
    c = np.asarray(task1["c_ecef_to_ned"])
    origin = np.asarray(task1["origin_ecef_m"])
    truth_position_ned = (c @ (truth.position_ecef_m - origin).T).T
    truth_velocity_ned = (c @ truth.velocity_ecef_mps.T).T
    output = {
        "time_s": time_s,
        "estimated_position": task5["position"][mask],
        "estimated_velocity": task5["velocity"][mask],
        "estimated_quaternion": task5["quaternion"][mask],
        "truth_position": _interp_columns(truth.time_s, truth_position_ned, time_s),
        "truth_velocity": _interp_columns(truth.time_s, truth_velocity_ned, time_s),
    }
    if truth.quaternion_wxyz is not None:
        truth_source = truth.quaternion_wxyz.copy()
        if cfg.truth_quaternion_frame == "body_to_ecef":
            from .math3d import matrix_to_quaternion_wxyz

            truth_source = np.vstack(
                [
                    matrix_to_quaternion_wxyz(
                        ecef_to_ned_matrix(
                            *ecef_to_geodetic(position)[:2]
                        )
                        @ quaternion_to_matrix(quaternion)
                    )
                    for quaternion, position in zip(
                        truth_source, truth.position_ecef_m
                    )
                ]
            )
        for index in range(1, len(truth_source)):
            if np.dot(truth_source[index - 1], truth_source[index]) < 0:
                truth_source[index] *= -1.0
        truth_q = _interp_columns(
            truth.time_s,
            truth_source,
            time_s + cfg.truth_attitude_time_offset_s,
        )
        truth_q /= np.linalg.norm(truth_q, axis=1, keepdims=True)
        estimate_q = output["estimated_quaternion"]
        estimate_q /= np.linalg.norm(estimate_q, axis=1, keepdims=True)
        output["truth_quaternion"] = truth_q
        output["estimated_quaternion"] = align_quaternion_sign(truth_q, estimate_q)
    return output


def _plot_task6(directory: Path, overlay: dict[str, np.ndarray], max_points: int) -> None:
    plt = _plot_import()
    step = _stride(len(overlay["time_s"]), max_points)
    t = overlay["time_s"][::step]
    ep, tp = overlay["estimated_position"][::step], overlay["truth_position"][::step]
    ev, tv = overlay["estimated_velocity"][::step], overlay["truth_velocity"][::step]
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex=True)
    for index, label in enumerate(("North", "East", "Down")):
        axes[0, index].plot(t, ep[:, index], label="Fused")
        axes[0, index].plot(t, tp[:, index], "--", label="Truth")
        axes[0, index].set_title(label)
        axes[0, index].set_ylabel("Position [m]")
        axes[1, index].plot(t, ev[:, index], label="Fused")
        axes[1, index].plot(t, tv[:, index], "--", label="Truth")
        axes[1, index].set_ylabel("Velocity [m/s]")
        axes[1, index].set_xlabel("Time [s]")
        for axis in axes[:, index]:
            axis.grid(True, alpha=0.3)
    axes[0, 0].legend()
    fig.suptitle("Task 6 — Fused state against truth in common NED frame")
    fig.tight_layout()
    fig.savefig(directory / "truth_overlay_ned.png", dpi=160)
    plt.close(fig)

    # Height is explicitly -Down in NED. Plotting ECEF Z or quaternion
    # components as height was the source of the previous mismatch.
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(t, -ep[:, 2], label="Fused height")
    ax.plot(t, -tp[:, 2], "--", label="Truth height")
    ax.set(xlabel="Time [s]", ylabel="Relative height [m]", title="Task 6.3 — Height comparison (height = −NED Down)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(directory / "height_comparison.png", dpi=160)
    plt.close(fig)

    if "truth_quaternion" in overlay:
        eq, tq = overlay["estimated_quaternion"][::step], overlay["truth_quaternion"][::step]
        fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
        for index, (axis, name) in enumerate(zip(axes.flat, ("w", "x", "y", "z"))):
            axis.plot(t, eq[:, index], label="Fused")
            axis.plot(t, tq[:, index], "--", label="Truth")
            axis.set_title(f"q{name}")
            axis.grid(True, alpha=0.3)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
        axes[0, 0].legend()
        fig.suptitle("Task 6.4 — Normalized, sign-aligned Body→NED quaternions")
        fig.tight_layout()
        fig.savefig(directory / "quaternion_comparison.png", dpi=160)
        plt.close(fig)


def _task6(truth: TruthData | None, task1: dict[str, Any], task5: dict[str, Any], cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    if truth is None:
        summary = {
            "task": 6,
            "name": "Truth overlay",
            "status": "skipped",
            "reason": "No truth file was supplied; Tasks 1-5 remain valid",
            "subtasks": {"6.1": "Align truth time", "6.2": "Convert truth ECEF to common NED", "6.3": "Compare height as -NED Down", "6.4": "Normalize/sign-align quaternions"},
        }
        _write_json(directory / "summary.json", summary)
        return summary
    overlay = _truth_on_fused_grid(truth, task1, task5, cfg)
    artifact = _save_npz(directory / "truth_overlay.npz", **overlay)
    summary = {
        "task": 6,
        "name": "Truth overlay",
        "status": "complete",
        "subtasks": {"6.1": "Align truth time", "6.2": "Convert truth ECEF to common NED", "6.3": "Compare height as -NED Down", "6.4": "Normalize/sign-align quaternions"},
        "samples": len(overlay["time_s"]),
        "artifact": str(artifact),
        "height_definition": "height_m = -position_ned_down_m",
        "quaternion_convention": "scalar-first [w,x,y,z], Body-to-NED, unit norm, truth-hemisphere aligned",
        "truth_source_quaternion_order": cfg.truth_quaternion_order,
        "truth_source_quaternion_frame": cfg.truth_quaternion_frame,
        "attitude_reference_frame": "time-varying local NED at each truth ECEF position",
        "truth_attitude_time_offset_s": cfg.truth_attitude_time_offset_s,
    }
    _write_json(directory / "summary.json", summary)
    if cfg.plots:
        _plot_task6(directory, overlay, cfg.max_plot_points)
    return {**summary, "overlay": overlay}


def _rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def _plot_task7(path: Path, time_s: np.ndarray, position_error: np.ndarray, velocity_error: np.ndarray, max_points: int) -> None:
    plt = _plot_import()
    step = _stride(len(time_s), max_points)
    t, pe, ve = time_s[::step], position_error[::step], velocity_error[::step]
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex=True)
    for index, label in enumerate(("North", "East", "Down")):
        axes[0, index].plot(t, pe[:, index])
        axes[0, index].set_title(label)
        axes[0, index].set_ylabel("Position error [m]")
        axes[1, index].plot(t, ve[:, index])
        axes[1, index].set_ylabel("Velocity error [m/s]")
        axes[1, index].set_xlabel("Time [s]")
        for axis in axes[:, index]:
            axis.axhline(0, color="black", linewidth=0.5)
            axis.grid(True, alpha=0.3)
    fig.suptitle("Task 7 — Fused minus truth residuals")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _task7(task5: dict[str, Any], task6: dict[str, Any], cfg: PipelineConfig, directory: Path) -> dict[str, Any]:
    if task6.get("status") == "complete":
        overlay = task6["overlay"]
        position_error = overlay["estimated_position"] - overlay["truth_position"]
        velocity_error = overlay["estimated_velocity"] - overlay["truth_velocity"]
        position_norm = np.linalg.norm(position_error, axis=1)
        velocity_norm = np.linalg.norm(velocity_error, axis=1)
        metrics: dict[str, Any] = {
            "position_rmse_m": _rmse(position_norm),
            "position_final_m": float(position_norm[-1]),
            "position_max_m": float(np.max(position_norm)),
            "velocity_rmse_mps": _rmse(velocity_norm),
            "velocity_final_mps": float(velocity_norm[-1]),
            "velocity_max_mps": float(np.max(velocity_norm)),
            "height_rmse_m": _rmse(-position_error[:, 2]),
            "height_final_error_m": float(-position_error[-1, 2]),
            "height_final_abs_m": float(abs(position_error[-1, 2])),
        }
        if "truth_quaternion" in overlay:
            dots = np.clip(np.abs(np.sum(overlay["estimated_quaternion"] * overlay["truth_quaternion"], axis=1)), 0.0, 1.0)
            attitude_error = 2.0 * np.degrees(np.arccos(dots))
            metrics.update(attitude_rmse_deg=_rmse(attitude_error), attitude_final_deg=float(attitude_error[-1]), attitude_max_deg=float(np.max(attitude_error)))
        artifact = _save_npz(directory / "residuals.npz", time_s=overlay["time_s"], position_error_ned_m=position_error, velocity_error_ned_mps=velocity_error)
        if cfg.plots:
            _plot_task7(directory / "residuals.png", overlay["time_s"], position_error, velocity_error, cfg.max_plot_points)
        status = "complete"
    else:
        innovations = task5["innovations"]
        metrics = {
            "gnss_updates": len(innovations),
            "innovation_position_rms_m": _rmse(np.linalg.norm(innovations[:, :3], axis=1)) if len(innovations) else None,
            "innovation_velocity_rms_mps": _rmse(np.linalg.norm(innovations[:, 3:], axis=1)) if len(innovations) else None,
        }
        artifact = None
        status = "complete_without_truth"
    summary = {
        "task": 7,
        "name": "Residual evaluation and quality metrics",
        "status": status,
        "subtasks": {"7.1": "Compute NED position residuals", "7.2": "Compute NED velocity residuals", "7.3": "Compute quaternion geodesic error", "7.4": "Export scalar comparison metrics"},
        "metrics": metrics,
        "artifact": str(artifact) if artifact else None,
    }
    _write_json(directory / "metrics.json", summary)
    return summary


def _execute_tasks(
    run_dir: Path,
    expanded: list[int],
    method: str,
    imu: ImuData,
    gnss: GnssData,
    truth: TruthData | None,
    cfg: PipelineConfig,
) -> dict[int, dict[str, Any]]:
    outputs: dict[int, dict[str, Any]] = {}
    task1 = _task1(gnss, imu, truth, cfg, _task_dir(run_dir, 1, "inputs_reference"))
    outputs[1] = task1
    if max(expanded) >= 2:
        task2 = _task2(imu, task1, cfg, _task_dir(run_dir, 2, "static_imu"))
        outputs[2] = task2
    if max(expanded) >= 3:
        task3 = _task3(method, task1, task2, cfg, _task_dir(run_dir, 3, "attitude"))
        outputs[3] = task3
    if max(expanded) >= 4:
        task4 = _task4(imu, gnss, task1, task3, cfg, _task_dir(run_dir, 4, "inertial"))
        outputs[4] = task4
    if max(expanded) >= 5:
        task5 = _task5(imu, gnss, task1, task4, cfg, _task_dir(run_dir, 5, "fusion"))
        outputs[5] = task5
    if max(expanded) >= 6:
        task6 = _task6(truth, task1, task5, cfg, _task_dir(run_dir, 6, "truth"))
        outputs[6] = task6
    if max(expanded) >= 7:
        outputs[7] = _task7(task5, task6, cfg, _task_dir(run_dir, 7, "evaluation"))
    return outputs


def run_pipeline(
    imu_path: str | Path,
    gnss_path: str | Path,
    truth_path: str | Path | None = None,
    *,
    method: str = "TRIAD",
    tasks: str | Iterable[int] = "1-7",
    output_root: str | Path = "results",
    run_id: str | None = None,
    config: PipelineConfig | dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Process one dataset with one method and write one folder per task."""
    method = _canonical_method(method)
    cfg = config if isinstance(config, PipelineConfig) else PipelineConfig.from_mapping(config)
    cfg.validate()
    requested, expanded = parse_tasks(tasks)
    imu = load_imu(imu_path, cfg.imu_measurement_type)
    gnss = load_gnss(gnss_path)
    truth = load_truth(truth_path, cfg.truth_quaternion_order) if truth_path else None
    run_id = run_id or f"{imu.path.stem}__{gnss.path.stem}"
    safe_run_id = _safe_name(run_id)
    run_dir = Path(output_root).expanduser().resolve() / safe_run_id / method.lower()
    run_dir.mkdir(parents=True, exist_ok=True)
    started = datetime.now(timezone.utc)
    manifest: dict[str, Any] = {
        "schema_version": "2.0",
        "status": "running",
        "run_id": safe_run_id,
        "method": method,
        "requested_tasks": requested,
        "executed_tasks": expanded,
        "auto_dependencies": sorted(set(expanded) - set(requested)),
        "started_utc": started.isoformat(),
        "inputs": {"imu": str(imu.path), "gnss": str(gnss.path), "truth": str(truth.path) if truth else None},
        "config": asdict(cfg),
        "environment": {"python": sys.version.split()[0], "platform": platform.platform()},
    }
    _write_json(run_dir / "manifest.json", manifest)
    try:
        outputs = _execute_tasks(run_dir, expanded, method, imu, gnss, truth, cfg)
    except Exception as exc:
        failed = datetime.now(timezone.utc)
        manifest.update(
            status="failed",
            finished_utc=failed.isoformat(),
            elapsed_s=(failed - started).total_seconds(),
            error={"type": type(exc).__name__, "message": str(exc)},
        )
        _write_json(run_dir / "manifest.json", manifest)
        raise
    finished = datetime.now(timezone.utc)
    manifest.update(
        status="complete",
        finished_utc=finished.isoformat(),
        elapsed_s=(finished - started).total_seconds(),
        task_directories={str(number): str(next(run_dir.glob(f"task_{number:02d}_*"))) for number in outputs},
    )
    if 7 in outputs:
        manifest["metrics"] = outputs[7]["metrics"]
    _write_json(run_dir / "manifest.json", manifest)
    return {"run_dir": run_dir, "manifest": manifest, "tasks": outputs}


def _plot_method_comparison(path: Path, results: dict[str, dict[str, Any]], max_points: int) -> None:
    plt = _plot_import()
    fig, axes = plt.subplots(1, 3, figsize=(13, 4), sharex=True)
    for method, result in results.items():
        task5 = result["tasks"][5]
        step = _stride(len(task5["time_s"]), max_points)
        for index, label in enumerate(("North", "East", "Down")):
            axes[index].plot(task5["time_s"][::step], task5["position"][::step, index], label=method)
            axes[index].set_title(label)
            axes[index].set_xlabel("Time [s]")
            axes[index].set_ylabel("Position [m]")
            axes[index].grid(True, alpha=0.3)
    axes[0].legend()
    fig.suptitle("TRIAD vs Davenport vs SVD — fused position")
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def run_methods(
    imu_path: str | Path,
    gnss_path: str | Path,
    truth_path: str | Path | None = None,
    *,
    methods: Iterable[str] = METHODS,
    tasks: str | Iterable[int] = "1-7",
    output_root: str | Path = "results",
    run_id: str | None = None,
    config: PipelineConfig | dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Run any subset of methods and create comparison artifacts."""
    cfg = config if isinstance(config, PipelineConfig) else PipelineConfig.from_mapping(config)
    canonical_methods = [_canonical_method(method) for method in methods]
    if len(set(canonical_methods)) != len(canonical_methods):
        raise ValueError("methods must not contain duplicates")
    imu_stem, gnss_stem = Path(imu_path).stem, Path(gnss_path).stem
    run_id = _safe_name(run_id or f"{imu_stem}__{gnss_stem}")
    results = {
        method: run_pipeline(
            imu_path,
            gnss_path,
            truth_path,
            method=method,
            tasks=tasks,
            output_root=output_root,
            run_id=run_id,
            config=cfg,
        )
        for method in canonical_methods
    }
    comparison_dir = Path(output_root).expanduser().resolve() / run_id / "comparison"
    comparison_dir.mkdir(parents=True, exist_ok=True)
    metrics = {
        method: result["tasks"].get(7, {}).get("metrics", {})
        for method, result in results.items()
    }
    comparison = {
        "schema_version": "2.0",
        "methods": canonical_methods,
        "metrics": metrics,
        "note": "Task 1-2 inputs are common; Task 3 initialization differs by method; Tasks 4-7 use that method-specific attitude.",
    }
    _write_json(comparison_dir / "method_comparison.json", comparison)
    keys = sorted({key for method_metrics in metrics.values() for key in method_metrics})
    with (comparison_dir / "method_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        import csv

        writer = csv.writer(handle)
        writer.writerow(["method", *keys])
        for method in canonical_methods:
            writer.writerow([method, *[metrics[method].get(key, "") for key in keys]])
    if cfg.plots and all(5 in result["tasks"] for result in results.values()):
        _plot_method_comparison(comparison_dir / "method_comparison.png", results, cfg.max_plot_points)
    return {"comparison_dir": comparison_dir, "comparison": comparison, "results": results}
