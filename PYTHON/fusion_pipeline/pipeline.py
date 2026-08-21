"""Canonical, configurable implementation of Tasks 1 through 7."""

from __future__ import annotations

import json
import os
import platform
import sys
from dataclasses import asdict, dataclass, field, fields
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from . import figures as figs
from .attitude import METHODS, solve_attitude
from .catalog import COMPARISON_SLUG, TASK_BY_NUMBER, catalog_as_dict
from .contracts import (
    GnssData,
    GnssLayout,
    ImuData,
    ImuLayout,
    TruthData,
    TruthLayout,
    load_gnss,
    load_imu,
    load_truth,
    validation_summary,
)
from .figures import FigureWriter
from .math3d import (
    align_quaternion_sign,
    ecef_to_geodetic,
    ecef_to_ned_matrix,
    normalize,
    quaternion_from_rotvec,
    quaternion_multiply,
    quaternion_to_matrix,
)
from .report import SILENT, Progress

ALL_METHODS_TOKEN = "ALL"


@dataclass
class PipelineConfig:
    """All user-adjustable settings, with defaults matching the bundled data.

    The ``imu_*``/``gnss_*``/``truth_*`` layout fields exist so a differently
    formatted log can be read by changing configuration rather than rewriting
    the data file.
    """

    # -- IMU file layout --------------------------------------------------
    imu_measurement_type: str = "delta"
    imu_time_column: int = 1
    imu_gyro_columns: list[int] = field(default_factory=lambda: [2, 3, 4])
    imu_accel_columns: list[int] = field(default_factory=lambda: [5, 6, 7])
    imu_time_unit: str = "s"
    imu_gyro_unit: str = "rad"
    imu_accel_unit: str = "mps2"
    imu_delimiter: str | None = None

    # -- GNSS file layout -------------------------------------------------
    gnss_column_overrides: dict[str, str] = field(default_factory=dict)
    gnss_delimiter: str = ","
    gnss_time_unit: str = "s"
    gnss_position_unit: str = "m"
    gnss_velocity_unit: str = "mps"

    # -- Truth file layout ------------------------------------------------
    truth_time_column: int = 1
    truth_position_columns: list[int] = field(default_factory=lambda: [2, 3, 4])
    truth_velocity_columns: list[int] = field(default_factory=lambda: [5, 6, 7])
    truth_quaternion_columns: list[int] | None = field(default_factory=lambda: [8, 9, 10, 11])
    truth_quaternion_order: str = "xyzw"
    truth_quaternion_frame: str = "body_to_ecef"
    truth_time_unit: str = "s"
    truth_position_unit: str = "m"
    truth_velocity_unit: str = "mps"
    truth_delimiter: str | None = None
    truth_attitude_time_offset_s: float = -0.05

    # -- Algorithm settings -----------------------------------------------
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

    # -- Output settings --------------------------------------------------
    max_plot_points: int = 50000
    plot_dpi: int = 160
    plots: bool = True

    @classmethod
    def from_mapping(cls, values: dict[str, Any] | None) -> "PipelineConfig":
        values = values or {}
        allowed = {item.name for item in fields(cls)}
        unknown = sorted(set(values) - allowed)
        if unknown:
            raise ValueError(
                f"Unknown pipeline configuration keys: {', '.join(unknown)}. "
                f"Accepted keys: {', '.join(sorted(allowed))}"
            )
        cfg = cls(**values)
        cfg.validate()
        return cfg

    # -- layout construction ---------------------------------------------
    def imu_layout(self) -> ImuLayout:
        return ImuLayout(
            time_column=int(self.imu_time_column),
            gyro_columns=tuple(int(v) for v in self.imu_gyro_columns),  # type: ignore[arg-type]
            accel_columns=tuple(int(v) for v in self.imu_accel_columns),  # type: ignore[arg-type]
            measurement_type=self.imu_measurement_type,
            time_unit=self.imu_time_unit,
            gyro_unit=self.imu_gyro_unit,
            accel_unit=self.imu_accel_unit,
            delimiter=self.imu_delimiter,
        )

    def gnss_layout(self) -> GnssLayout:
        return GnssLayout(
            column_overrides=dict(self.gnss_column_overrides),
            delimiter=self.gnss_delimiter,
            time_unit=self.gnss_time_unit,
            position_unit=self.gnss_position_unit,
            velocity_unit=self.gnss_velocity_unit,
        )

    def truth_layout(self) -> TruthLayout:
        quaternion_columns = (
            None
            if self.truth_quaternion_columns is None
            else tuple(int(v) for v in self.truth_quaternion_columns)
        )
        return TruthLayout(
            time_column=int(self.truth_time_column),
            position_columns=tuple(int(v) for v in self.truth_position_columns),  # type: ignore[arg-type]
            velocity_columns=tuple(int(v) for v in self.truth_velocity_columns),  # type: ignore[arg-type]
            quaternion_columns=quaternion_columns,  # type: ignore[arg-type]
            quaternion_order=self.truth_quaternion_order,
            time_unit=self.truth_time_unit,
            position_unit=self.truth_position_unit,
            velocity_unit=self.truth_velocity_unit,
            delimiter=self.truth_delimiter,
        )

    def validate(self) -> None:
        # Layout validation lives with the readers so one rule has one home.
        self.imu_layout().validate()
        self.gnss_layout().validate()
        self.truth_layout().validate()
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
            "plot_dpi",
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
        raise ValueError(
            f"Unknown method {method!r}; choose {', '.join(METHODS)} or {ALL_METHODS_TOKEN}"
        )
    return match


def parse_methods(specification: str | Iterable[str]) -> list[str]:
    """Resolve a method selection into an ordered, de-duplicated method list.

    Accepts ``"TRIAD"``, ``"triad,svd"``, ``"ALL"``, or any iterable of names.
    Order always follows :data:`METHODS` so comparison artifacts are stable.
    """
    if isinstance(specification, str):
        tokens = [token.strip() for token in specification.split(",") if token.strip()]
    else:
        tokens = [str(token).strip() for token in specification if str(token).strip()]
    if not tokens:
        raise ValueError("No attitude method was selected")
    if any(token.upper() == ALL_METHODS_TOKEN for token in tokens):
        if len(tokens) > 1:
            raise ValueError(f"{ALL_METHODS_TOKEN} cannot be combined with individual method names")
        return list(METHODS)
    selected = [_canonical_method(token) for token in tokens]
    duplicates = sorted({name for name in selected if selected.count(name) > 1})
    if duplicates:
        raise ValueError(f"Method list contains duplicates: {', '.join(duplicates)}")
    return [name for name in METHODS if name in selected]


def _safe_name(value: str) -> str:
    return figs.safe_token(value)


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


def _task_dir(run_dir: Path, number: int) -> Path:
    path = run_dir / TASK_BY_NUMBER[number].directory_name
    path.mkdir(parents=True, exist_ok=True)
    return path


def _task_header(number: int) -> dict[str, Any]:
    task = TASK_BY_NUMBER[number]
    return {
        "task": task.number,
        "name": task.name,
        "purpose": task.purpose,
        "directory": task.directory_name,
        "subtasks": task.subtask_map,
    }


def _normal_gravity(lat_rad: float, altitude_m: float) -> float:
    sin2 = np.sin(lat_rad) ** 2
    surface = 9.7803253359 * (1 + 0.00193185265241 * sin2) / np.sqrt(1 - 0.00669437999013 * sin2)
    return float(surface - 3.086e-6 * altitude_m)


# ---------------------------------------------------------------------------
# Task 1
# ---------------------------------------------------------------------------
def _task1(
    gnss: GnssData,
    imu: ImuData,
    truth: TruthData | None,
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
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
        **_task_header(1),
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
    validation = summary["validation"]
    imu_info, gnss_info, truth_info = validation["imu"], validation["gnss"], validation["truth"]
    progress.subtask(
        "1.1",
        f"IMU {imu_info['rows']} rows @ {imu_info['sample_rate_hz']:.1f} Hz"
        f" · GNSS {gnss_info['rows']} epochs"
        + (f" · Truth {truth_info['rows']} states" if truth_info else " · no truth file")
        + (f" · {imu_info['clock_wraps_repaired']} clock resets repaired"
           if imu_info["clock_wraps_repaired"] else ""),
    )
    progress.subtask(
        "1.2",
        f"origin lat {np.degrees(lat):.6f}°, lon {np.degrees(lon):.6f}°, alt {altitude:.2f} m",
    )
    progress.subtask(
        "1.3",
        f"gravity {gravity:.6f} m/s² · Earth rate {cfg.earth_rate_rps:.6e} rad/s",
    )
    _write_json(directory / "reference.json", summary)
    figs.draw_task1(
        writer,
        directory,
        {
            "imu": imu,
            "gnss": gnss,
            "truth": truth,
            "task1": summary,
            "earth_rate_rps": cfg.earth_rate_rps,
        },
    )
    return {**summary, "lat_rad": lat, "lon_rad": lon}


# ---------------------------------------------------------------------------
# Task 2
# ---------------------------------------------------------------------------
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


def _task2(
    imu: ImuData,
    task1: dict[str, Any],
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
    start, end, variances = _rolling_static_window(imu, cfg.static_samples)
    mean_accel = np.mean(imu.accel_mps2[start:end], axis=0)
    mean_gyro = np.mean(imu.gyro_rps[start:end], axis=0)
    body_vectors = np.vstack(
        [normalize(mean_accel, "static specific force"), normalize(mean_gyro, "static angular rate")]
    )
    reference_vectors = np.vstack(
        [
            normalize(np.asarray(task1["specific_force_reference_ned_mps2"])),
            normalize(np.asarray(task1["earth_rate_reference_ned_rps"])),
        ]
    )
    summary = {
        **_task_header(2),
        "static_start_index": start,
        "static_end_index_exclusive": end,
        "static_start_time_s": float(imu.time_s[start]),
        "static_end_time_s": float(imu.time_s[end - 1]),
        "static_duration_s": float(imu.time_s[end - 1] - imu.time_s[start] + imu.dt_s),
        "static_feature_variance": float(variances[start]),
        "mean_accel_body_mps2": mean_accel,
        "mean_gyro_body_rps": mean_gyro,
        "body_vectors_unit": body_vectors,
        "reference_vectors_unit": reference_vectors,
    }
    progress.subtask(
        "2.1",
        f"{len(imu.time_s)} samples as '{cfg.imu_measurement_type}'"
        f" → rad/s and m/s² at {1.0 / imu.dt_s:.1f} Hz",
    )
    progress.subtask(
        "2.2",
        f"samples {start}–{end - 1} ({imu.time_s[start]:.3f}–{imu.time_s[end - 1]:.3f} s)"
        f", variance {float(variances[start]):.3e}",
    )
    progress.subtask(
        "2.3",
        f"|specific force| {np.linalg.norm(mean_accel):.4f} m/s²"
        f" · |angular rate| {np.linalg.norm(mean_gyro):.3e} rad/s",
    )
    _write_json(directory / "body_vectors.json", summary)
    figs.draw_task2(
        writer,
        directory,
        {
            "imu": imu,
            "task2": summary,
            "variances": variances,
            "imu_measurement_type": cfg.imu_measurement_type,
        },
    )
    return summary


# ---------------------------------------------------------------------------
# Task 3
# ---------------------------------------------------------------------------
def _task3(
    method: str,
    task1: dict[str, Any],
    task2: dict[str, Any],
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
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
        **_task_header(3),
        "method": method,
        "c_body_to_ned": c_b2n,
        "quaternion_wxyz_body_to_ned": q_wxyz,
        "quaternion_norm": float(np.linalg.norm(q_wxyz)),
        "accel_bias_body_mps2": accel_bias,
        "gyro_bias_body_rps": gyro_bias,
        **errors,
    }
    progress.subtask(
        "3.1",
        f"{method}: gravity error {errors['gravity_error_deg']:.4g}°"
        f" · Earth-rate error {errors['earth_rate_error_deg']:.4g}°",
    )
    progress.subtask(
        "3.2",
        "q_wxyz = [" + ", ".join(f"{value:.6f}" for value in q_wxyz) + "]"
        f", |q| = {float(np.linalg.norm(q_wxyz)):.12f}",
    )
    progress.subtask(
        "3.3",
        f"|accel bias| {np.linalg.norm(accel_bias):.5f} m/s²"
        f" · |gyro bias| {np.linalg.norm(gyro_bias):.3e} rad/s",
    )
    _write_json(directory / "initial_attitude.json", summary)
    figs.draw_task3(writer, directory, {"task3": summary, "method": method})
    return summary


# ---------------------------------------------------------------------------
# Task 4
# ---------------------------------------------------------------------------
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
) -> dict[str, Any]:
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
    return {
        "quaternion": quaternions,
        "acceleration": acceleration_ned,
        "position": position,
        "velocity": velocity,
        "accel_screened": accel,
        "gyro_screened": gyro,
        "range_screening": {
            "accelerometer_samples_interpolated": accel_rejected,
            "gyroscope_samples_interpolated": gyro_rejected,
        },
    }


def _task4(
    imu: ImuData,
    gnss: GnssData,
    task1: dict[str, Any],
    task3: dict[str, Any],
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
    propagated = _propagate_attitude_and_acceleration(imu, gnss, task1, task3, cfg)
    artifact = _save_npz(
        directory / "inertial_solution.npz",
        time_s=imu.time_s,
        position_ned_m=propagated["position"],
        velocity_ned_mps=propagated["velocity"],
        acceleration_ned_mps2=propagated["acceleration"],
        quaternion_wxyz=propagated["quaternion"],
    )
    summary = {
        **_task_header(4),
        "samples": len(imu.time_s),
        "duration_s": float(imu.time_s[-1]),
        "range_screening": propagated["range_screening"],
        "artifact": str(artifact),
        "final_position_ned_m": propagated["position"][-1],
        "final_velocity_ned_mps": propagated["velocity"][-1],
    }
    screened = propagated["range_screening"]
    progress.subtask(
        "4.1",
        f"{screened['accelerometer_samples_interpolated']} accel and"
        f" {screened['gyroscope_samples_interpolated']} gyro samples interpolated"
        f" beyond {cfg.max_specific_force_mps2:g} m/s² / {cfg.max_angular_rate_rps:g} rad/s",
    )
    progress.subtask(
        "4.2",
        f"{len(imu.time_s)} samples propagated"
        f", final |q| = {float(np.linalg.norm(propagated['quaternion'][-1])):.12f}",
    )
    final_position = propagated["position"][-1]
    progress.subtask(
        "4.3",
        f"final IMU-only position N {final_position[0]:.2f}"
        f" E {final_position[1]:.2f} D {final_position[2]:.2f} m",
    )
    progress.subtask(
        "4.6",
        "GNSS and IMU position, velocity, and acceleration prepared in NED, ECEF, and Body",
    )
    _write_json(directory / "summary.json", summary)
    result = {
        **summary,
        "time_s": imu.time_s,
        "position": propagated["position"],
        "velocity": propagated["velocity"],
        "acceleration": propagated["acceleration"],
        "quaternion": propagated["quaternion"],
    }
    figs.draw_task4(
        writer,
        directory,
        {
            "imu": imu,
            "gnss": gnss,
            "task1": task1,
            "task4": result,
            "accel_screened": propagated["accel_screened"],
            "gyro_screened": propagated["gyro_screened"],
            "accel_bias": task3["accel_bias_body_mps2"],
            "gyro_bias": task3["gyro_bias_body_rps"],
            "max_specific_force": cfg.max_specific_force_mps2,
            "max_angular_rate": cfg.max_angular_rate_rps,
        },
    )
    return result


# ---------------------------------------------------------------------------
# Task 5
# ---------------------------------------------------------------------------
def _gnss_ned(gnss: GnssData, task1: dict[str, Any]) -> tuple[np.ndarray, np.ndarray]:
    c = np.asarray(task1["c_ecef_to_ned"])
    origin = np.asarray(task1["origin_ecef_m"])
    return (c @ (gnss.position_ecef_m - origin).T).T, (c @ gnss.velocity_ecef_mps.T).T


def _task5(
    imu: ImuData,
    gnss: GnssData,
    task1: dict[str, Any],
    task4: dict[str, Any],
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
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
    innovation_times_array = np.asarray(innovation_times, dtype=float)
    artifact = _save_npz(
        directory / "fused_solution.npz",
        time_s=imu.time_s,
        position_ned_m=position,
        velocity_ned_mps=velocity,
        acceleration_ned_mps2=task4["acceleration"],
        quaternion_wxyz=task4["quaternion"],
        innovation_time_s=innovation_times_array,
        innovations=innovations_array,
    )
    summary = {
        **_task_header(5),
        "samples": len(imu.time_s),
        "gnss_updates": len(innovations_array),
        "artifact": str(artifact),
        "final_position_ned_m": position[-1],
        "final_velocity_ned_mps": velocity[-1],
    }
    progress.subtask("5.1", f"{len(imu.time_s)} prediction steps from Task 4 acceleration")
    progress.subtask(
        "5.2",
        f"{len(innovations_array)} GNSS updates applied"
        + (
            f" · final |innovation| {float(np.linalg.norm(innovations_array[-1, :3])):.3f} m"
            if len(innovations_array)
            else ""
        ),
    )
    progress.subtask(
        "5.3",
        f"final fused position N {position[-1, 0]:.2f}"
        f" E {position[-1, 1]:.2f} D {position[-1, 2]:.2f} m",
    )
    progress.subtask(
        "5.10",
        "final fused position, velocity, and acceleration prepared in NED, ECEF, and Body",
    )
    _write_json(directory / "summary.json", summary)
    result = {
        **summary,
        "time_s": imu.time_s,
        "position": position,
        "velocity": velocity,
        "acceleration": task4["acceleration"],
        "quaternion": task4["quaternion"],
        "innovation_times": innovation_times_array,
        "innovations": innovations_array,
    }
    figs.draw_task5(
        writer,
        directory,
        {
            "task1": task1,
            "task4": task4,
            "task5": result,
            "gnss_time_s": gnss.time_s,
            "gnss_position_ned": gnss_position,
            "gnss_velocity_ned": gnss_velocity,
        },
    )
    return result


# ---------------------------------------------------------------------------
# Task 6
# ---------------------------------------------------------------------------
def _interp_columns(source_time: np.ndarray, values: np.ndarray, target_time: np.ndarray) -> np.ndarray:
    return np.column_stack(
        [np.interp(target_time, source_time, values[:, index]) for index in range(values.shape[1])]
    )


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
                        ecef_to_ned_matrix(*ecef_to_geodetic(position)[:2])
                        @ quaternion_to_matrix(quaternion)
                    )
                    for quaternion, position in zip(truth_source, truth.position_ecef_m)
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
        estimate_q = estimate_q / np.linalg.norm(estimate_q, axis=1, keepdims=True)
        output["truth_quaternion"] = truth_q
        output["estimated_quaternion"] = align_quaternion_sign(truth_q, estimate_q)
    return output


def _task6(
    truth: TruthData | None,
    task1: dict[str, Any],
    task5: dict[str, Any],
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
    if truth is None:
        summary = {
            **_task_header(6),
            "status": "skipped",
            "reason": "No truth file was supplied; Tasks 1-5 remain valid",
        }
        _write_json(directory / "summary.json", summary)
        for number in ("6.1", "6.2", "6.3", "6.4"):
            progress.subtask(number, "skipped — no truth file was supplied")
        figs.skip_task6(writer, "no truth file was supplied")
        return summary
    overlay = _truth_on_fused_grid(truth, task1, task5, cfg)
    artifact = _save_npz(directory / "truth_overlay.npz", **overlay)
    summary = {
        **_task_header(6),
        "status": "complete",
        "samples": len(overlay["time_s"]),
        "artifact": str(artifact),
        "height_definition": "height_m = -position_ned_down_m",
        "quaternion_convention": "scalar-first [w,x,y,z], Body-to-NED, unit norm, truth-hemisphere aligned",
        "truth_source_quaternion_order": cfg.truth_quaternion_order,
        "truth_source_quaternion_frame": cfg.truth_quaternion_frame,
        "attitude_reference_frame": "time-varying local NED at each truth ECEF position",
        "truth_attitude_time_offset_s": cfg.truth_attitude_time_offset_s,
        "has_truth_attitude": "truth_quaternion" in overlay,
    }
    progress.subtask(
        "6.1",
        f"{len(overlay['time_s'])} overlapping samples"
        f" ({overlay['time_s'][0]:.3f}–{overlay['time_s'][-1]:.3f} s)"
        f" · attitude offset {cfg.truth_attitude_time_offset_s:+.3f} s",
    )
    truth_ned = overlay["truth_position"]
    progress.subtask(
        "6.2",
        f"truth NED extent N {truth_ned[:, 0].min():.1f}…{truth_ned[:, 0].max():.1f}"
        f" E {truth_ned[:, 1].min():.1f}…{truth_ned[:, 1].max():.1f} m"
        f" about the Task 1 origin",
    )
    heights = -overlay["truth_position"][:, 2]
    progress.subtask(
        "6.3",
        f"truth height {heights.min():.2f} → {heights.max():.2f} m (height = -NED Down)",
    )
    progress.subtask(
        "6.4",
        "quaternions normalised and hemisphere-aligned to truth"
        if "truth_quaternion" in overlay
        else "truth file has no quaternion columns",
    )
    _write_json(directory / "summary.json", summary)
    figs.draw_task6(
        writer,
        directory,
        {
            "overlay": overlay,
            "estimate_time_s": task5["time_s"],
            "truth_time_s": truth.time_s,
            "attitude_offset_s": cfg.truth_attitude_time_offset_s,
        },
    )
    return {**summary, "overlay": overlay}


# ---------------------------------------------------------------------------
# Task 7
# ---------------------------------------------------------------------------
def _rmse(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def _task7(
    task1: dict[str, Any],
    task5: dict[str, Any],
    task6: dict[str, Any],
    cfg: PipelineConfig,
    directory: Path,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[str, Any]:
    figure_context: dict[str, Any] = {
        "innovations": task5.get("innovations"),
        "c_ecef_to_ned": task1["c_ecef_to_ned"],
        "origin_ecef_m": task1["origin_ecef_m"],
    }
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
        attitude_error = None
        if "truth_quaternion" in overlay:
            dots = np.clip(
                np.abs(np.sum(overlay["estimated_quaternion"] * overlay["truth_quaternion"], axis=1)),
                0.0,
                1.0,
            )
            attitude_error = 2.0 * np.degrees(np.arccos(dots))
            metrics.update(
                attitude_rmse_deg=_rmse(attitude_error),
                attitude_final_deg=float(attitude_error[-1]),
                attitude_max_deg=float(np.max(attitude_error)),
            )
        artifact = _save_npz(
            directory / "residuals.npz",
            time_s=overlay["time_s"],
            position_error_ned_m=position_error,
            velocity_error_ned_mps=velocity_error,
        )
        figure_context.update(
            overlay=overlay,
            position_error=position_error,
            velocity_error=velocity_error,
            attitude_error_deg=attitude_error,
        )
        progress.subtask("7.1", f"position RMSE {metrics['position_rmse_m']:.4f} m"
                         f" · max {metrics['position_max_m']:.4f} m")
        progress.subtask("7.2", f"velocity RMSE {metrics['velocity_rmse_mps']:.4f} m/s"
                         f" · max {metrics['velocity_max_mps']:.4f} m/s")
        progress.subtask(
            "7.3",
            f"attitude RMSE {metrics['attitude_rmse_deg']:.4f}°"
            if attitude_error is not None
            else "skipped — truth file has no quaternion columns",
        )
        progress.subtask("7.4", f"{len(metrics)} scalar metrics exported")
        progress.subtask(
            "7.6",
            "NED/ECEF/Body truth overlays and detailed attitude diagnostics prepared",
        )
        status = "complete"
    else:
        innovations = task5.get("innovations", np.empty((0, 6)))
        metrics = {
            "gnss_updates": len(innovations),
            "innovation_position_rms_m": _rmse(np.linalg.norm(innovations[:, :3], axis=1)) if len(innovations) else None,
            "innovation_velocity_rms_mps": _rmse(np.linalg.norm(innovations[:, 3:], axis=1)) if len(innovations) else None,
        }
        artifact = None
        for number in ("7.1", "7.2", "7.3"):
            progress.subtask(number, "skipped — no truth overlay available")
        progress.subtask(
            "7.4",
            f"{metrics['gnss_updates']} GNSS updates"
            + (
                f" · innovation position RMS {metrics['innovation_position_rms_m']:.4f} m"
                if metrics.get("innovation_position_rms_m") is not None
                else ""
            ),
        )
        progress.subtask("7.6", "skipped — no truth overlay available")
        figure_context["overlay"] = None
        figure_context["skip_reason"] = task6.get("reason", "no truth overlay available")
        status = "complete_without_truth"
    summary = {
        **_task_header(7),
        "status": status,
        "metrics": metrics,
        "artifact": str(artifact) if artifact else None,
    }
    _write_json(directory / "metrics.json", summary)
    figure_context["metrics"] = metrics
    figs.draw_task7(writer, directory, figure_context)
    return summary


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------
def _execute_tasks(
    run_dir: Path,
    expanded: list[int],
    method: str,
    imu: ImuData,
    gnss: GnssData,
    truth: TruthData | None,
    cfg: PipelineConfig,
    writer: FigureWriter,
    progress: Progress = SILENT,
) -> dict[int, dict[str, Any]]:
    outputs: dict[int, dict[str, Any]] = {}
    last = max(expanded)

    progress.task_begin(1)
    task1 = _task1(gnss, imu, truth, cfg, _task_dir(run_dir, 1), writer, progress)
    progress.task_end(1)
    outputs[1] = task1

    if last >= 2:
        progress.task_begin(2)
        task2 = _task2(imu, task1, cfg, _task_dir(run_dir, 2), writer, progress)
        progress.task_end(2)
        outputs[2] = task2
    if last >= 3:
        progress.task_begin(3)
        task3 = _task3(method, task1, task2, cfg, _task_dir(run_dir, 3), writer, progress)
        progress.task_end(3)
        outputs[3] = task3
    if last >= 4:
        progress.task_begin(4)
        task4 = _task4(imu, gnss, task1, task3, cfg, _task_dir(run_dir, 4), writer, progress)
        progress.task_end(4)
        outputs[4] = task4
    if last >= 5:
        progress.task_begin(5)
        task5 = _task5(imu, gnss, task1, task4, cfg, _task_dir(run_dir, 5), writer, progress)
        progress.task_end(5)
        outputs[5] = task5
    if last >= 6:
        progress.task_begin(6)
        task6 = _task6(truth, task1, task5, cfg, _task_dir(run_dir, 6), writer, progress)
        progress.task_end(6)
        outputs[6] = task6
    if last >= 7:
        progress.task_begin(7)
        outputs[7] = _task7(
            task1, task5, task6, cfg, _task_dir(run_dir, 7), writer, progress
        )
        progress.task_end(7)
    return outputs


def _dataset_tags(imu_path: Path, gnss_path: Path, truth_path: Path | None) -> dict[str, str | None]:
    return {
        "imu": Path(imu_path).stem,
        "gnss": Path(gnss_path).stem,
        "truth": Path(truth_path).stem if truth_path else None,
    }


def _dataset_details(
    imu: ImuData, gnss: GnssData, truth: TruthData | None
) -> dict[str, dict[str, str] | None]:
    """Name, file and size of each input, for the progress header."""
    from .report import relative

    details: dict[str, dict[str, str] | None] = {
        "imu": {
            "name": imu.path.stem,
            "path": relative(imu.path),
            "size": f"{imu.rows} rows @ {1.0 / imu.dt_s:.1f} Hz, {imu.time_s[-1]:.2f} s",
        },
        "gnss": {
            "name": gnss.path.stem,
            "path": relative(gnss.path),
            "size": f"{gnss.rows} epochs, {gnss.time_s[-1]:.2f} s",
        },
        "truth": None,
    }
    if truth is not None:
        details["truth"] = {
            "name": truth.path.stem,
            "path": relative(truth.path),
            "size": f"{truth.rows} states, {truth.time_s[-1]:.2f} s"
            + (", with attitude" if truth.quaternion_wxyz is not None else ", no attitude"),
        }
    return details


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
    progress: Progress | None = None,
) -> dict[str, Any]:
    """Process one dataset with one method and write one folder per task."""
    progress = progress if progress is not None else SILENT
    method = _canonical_method(method)
    cfg = config if isinstance(config, PipelineConfig) else PipelineConfig.from_mapping(config)
    cfg.validate()
    requested, expanded = parse_tasks(tasks)
    imu = load_imu(imu_path, cfg.imu_layout())
    gnss = load_gnss(gnss_path, cfg.gnss_layout())
    truth = load_truth(truth_path, cfg.truth_layout()) if truth_path else None
    run_id = run_id or f"{imu.path.stem}__{gnss.path.stem}"
    safe_run_id = _safe_name(run_id)
    run_dir = Path(output_root).expanduser().resolve() / safe_run_id / method.lower()
    run_dir.mkdir(parents=True, exist_ok=True)
    writer = FigureWriter(
        run_id=safe_run_id,
        method=method,
        dataset_tags=_dataset_tags(imu.path, gnss.path, truth.path if truth else None),
        enabled=cfg.plots,
        max_points=cfg.max_plot_points,
        dpi=cfg.plot_dpi,
        progress=progress,
    )
    progress.run_begin(
        method, safe_run_id, expanded, _dataset_details(imu, gnss, truth), run_dir
    )
    started = datetime.now(timezone.utc)
    manifest: dict[str, Any] = {
        "schema_version": "3.0",
        "status": "running",
        "run_id": safe_run_id,
        "method": method,
        "requested_tasks": requested,
        "executed_tasks": expanded,
        "auto_dependencies": sorted(set(expanded) - set(requested)),
        "started_utc": started.isoformat(),
        "inputs": {
            "imu": str(imu.path),
            "gnss": str(gnss.path),
            "truth": str(truth.path) if truth else None,
        },
        "dataset_tags": writer.dataset_tags,
        "config": asdict(cfg),
        "environment": {"python": sys.version.split()[0], "platform": platform.platform()},
    }
    _write_json(run_dir / "manifest.json", manifest)
    try:
        outputs = _execute_tasks(
            run_dir, expanded, method, imu, gnss, truth, cfg, writer, progress
        )
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
    index_paths = writer.write_index(run_dir)
    finished = datetime.now(timezone.utc)
    written_figures = [
        record for record in writer.records if record["status"] == "written"
    ]
    manifest.update(
        status="complete",
        finished_utc=finished.isoformat(),
        elapsed_s=(finished - started).total_seconds(),
        task_directories={
            str(number): str(run_dir / TASK_BY_NUMBER[number].directory_name) for number in outputs
        },
        figures={
            "written": len(written_figures),
            "skipped": sum(1 for record in writer.records if record["status"] == "skipped"),
            "native_figures_written": sum(
                record.get("artifacts", {}).get("fig_status") == "written"
                for record in written_figures
            ),
            "native_figures_deferred": sum(
                record.get("artifacts", {}).get("fig_status") == "deferred"
                for record in written_figures
            ),
            "index_json": str(index_paths["json"]),
            "index_csv": str(index_paths["csv"]),
        },
        catalog=catalog_as_dict(),
    )
    if 7 in outputs:
        manifest["metrics"] = outputs[7]["metrics"]
    _write_json(run_dir / "manifest.json", manifest)
    progress.run_end(method, manifest["elapsed_s"], manifest["figures"]["written"])
    return {"run_dir": run_dir, "manifest": manifest, "tasks": outputs, "figures": writer.records}


def run_methods(
    imu_path: str | Path,
    gnss_path: str | Path,
    truth_path: str | Path | None = None,
    *,
    methods: str | Iterable[str] = METHODS,
    tasks: str | Iterable[int] = "1-7",
    output_root: str | Path = "results",
    run_id: str | None = None,
    config: PipelineConfig | dict[str, Any] | None = None,
    progress: Progress | None = None,
) -> dict[str, Any]:
    """Run any subset of methods and create the cross-method comparison."""
    progress = progress if progress is not None else SILENT
    cfg = config if isinstance(config, PipelineConfig) else PipelineConfig.from_mapping(config)
    canonical_methods = parse_methods(methods)
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
            progress=progress,
        )
        for method in canonical_methods
    }
    comparison_dir = Path(output_root).expanduser().resolve() / run_id / COMPARISON_SLUG
    comparison_dir.mkdir(parents=True, exist_ok=True)
    metrics = {
        method: result["tasks"].get(7, {}).get("metrics", {})
        for method, result in results.items()
    }
    comparison = {
        "schema_version": "3.0",
        "run_id": run_id,
        "methods": canonical_methods,
        "metrics": metrics,
        "run_directories": {method: str(result["run_dir"]) for method, result in results.items()},
        "note": (
            "Task 1-2 inputs are common; Task 3 initialization differs by method; "
            "Tasks 4-7 use that method-specific attitude."
        ),
    }
    _write_json(comparison_dir / "method_comparison.json", comparison)
    keys = sorted({key for method_metrics in metrics.values() for key in method_metrics})
    with (comparison_dir / "method_comparison.csv").open("w", encoding="utf-8", newline="") as handle:
        import csv

        writer_csv = csv.writer(handle)
        writer_csv.writerow(["method", *keys])
        for method in canonical_methods:
            writer_csv.writerow([method, *[metrics[method].get(key, "") for key in keys]])

    comparison_writer = FigureWriter(
        run_id=run_id,
        method="+".join(canonical_methods),
        dataset_tags=_dataset_tags(imu_path, gnss_path, truth_path),
        enabled=cfg.plots and all(5 in result["tasks"] for result in results.values()),
        max_points=cfg.max_plot_points,
        dpi=cfg.plot_dpi,
    )
    figs.draw_comparison(
        comparison_writer,
        comparison_dir,
        {"results": results, "metrics": metrics},
    )
    index_paths = comparison_writer.write_index(comparison_dir)
    comparison_written = [
        record
        for record in comparison_writer.records
        if record["status"] == "written"
    ]
    comparison["figures"] = {
        "written": len(comparison_written),
        "native_figures_written": sum(
            record.get("artifacts", {}).get("fig_status") == "written"
            for record in comparison_written
        ),
        "native_figures_deferred": sum(
            record.get("artifacts", {}).get("fig_status") == "deferred"
            for record in comparison_written
        ),
        "index_json": str(index_paths["json"]),
        "index_csv": str(index_paths["csv"]),
    }
    _write_json(comparison_dir / "method_comparison.json", comparison)
    return {
        "comparison_dir": comparison_dir,
        "comparison": comparison,
        "results": results,
        "figures": comparison_writer.records,
    }
