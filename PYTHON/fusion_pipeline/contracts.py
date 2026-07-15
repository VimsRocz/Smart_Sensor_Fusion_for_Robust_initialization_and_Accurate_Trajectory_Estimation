"""Input contracts and readers shared by every pipeline task."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


class InputContractError(ValueError):
    """Raised when an input file cannot satisfy the documented contract."""


GNSS_ALIASES = {
    "time": ("Posix_Time", "POSIX_Time", "time", "Time", "timestamp"),
    "x": ("X_ECEF_m", "X_ECEF", "ecef_x_m", "x"),
    "y": ("Y_ECEF_m", "Y_ECEF", "ecef_y_m", "y"),
    "z": ("Z_ECEF_m", "Z_ECEF", "ecef_z_m", "z"),
    "vx": ("VX_ECEF_mps", "VX_ECEF", "ecef_vx_mps", "vx"),
    "vy": ("VY_ECEF_mps", "VY_ECEF", "ecef_vy_mps", "vy"),
    "vz": ("VZ_ECEF_mps", "VZ_ECEF", "ecef_vz_mps", "vz"),
}


@dataclass(frozen=True)
class ImuData:
    path: Path
    time_s: np.ndarray
    gyro_rps: np.ndarray
    accel_mps2: np.ndarray
    dt_s: float
    rows: int
    columns: int
    clock_wraps: int


@dataclass(frozen=True)
class GnssData:
    path: Path
    time_s: np.ndarray
    position_ecef_m: np.ndarray
    velocity_ecef_mps: np.ndarray
    rows: int
    column_map: dict[str, str]


@dataclass(frozen=True)
class TruthData:
    path: Path
    time_s: np.ndarray
    position_ecef_m: np.ndarray
    velocity_ecef_mps: np.ndarray
    quaternion_wxyz: np.ndarray | None
    source_quaternion_order: str
    rows: int


def _require_file(path: str | Path, label: str) -> Path:
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise InputContractError(f"{label} file does not exist: {resolved}")
    if resolved.stat().st_size == 0:
        raise InputContractError(f"{label} file is empty: {resolved}")
    return resolved


def _finite(name: str, values: np.ndarray) -> None:
    if not np.all(np.isfinite(values)):
        bad = np.argwhere(~np.isfinite(values))[0].tolist()
        raise InputContractError(f"{name} contains NaN/Inf at index {bad}")


def _strict_time(name: str, values: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float).reshape(-1)
    if values.size < 2:
        raise InputContractError(f"{name} needs at least two timestamps")
    values = values - values[0]
    if np.any(np.diff(values) <= 0):
        raise InputContractError(f"{name} timestamps must be strictly increasing")
    return values


def _unwrap_imu_clock(raw_time: np.ndarray) -> tuple[np.ndarray, int]:
    """Unwrap the supported one-second-resetting IMU clock."""
    raw_time = np.asarray(raw_time, dtype=float).reshape(-1)
    positive = np.diff(raw_time)
    positive = positive[positive > 0]
    if positive.size == 0:
        raise InputContractError("IMU time column has no positive increments")
    dt = float(np.median(positive))
    out = np.empty_like(raw_time)
    out[0] = 0.0
    wraps = 0
    for i in range(1, raw_time.size):
        step = raw_time[i] - raw_time[i - 1]
        if step < -max(0.25, 20.0 * dt):
            wraps += 1
            step = dt
        if step <= 0:
            raise InputContractError(
                f"IMU timestamp is duplicated or reverses at row {i + 1}; "
                "only one-second clock resets are repaired automatically"
            )
        out[i] = out[i - 1] + step
    return out, wraps


def load_imu(path: str | Path, measurement_type: str = "delta") -> ImuData:
    """Load a numeric IMU file.

    Required columns are ``sample, time, gx, gy, gz, ax, ay, az``. Gyroscope
    and accelerometer columns are delta-angle/delta-velocity by default; set
    ``measurement_type='rate'`` when they already contain SI rates.
    """
    resolved = _require_file(path, "IMU")
    try:
        raw = np.loadtxt(resolved, comments="#", dtype=float)
    except Exception as exc:
        raise InputContractError(f"Cannot parse IMU as whitespace numeric data: {exc}") from exc
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    if raw.shape[0] < 3 or raw.shape[1] < 8:
        raise InputContractError(
            f"IMU requires at least 3 rows and 8 columns; got {raw.shape}"
        )
    _finite("IMU", raw[:, :8])
    time_s, wraps = _unwrap_imu_clock(raw[:, 1])
    dt = float(np.median(np.diff(time_s)))
    if not (1e-6 <= dt <= 10.0):
        raise InputContractError(f"IMU median sample interval is implausible: {dt:g} s")
    gyro = raw[:, 2:5].copy()
    accel = raw[:, 5:8].copy()
    kind = str(measurement_type).strip().lower()
    if kind == "delta":
        per_row_dt = np.r_[dt, np.diff(time_s)]
        gyro /= per_row_dt[:, None]
        accel /= per_row_dt[:, None]
    elif kind != "rate":
        raise InputContractError("imu.measurement_type must be 'delta' or 'rate'")
    _finite("IMU angular rate", gyro)
    _finite("IMU acceleration", accel)
    return ImuData(resolved, time_s, gyro, accel, dt, raw.shape[0], raw.shape[1], wraps)


def _resolve_headers(headers: list[str]) -> dict[str, str]:
    normalized = {h.strip(): h for h in headers if h is not None}
    mapping: dict[str, str] = {}
    for canonical, aliases in GNSS_ALIASES.items():
        match = next((normalized[a] for a in aliases if a in normalized), None)
        if match is None:
            raise InputContractError(
                f"GNSS is missing {canonical!r}; accepted headers: {', '.join(aliases)}"
            )
        mapping[canonical] = match
    return mapping


def load_gnss(path: str | Path) -> GnssData:
    """Load a CSV GNSS file using documented canonical headers or aliases."""
    resolved = _require_file(path, "GNSS")
    try:
        with resolved.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle)
            if not reader.fieldnames:
                raise InputContractError("GNSS CSV has no header row")
            mapping = _resolve_headers(reader.fieldnames)
            rows: list[list[float]] = []
            for row_no, row in enumerate(reader, start=2):
                if not row or all(v in (None, "") for v in row.values()):
                    continue
                try:
                    rows.append([float(row[mapping[key]]) for key in GNSS_ALIASES])
                except (TypeError, ValueError, KeyError) as exc:
                    raise InputContractError(
                        f"GNSS row {row_no} has a non-numeric required value"
                    ) from exc
    except InputContractError:
        raise
    except Exception as exc:
        raise InputContractError(f"Cannot parse GNSS CSV: {exc}") from exc
    values = np.asarray(rows, dtype=float)
    if values.shape[0] < 2:
        raise InputContractError("GNSS needs at least two data rows")
    _finite("GNSS", values)
    time_s = _strict_time("GNSS", values[:, 0])
    position = values[:, 1:4]
    velocity = values[:, 4:7]
    if np.any(np.linalg.norm(position, axis=1) < 6.0e6):
        raise InputContractError(
            "GNSS ECEF position norm must be at least 6,000 km; check units/columns"
        )
    return GnssData(resolved, time_s, position, velocity, len(values), mapping)


def load_truth(path: str | Path, quaternion_order: str = "xyzw") -> TruthData:
    """Load truth and convert its declared quaternion order to ``wxyz``."""
    resolved = _require_file(path, "Truth")
    try:
        raw = np.loadtxt(resolved, comments="#", dtype=float)
    except Exception as exc:
        raise InputContractError(f"Cannot parse truth as whitespace numeric data: {exc}") from exc
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    if raw.shape[0] < 2 or raw.shape[1] < 8:
        raise InputContractError(
            f"Truth requires at least 2 rows and 8 columns; got {raw.shape}"
        )
    _finite("Truth", raw[:, : min(raw.shape[1], 12)])
    time_s = _strict_time("Truth", raw[:, 1])
    quaternion = None
    order = str(quaternion_order).strip().lower()
    if order not in {"wxyz", "xyzw"}:
        raise InputContractError("truth_quaternion_order must be 'wxyz' or 'xyzw'")
    if raw.shape[1] >= 12:
        quaternion = raw[:, 8:12].copy()
        if order == "xyzw":
            quaternion = quaternion[:, [3, 0, 1, 2]]
        norms = np.linalg.norm(quaternion, axis=1)
        if np.any(norms < 1e-12):
            raise InputContractError("Truth contains a zero quaternion")
        quaternion /= norms[:, None]
    return TruthData(
        resolved,
        time_s,
        raw[:, 2:5].copy(),
        raw[:, 5:8].copy(),
        quaternion,
        order,
        raw.shape[0],
    )


def validation_summary(imu: ImuData, gnss: GnssData, truth: TruthData | None) -> dict[str, Any]:
    return {
        "status": "valid",
        "imu": {
            "path": str(imu.path),
            "rows": imu.rows,
            "columns": imu.columns,
            "sample_interval_s": imu.dt_s,
            "duration_s": float(imu.time_s[-1]),
            "clock_wraps_repaired": imu.clock_wraps,
        },
        "gnss": {
            "path": str(gnss.path),
            "rows": gnss.rows,
            "duration_s": float(gnss.time_s[-1]),
            "resolved_columns": gnss.column_map,
        },
        "truth": None
        if truth is None
        else {
            "path": str(truth.path),
            "rows": truth.rows,
            "duration_s": float(truth.time_s[-1]),
            "has_quaternion": truth.quaternion_wxyz is not None,
            "source_quaternion_order": truth.source_quaternion_order,
        },
    }
