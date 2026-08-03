"""Input contracts and readers shared by every pipeline task.

The default layouts match the bundled ``DATA/`` files. A different logger can be
read without editing the data by declaring its column indices, delimiter, and
units in the ``pipeline`` section of the run configuration — see
``docs/INPUT_OUTPUT_CONTRACTS.md``.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Sequence

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

TIME_UNIT_SCALE = {"s": 1.0, "ms": 1.0e-3, "us": 1.0e-6, "ns": 1.0e-9}
ANGLE_UNIT_SCALE = {"rad": 1.0, "deg": np.pi / 180.0}
#: Unit definition of the "g" symbol, not the local gravity used by Task 1.
ACCEL_UNIT_SCALE = {"mps2": 1.0, "g": 9.80665, "mg": 9.80665e-3}
LENGTH_UNIT_SCALE = {"m": 1.0, "km": 1000.0, "cm": 0.01}
VELOCITY_UNIT_SCALE = {"mps": 1.0, "kmph": 1000.0 / 3600.0, "kn": 1852.0 / 3600.0}


def _scale(table: dict[str, float], unit: str, label: str) -> float:
    key = str(unit).strip().lower()
    if key not in table:
        raise InputContractError(
            f"{label} must be one of {', '.join(sorted(table))}; got {unit!r}"
        )
    return table[key]


def _triple(values: Sequence[int], label: str) -> tuple[int, int, int]:
    indices = [int(value) for value in values]
    if len(indices) != 3:
        raise InputContractError(f"{label} must list exactly three column indices")
    if any(index < 0 for index in indices):
        raise InputContractError(f"{label} column indices must be zero-based and non-negative")
    return tuple(indices)  # type: ignore[return-value]


# ---------------------------------------------------------------------------
# Layout declarations
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ImuLayout:
    """Where the IMU quantities live in the file and in which units."""

    time_column: int = 1
    gyro_columns: tuple[int, int, int] = (2, 3, 4)
    accel_columns: tuple[int, int, int] = (5, 6, 7)
    measurement_type: str = "delta"  # delta increments or already-SI rates
    time_unit: str = "s"
    gyro_unit: str = "rad"
    accel_unit: str = "mps2"
    delimiter: str | None = None  # None = any run of whitespace
    comment: str = "#"

    def validate(self) -> None:
        if int(self.time_column) < 0:
            raise InputContractError("imu_time_column must be zero-based and non-negative")
        _triple(self.gyro_columns, "imu_gyro_columns")
        _triple(self.accel_columns, "imu_accel_columns")
        if str(self.measurement_type).strip().lower() not in {"delta", "rate"}:
            raise InputContractError("imu_measurement_type must be 'delta' or 'rate'")
        _scale(TIME_UNIT_SCALE, self.time_unit, "imu_time_unit")
        _scale(ANGLE_UNIT_SCALE, self.gyro_unit, "imu_gyro_unit")
        _scale(ACCEL_UNIT_SCALE, self.accel_unit, "imu_accel_unit")

    @property
    def required_columns(self) -> int:
        return 1 + max([int(self.time_column), *self.gyro_columns, *self.accel_columns])


@dataclass(frozen=True)
class GnssLayout:
    """How to find the GNSS columns in a delimited text file."""

    #: Maps a canonical name (time, x, y, z, vx, vy, vz) to the exact header in
    #: the file. Anything omitted falls back to the documented alias list.
    column_overrides: dict[str, str] = field(default_factory=dict)
    delimiter: str = ","
    time_unit: str = "s"
    position_unit: str = "m"
    velocity_unit: str = "mps"

    def validate(self) -> None:
        unknown = sorted(set(self.column_overrides) - set(GNSS_ALIASES))
        if unknown:
            raise InputContractError(
                f"gnss_column_overrides keys must be in {sorted(GNSS_ALIASES)}; got {unknown}"
            )
        if not isinstance(self.delimiter, str) or len(self.delimiter) != 1:
            raise InputContractError("gnss_delimiter must be a single character")
        _scale(TIME_UNIT_SCALE, self.time_unit, "gnss_time_unit")
        _scale(LENGTH_UNIT_SCALE, self.position_unit, "gnss_position_unit")
        if str(self.velocity_unit).strip().lower() not in VELOCITY_UNIT_SCALE:
            raise InputContractError(
                f"gnss_velocity_unit must be one of {', '.join(sorted(VELOCITY_UNIT_SCALE))}"
            )

    @property
    def velocity_scale(self) -> float:
        return VELOCITY_UNIT_SCALE[str(self.velocity_unit).strip().lower()]


@dataclass(frozen=True)
class TruthLayout:
    """Where the reference-trajectory quantities live."""

    time_column: int = 1
    position_columns: tuple[int, int, int] = (2, 3, 4)
    velocity_columns: tuple[int, int, int] = (5, 6, 7)
    #: Four columns, or None when the truth file carries no attitude.
    quaternion_columns: tuple[int, int, int, int] | None = (8, 9, 10, 11)
    quaternion_order: str = "xyzw"
    time_unit: str = "s"
    position_unit: str = "m"
    velocity_unit: str = "mps"
    delimiter: str | None = None
    comment: str = "#"

    def validate(self) -> None:
        if int(self.time_column) < 0:
            raise InputContractError("truth_time_column must be zero-based and non-negative")
        _triple(self.position_columns, "truth_position_columns")
        _triple(self.velocity_columns, "truth_velocity_columns")
        if self.quaternion_columns is not None:
            indices = [int(value) for value in self.quaternion_columns]
            if len(indices) != 4 or any(index < 0 for index in indices):
                raise InputContractError(
                    "truth_quaternion_columns must list four non-negative column indices, or be null"
                )
        if str(self.quaternion_order).strip().lower() not in {"wxyz", "xyzw"}:
            raise InputContractError("truth_quaternion_order must be 'wxyz' or 'xyzw'")
        _scale(TIME_UNIT_SCALE, self.time_unit, "truth_time_unit")
        _scale(LENGTH_UNIT_SCALE, self.position_unit, "truth_position_unit")
        if str(self.velocity_unit).strip().lower() not in VELOCITY_UNIT_SCALE:
            raise InputContractError(
                f"truth_velocity_unit must be one of {', '.join(sorted(VELOCITY_UNIT_SCALE))}"
            )

    @property
    def velocity_scale(self) -> float:
        return VELOCITY_UNIT_SCALE[str(self.velocity_unit).strip().lower()]

    @property
    def required_columns(self) -> int:
        indices = [int(self.time_column), *self.position_columns, *self.velocity_columns]
        return 1 + max(indices)


# ---------------------------------------------------------------------------
# Loaded data
# ---------------------------------------------------------------------------
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
    layout: ImuLayout


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


def _load_numeric(path: Path, label: str, delimiter: str | None, comment: str) -> np.ndarray:
    try:
        raw = np.loadtxt(path, comments=comment, delimiter=delimiter, dtype=float)
    except Exception as exc:
        hint = "whitespace-separated" if delimiter is None else f"{delimiter!r}-separated"
        raise InputContractError(
            f"Cannot parse {label} as {hint} numeric data: {exc}"
        ) from exc
    if raw.ndim == 1:
        raw = raw.reshape(1, -1)
    return raw


def _check_columns(label: str, raw: np.ndarray, needed: int, hint: str) -> None:
    if raw.shape[1] < needed:
        raise InputContractError(
            f"{label} needs at least {needed} columns for the configured layout "
            f"but has {raw.shape[1]}. {hint}"
        )


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


def load_imu(path: str | Path, layout: ImuLayout | str | None = None) -> ImuData:
    """Load a numeric IMU file according to ``layout``.

    ``layout`` may also be the string ``"delta"`` or ``"rate"`` as a shorthand
    for the default column layout with that measurement type.
    """
    if layout is None:
        layout = ImuLayout()
    elif isinstance(layout, str):
        layout = ImuLayout(measurement_type=layout)
    layout.validate()

    resolved = _require_file(path, "IMU")
    raw = _load_numeric(resolved, "IMU", layout.delimiter, layout.comment)
    if raw.shape[0] < 3:
        raise InputContractError(f"IMU requires at least 3 rows; got {raw.shape[0]}")
    _check_columns(
        "IMU",
        raw,
        layout.required_columns,
        "Set imu_time_column / imu_gyro_columns / imu_accel_columns to match your file.",
    )

    used = np.column_stack(
        [
            raw[:, layout.time_column],
            raw[:, list(layout.gyro_columns)],
            raw[:, list(layout.accel_columns)],
        ]
    )
    _finite("IMU", used)

    time_raw = raw[:, layout.time_column] * _scale(TIME_UNIT_SCALE, layout.time_unit, "imu_time_unit")
    time_s, wraps = _unwrap_imu_clock(time_raw)
    dt = float(np.median(np.diff(time_s)))
    if not (1e-6 <= dt <= 10.0):
        raise InputContractError(
            f"IMU median sample interval is implausible: {dt:g} s. "
            "Check imu_time_column and imu_time_unit."
        )

    gyro = raw[:, list(layout.gyro_columns)] * _scale(ANGLE_UNIT_SCALE, layout.gyro_unit, "imu_gyro_unit")
    accel = raw[:, list(layout.accel_columns)] * _scale(ACCEL_UNIT_SCALE, layout.accel_unit, "imu_accel_unit")
    if str(layout.measurement_type).strip().lower() == "delta":
        per_row_dt = np.r_[dt, np.diff(time_s)]
        gyro = gyro / per_row_dt[:, None]
        accel = accel / per_row_dt[:, None]
    _finite("IMU angular rate", gyro)
    _finite("IMU acceleration", accel)
    return ImuData(resolved, time_s, gyro, accel, dt, raw.shape[0], raw.shape[1], wraps, layout)


def _resolve_headers(headers: list[str], overrides: dict[str, str]) -> dict[str, str]:
    normalized = {h.strip(): h for h in headers if h is not None}
    mapping: dict[str, str] = {}
    for canonical, aliases in GNSS_ALIASES.items():
        override = overrides.get(canonical)
        if override is not None:
            if override.strip() not in normalized:
                raise InputContractError(
                    f"gnss_column_overrides maps {canonical!r} to {override!r}, "
                    f"which is not a header in the file: {', '.join(sorted(normalized))}"
                )
            mapping[canonical] = normalized[override.strip()]
            continue
        match = next((normalized[a] for a in aliases if a in normalized), None)
        if match is None:
            raise InputContractError(
                f"GNSS is missing {canonical!r}; accepted headers: {', '.join(aliases)}. "
                f"Use gnss_column_overrides to name your own column."
            )
        mapping[canonical] = match
    return mapping


def load_gnss(path: str | Path, layout: GnssLayout | None = None) -> GnssData:
    """Load a delimited GNSS file using canonical headers, aliases or overrides."""
    layout = layout or GnssLayout()
    layout.validate()
    resolved = _require_file(path, "GNSS")
    try:
        with resolved.open("r", encoding="utf-8-sig", newline="") as handle:
            reader = csv.DictReader(handle, delimiter=layout.delimiter)
            if not reader.fieldnames:
                raise InputContractError("GNSS file has no header row")
            mapping = _resolve_headers(reader.fieldnames, layout.column_overrides)
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
        raise InputContractError(f"Cannot parse GNSS file: {exc}") from exc
    values = np.asarray(rows, dtype=float)
    if values.shape[0] < 2:
        raise InputContractError("GNSS needs at least two data rows")
    _finite("GNSS", values)
    time_s = _strict_time("GNSS", values[:, 0] * _scale(TIME_UNIT_SCALE, layout.time_unit, "gnss_time_unit"))
    position = values[:, 1:4] * _scale(LENGTH_UNIT_SCALE, layout.position_unit, "gnss_position_unit")
    velocity = values[:, 4:7] * layout.velocity_scale
    if np.any(np.linalg.norm(position, axis=1) < 6.0e6):
        raise InputContractError(
            "GNSS position norm must be at least 6,000 km, i.e. genuine ECEF metres. "
            "Check gnss_position_unit and that the X/Y/Z columns are ECEF, not local."
        )
    return GnssData(resolved, time_s, position, velocity, len(values), mapping)


def load_truth(path: str | Path, layout: TruthLayout | str | None = None) -> TruthData:
    """Load truth and convert its declared quaternion order to ``wxyz``."""
    if layout is None:
        layout = TruthLayout()
    elif isinstance(layout, str):
        layout = TruthLayout(quaternion_order=layout)
    layout.validate()

    resolved = _require_file(path, "Truth")
    raw = _load_numeric(resolved, "Truth", layout.delimiter, layout.comment)
    if raw.shape[0] < 2:
        raise InputContractError(f"Truth requires at least 2 rows; got {raw.shape[0]}")
    _check_columns(
        "Truth",
        raw,
        layout.required_columns,
        "Set truth_time_column / truth_position_columns / truth_velocity_columns to match your file.",
    )

    position_scale = _scale(LENGTH_UNIT_SCALE, layout.position_unit, "truth_position_unit")
    used = np.column_stack(
        [
            raw[:, layout.time_column],
            raw[:, list(layout.position_columns)],
            raw[:, list(layout.velocity_columns)],
        ]
    )
    _finite("Truth", used)
    time_s = _strict_time(
        "Truth", raw[:, layout.time_column] * _scale(TIME_UNIT_SCALE, layout.time_unit, "truth_time_unit")
    )

    quaternion = None
    order = str(layout.quaternion_order).strip().lower()
    if layout.quaternion_columns is not None:
        needed = 1 + max(int(index) for index in layout.quaternion_columns)
        if raw.shape[1] < needed:
            raise InputContractError(
                f"Truth quaternion needs {needed} columns but the file has {raw.shape[1]}. "
                "Set truth_quaternion_columns to null if your truth has no attitude."
            )
        quaternion = raw[:, list(layout.quaternion_columns)].copy()
        _finite("Truth quaternion", quaternion)
        if order == "xyzw":
            quaternion = quaternion[:, [3, 0, 1, 2]]
        norms = np.linalg.norm(quaternion, axis=1)
        if np.any(norms < 1e-12):
            raise InputContractError("Truth contains a zero quaternion")
        quaternion /= norms[:, None]

    return TruthData(
        resolved,
        time_s,
        raw[:, list(layout.position_columns)] * position_scale,
        raw[:, list(layout.velocity_columns)] * layout.velocity_scale,
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
            "sample_rate_hz": 1.0 / imu.dt_s if imu.dt_s else None,
            "duration_s": float(imu.time_s[-1]),
            "clock_wraps_repaired": imu.clock_wraps,
            "measurement_type": imu.layout.measurement_type,
            "time_column": imu.layout.time_column,
            "gyro_columns": list(imu.layout.gyro_columns),
            "accel_columns": list(imu.layout.accel_columns),
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
