#!/usr/bin/env python3
"""Run any bundled IMU x GNSS x attitude-method release combination.

The bundled cross product is 3 IMU files x 2 GNSS files x 3 methods =
18 combinations.  Every run executes the legacy full-rate Tasks 1--7 release
pipeline after checking that the inputs have compatible time coverage.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import shutil
import statistics
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
IMU_FILES = {
    "x001": ROOT / "DATA/IMU/IMU_X001.dat",
    "x002": ROOT / "DATA/IMU/IMU_X002.dat",
    "x003": ROOT / "DATA/IMU/IMU_X003.dat",
}
GNSS_FILES = {
    "x001": ROOT / "DATA/GNSS/GNSS_X001.csv",
    "x002": ROOT / "DATA/GNSS/GNSS_X002.csv",
}
TRUTH_FILE = ROOT / "DATA/Truth/STATE_X001.txt"
METHODS = ("TRIAD", "Davenport", "SVD")
GNSS_FIELDS = (
    "Posix_Time",
    "X_ECEF_m",
    "Y_ECEF_m",
    "Z_ECEF_m",
    "VX_ECEF_mps",
    "VY_ECEF_mps",
    "VZ_ECEF_mps",
)


class InputValidationError(ValueError):
    """Raised when release inputs do not satisfy the fixed-format contract."""


@dataclass(frozen=True)
class SampleInfo:
    path: Path
    rows: int
    columns: int
    sample_period_s: float
    coverage_s: float

    @property
    def sample_rate_hz(self) -> float:
        return 1.0 / self.sample_period_s


@dataclass(frozen=True)
class Combination:
    imu_id: str
    gnss_id: str
    method: str
    imu_path: Path
    gnss_path: Path


def bundled_combinations() -> list[Combination]:
    """Return the stable, documented ordering of all 18 bundled runs."""
    return [
        Combination(imu_id, gnss_id, method, IMU_FILES[imu_id], GNSS_FILES[gnss_id])
        for imu_id in IMU_FILES
        for gnss_id in GNSS_FILES
        for method in METHODS
    ]


def _normalise_method(value: str) -> str:
    methods = {method.lower(): method for method in METHODS}
    try:
        return methods[value.strip().lower()]
    except KeyError as exc:
        raise InputValidationError(
            f"Unknown method {value!r}; use TRIAD, Davenport or SVD."
        ) from exc


def _resolve_input(value: str, files: dict[str, Path]) -> tuple[str, Path]:
    key = value.strip().lower()
    if key in files:
        return key, files[key]
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    return path.stem, path.resolve()


def _median(values: list[float], label: str) -> float:
    if not values:
        raise InputValidationError(f"Cannot infer {label}: fewer than two samples.")
    value = float(statistics.median(values))
    if not math.isfinite(value) or value <= 0.0:
        raise InputValidationError(f"Invalid {label}: {value!r}.")
    return value


def inspect_imu(path: Path) -> SampleInfo:
    """Validate the fixed release IMU layout and infer its sampling period."""
    if not path.is_file():
        raise InputValidationError(f"Missing IMU file: {path}")

    rows = 0
    minimum_columns: int | None = None
    previous_time: float | None = None
    positive_steps: list[float] = []
    with path.open("r", encoding="utf-8-sig") as handle:
        for line_number, line in enumerate(handle, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) < 8:
                raise InputValidationError(
                    f"{path}:{line_number}: IMU rows need at least 8 columns; "
                    f"found {len(fields)}."
                )
            try:
                numbers = [float(value) for value in fields[:8]]
            except ValueError as exc:
                raise InputValidationError(
                    f"{path}:{line_number}: IMU columns 0--7 must be numeric."
                ) from exc
            if not all(math.isfinite(value) for value in numbers):
                raise InputValidationError(
                    f"{path}:{line_number}: IMU columns 0--7 must be finite."
                )
            time_value = numbers[1]
            if previous_time is not None and time_value > previous_time:
                # Keeping a bounded sample makes the check inexpensive on very
                # large custom logs while remaining robust to PPS rollovers.
                if len(positive_steps) < 100_000:
                    positive_steps.append(time_value - previous_time)
            previous_time = time_value
            rows += 1
            minimum_columns = len(fields) if minimum_columns is None else min(
                minimum_columns, len(fields)
            )

    if rows < 3:
        raise InputValidationError(f"{path}: IMU file needs at least 3 data rows.")
    sample_period = _median(positive_steps, "IMU sample period")
    return SampleInfo(
        path=path,
        rows=rows,
        columns=int(minimum_columns or 0),
        sample_period_s=sample_period,
        # Delta-angle/delta-velocity rows represent one complete sample each.
        coverage_s=rows * sample_period,
    )


def inspect_gnss(path: Path) -> SampleInfo:
    """Validate the release GNSS CSV layout and its ECEF/time fields."""
    if not path.is_file():
        raise InputValidationError(f"Missing GNSS file: {path}")

    times: list[float] = []
    minimum_columns: int | None = None
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        headers = [header.strip() for header in (reader.fieldnames or [])]
        missing = [field for field in GNSS_FIELDS if field not in headers]
        if missing:
            raise InputValidationError(
                f"{path}: missing GNSS headers {missing}; found {headers}."
            )
        for line_number, raw_row in enumerate(reader, 2):
            row = {(key or "").strip(): value for key, value in raw_row.items()}
            try:
                values = [float(row[field]) for field in GNSS_FIELDS]
            except (KeyError, TypeError, ValueError) as exc:
                raise InputValidationError(
                    f"{path}:{line_number}: GNSS time, ECEF position and ECEF "
                    "velocity fields must be numeric."
                ) from exc
            if not all(math.isfinite(value) for value in values):
                raise InputValidationError(
                    f"{path}:{line_number}: GNSS required fields must be finite."
                )
            position_norm = math.sqrt(sum(value * value for value in values[1:4]))
            if position_norm < 6_000_000.0:
                raise InputValidationError(
                    f"{path}:{line_number}: position norm {position_norm:.1f} m is "
                    "not a valid ECEF Earth position."
                )
            if times and values[0] <= times[-1]:
                raise InputValidationError(
                    f"{path}:{line_number}: GNSS time must strictly increase."
                )
            times.append(values[0])
            minimum_columns = len(raw_row) if minimum_columns is None else min(
                minimum_columns, len(raw_row)
            )

    if len(times) < 2:
        raise InputValidationError(f"{path}: GNSS file needs at least 2 data rows.")
    steps = [current - previous for previous, current in zip(times, times[1:])]
    sample_period = _median(steps, "GNSS sample period")
    return SampleInfo(
        path=path,
        rows=len(times),
        columns=int(minimum_columns or 0),
        sample_period_s=sample_period,
        # A 1 Hz log at t=0..1249 contains 1250 one-second measurement epochs.
        coverage_s=(times[-1] - times[0]) + sample_period,
    )


def inspect_truth(path: Path) -> SampleInfo:
    """Validate full Tasks 6--7 truth: ECEF state plus attitude quaternion."""
    if not path.is_file():
        raise InputValidationError(f"Missing truth file: {path}")

    rows = 0
    minimum_columns: int | None = None
    times: list[float] = []
    with path.open("r", encoding="utf-8-sig") as handle:
        for line_number, line in enumerate(handle, 1):
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            fields = stripped.split()
            if len(fields) < 12:
                raise InputValidationError(
                    f"{path}:{line_number}: full Tasks 6--7 truth needs 12 columns "
                    "(counter, time, ECEF position, ECEF velocity, quaternion)."
                )
            try:
                values = [float(value) for value in fields[:12]]
            except ValueError as exc:
                raise InputValidationError(
                    f"{path}:{line_number}: truth columns 0--11 must be numeric."
                ) from exc
            if not all(math.isfinite(value) for value in values):
                raise InputValidationError(
                    f"{path}:{line_number}: truth columns 0--11 must be finite."
                )
            if times and values[1] <= times[-1]:
                raise InputValidationError(
                    f"{path}:{line_number}: truth time must strictly increase."
                )
            quaternion_norm = math.sqrt(sum(value * value for value in values[8:12]))
            if quaternion_norm <= 1e-12:
                raise InputValidationError(
                    f"{path}:{line_number}: truth quaternion cannot be zero."
                )
            times.append(values[1])
            rows += 1
            minimum_columns = len(fields) if minimum_columns is None else min(
                minimum_columns, len(fields)
            )

    if rows < 2:
        raise InputValidationError(f"{path}: truth file needs at least 2 data rows.")
    steps = [current - previous for previous, current in zip(times, times[1:])]
    sample_period = _median(steps, "truth sample period")
    return SampleInfo(
        path=path,
        rows=rows,
        columns=int(minimum_columns or 0),
        sample_period_s=sample_period,
        coverage_s=times[-1] - times[0],
    )


def validate_alignment(
    imu: SampleInfo, gnss: SampleInfo, truth: SampleInfo | None = None
) -> str:
    """Require compatible coverage while allowing different sensor rates."""
    difference = abs(imu.coverage_s - gnss.coverage_s)
    tolerance = max(gnss.sample_period_s, 2.0 * imu.sample_period_s)
    if difference > tolerance:
        raise InputValidationError(
            "IMU/GNSS coverage mismatch: "
            f"IMU={imu.coverage_s:.6f} s ({imu.rows} rows), "
            f"GNSS={gnss.coverage_s:.6f} s ({gnss.rows} rows), "
            f"allowed difference={tolerance:.6f} s. Provide logs from the same "
            "time window; their row counts should differ when rates differ."
        )

    common_last_epoch = min(imu.coverage_s, gnss.coverage_s) - gnss.sample_period_s
    if truth is not None and truth.coverage_s + truth.sample_period_s < common_last_epoch:
        raise InputValidationError(
            "Truth does not cover the complete sensor window: "
            f"truth={truth.coverage_s:.6f} s, required through "
            f"{common_last_epoch:.6f} s."
        )

    ratio = gnss.sample_period_s / imu.sample_period_s
    return (
        f"aligned coverage {min(imu.coverage_s, gnss.coverage_s):.3f} s; "
        f"IMU {imu.rows:,} rows @ {imu.sample_rate_hz:.3f} Hz; "
        f"GNSS {gnss.rows:,} rows @ {gnss.sample_rate_hz:.3f} Hz; "
        f"about {ratio:.3f} IMU samples/GNSS epoch"
    )


def _dataset_tag(path: Path) -> str | None:
    match = re.search(r"X\d{3}", path.stem, flags=re.IGNORECASE)
    return match.group(0).upper() if match else None


def build_command(
    combination: Combination,
    truth_path: Path | None,
    *,
    no_plots: bool = False,
) -> list[str]:
    command = [
        sys.executable,
        str(ROOT / "PYTHON/src/GNSS_IMU_Fusion.py"),
        "--imu-file",
        str(combination.imu_path),
        "--gnss-file",
        str(combination.gnss_path),
        "--method",
        combination.method,
    ]
    if truth_path is not None:
        command.extend(("--truth-file", str(truth_path)))
        tags = tuple(
            _dataset_tag(path)
            for path in (combination.imu_path, combination.gnss_path, truth_path)
        )
        if not all(tags) or len(set(tags)) != 1:
            command.append("--allow-truth-mismatch")
    if no_plots:
        command.append("--no-plots")
    return command


def _display_command(command: Iterable[str]) -> str:
    import shlex

    return shlex.join(str(part) for part in command)


def find_matlab(explicit: str | None = None) -> Path | None:
    """Locate a MATLAB executable for automatic native ``.fig`` export."""
    candidates: list[Path] = []
    if explicit:
        candidates.append(Path(explicit).expanduser())
    discovered = shutil.which("matlab")
    if discovered:
        candidates.append(Path(discovered))
    for base in (Path("/Applications"), Path.home() / "Applications"):
        if base.is_dir():
            candidates.extend(
                sorted(base.glob("MATLAB*.app/bin/matlab"), reverse=True)
            )
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    return None


def export_native_figures(results_dir: Path, matlab_binary: Path) -> int:
    """Use MATLAB once to turn every result PNG into a directly openable FIG."""
    results_dir.mkdir(parents=True, exist_ok=True)

    def matlab_string(path: Path) -> str:
        return str(path).replace("'", "''")

    matlab_dir = ROOT / "MATLAB"
    expression = (
        f"addpath('{matlab_string(matlab_dir)}'); "
        f"export_release_figures('{matlab_string(results_dir.resolve())}');"
    )
    subprocess.run(
        [str(matlab_binary), "-batch", expression],
        cwd=ROOT,
        check=True,
    )
    pngs = sorted(results_dir.rglob("*.png"))
    missing = [png for png in pngs if not png.with_suffix(".fig").is_file()]
    if missing:
        preview = "\n".join(f"  {path}" for path in missing[:10])
        raise RuntimeError(
            f"MATLAB FIG audit failed: {len(missing)} of {len(pngs)} PNG files "
            f"have no native .fig companion.\n{preview}"
        )
    print(f"MATLAB FIG audit: {len(pngs)}/{len(pngs)} PNG files have native .fig companions")
    return len(pngs)


def apply_matlab_policy(
    matlab_binary: Path | None,
    *,
    plots_enabled: bool,
    defer_fig: bool,
) -> None:
    """Require MATLAB unless the user explicitly chooses two-stage export.

    A deferred run computes PNG/MAT here and performs only the lightweight FIG
    serialization later on another system that has MATLAB.
    """
    if not plots_enabled or matlab_binary is not None:
        return
    message = (
        "MATLAB was not found, so genuine native .fig files cannot be created "
        "on this system. Use --defer-fig to compute PNG/MAT now, then copy the "
        "results directory to a MATLAB system and run its bundled "
        "export_release_figures.m once. That conversion does not rerun fusion."
    )
    if not defer_fig:
        raise InputValidationError(message)
    print(f"WARNING: {message}", file=sys.stderr)


def prepare_deferred_fig_bundle(results_dir: Path) -> None:
    """Make a results directory self-contained for FIG conversion elsewhere."""
    results_dir.mkdir(parents=True, exist_ok=True)
    exporter = results_dir / "export_release_figures.m"
    shutil.copy2(ROOT / "MATLAB/export_release_figures.m", exporter)
    instructions = results_dir / "CREATE_NATIVE_FIGS.txt"
    instructions.write_text(
        "NATIVE MATLAB FIG CONVERSION (NO FUSION RECOMPUTATION)\n\n"
        "1. Copy this complete results directory to a system with MATLAB.\n"
        "2. In MATLAB, change Current Folder to this directory.\n"
        "3. Run:  export_release_figures(pwd)\n"
        "4. Every PNG will then have a same-stem native FIG. Open any FIG "
        "directly with openfig or by double-clicking it.\n",
        encoding="utf-8",
    )
    print(f"Deferred FIG conversion bundle: {exporter}")
    print(f"Deferred FIG instructions: {instructions}")


def _print_combinations() -> None:
    print("#  IMU   GNSS  Method")
    for index, combination in enumerate(bundled_combinations(), 1):
        print(
            f"{index:2d} {combination.imu_id.upper():5s} "
            f"{combination.gnss_id.upper():5s} {combination.method}"
        )


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run one or all 18 full Tasks 1-7 IMU/GNSS/method combinations."
    )
    parser.add_argument("--imu", default="x001", help="x001/x002/x003 or IMU path")
    parser.add_argument("--gnss", default="x001", help="x001/x002 or GNSS CSV path")
    parser.add_argument("--method", default="TRIAD", help="TRIAD, Davenport or SVD")
    parser.add_argument("--truth", help="truth path (default: bundled STATE_X001)")
    parser.add_argument("--no-truth", action="store_true", help="skip truth comparisons")
    parser.add_argument("--all", action="store_true", help="run all 18 bundled combinations")
    parser.add_argument("--list", action="store_true", help="list all 18 and exit")
    parser.add_argument(
        "--check-only", action="store_true", help="validate inputs and synchronization only"
    )
    parser.add_argument("--no-plots", action="store_true", help="run numeric tasks only")
    parser.add_argument(
        "--matlab-bin", help="path to MATLAB executable (auto-detected when omitted)"
    )
    parser.add_argument(
        "--defer-fig",
        "--allow-missing-fig",
        dest="defer_fig",
        action="store_true",
        help=(
            "compute PNG/MAT without MATLAB and bundle a no-recompute FIG "
            "converter for a MATLAB system"
        ),
    )
    parser.add_argument(
        "--export-figs-only",
        action="store_true",
        help="create/audit FIG companions for existing PNG results, then exit",
    )
    parser.add_argument(
        "--output", default=str(ROOT / "results"), help="results directory"
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    if args.list:
        _print_combinations()
        return 0
    matlab_binary = find_matlab(args.matlab_bin)
    output_dir = Path(args.output).expanduser().resolve()
    if args.export_figs_only:
        if matlab_binary is None:
            raise InputValidationError(
                "Native .fig creation requires MATLAB. Install MATLAB or pass "
                "--matlab-bin /path/to/matlab. A .mat file is data and cannot be "
                "opened as a figure by double-clicking."
            )
        export_native_figures(output_dir, matlab_binary)
        return 0
    if args.no_truth and args.truth:
        raise InputValidationError("Use either --truth or --no-truth, not both.")

    if args.all:
        combinations = bundled_combinations()
    else:
        imu_id, imu_path = _resolve_input(args.imu, IMU_FILES)
        gnss_id, gnss_path = _resolve_input(args.gnss, GNSS_FILES)
        combinations = [
            Combination(
                imu_id=imu_id,
                gnss_id=gnss_id,
                method=_normalise_method(args.method),
                imu_path=imu_path,
                gnss_path=gnss_path,
            )
        ]

    truth_path = None
    if not args.no_truth:
        truth_path = Path(args.truth).expanduser() if args.truth else TRUTH_FILE
        if not truth_path.is_absolute():
            truth_path = ROOT / truth_path
        truth_path = truth_path.resolve()

    imu_cache: dict[Path, SampleInfo] = {}
    gnss_cache: dict[Path, SampleInfo] = {}
    truth_info = inspect_truth(truth_path) if truth_path is not None else None
    print("Preflight: validating file layouts and synchronized coverage")
    seen_pairs: set[tuple[Path, Path]] = set()
    for combination in combinations:
        pair = (combination.imu_path, combination.gnss_path)
        if pair in seen_pairs:
            continue
        seen_pairs.add(pair)
        if combination.imu_path not in imu_cache:
            imu_cache[combination.imu_path] = inspect_imu(combination.imu_path)
        if combination.gnss_path not in gnss_cache:
            gnss_cache[combination.gnss_path] = inspect_gnss(combination.gnss_path)
        imu_info = imu_cache[combination.imu_path]
        gnss_info = gnss_cache[combination.gnss_path]
        message = validate_alignment(imu_info, gnss_info, truth_info)
        print(
            f"  {combination.imu_id.upper()} + {combination.gnss_id.upper()}: {message}"
        )

    if args.check_only:
        print(f"Preflight passed for {len(combinations)} combination(s); nothing was run.")
        return 0

    apply_matlab_policy(
        matlab_binary,
        plots_enabled=not args.no_plots,
        defer_fig=args.defer_fig,
    )
    if args.defer_fig and matlab_binary is None and not args.no_plots:
        prepare_deferred_fig_bundle(output_dir)

    if len(combinations) == 18:
        output_parent = output_dir.parent
        while not output_parent.exists():
            output_parent = output_parent.parent
        free_gib = shutil.disk_usage(output_parent).free / 1024**3
        if free_gib < 20.0:
            print(
                f"WARNING: only {free_gib:.1f} GiB is free. All 18 full runs can "
                "require more than 20 GiB; choose --output on a larger disk or run "
                "individual combinations.",
                file=sys.stderr,
            )

    environment = os.environ.copy()
    cache_root = Path(tempfile.gettempdir()) / "sensor-fusion-release-cache"
    (cache_root / "matplotlib").mkdir(parents=True, exist_ok=True)
    environment.setdefault("MPLCONFIGDIR", str(cache_root / "matplotlib"))
    environment.setdefault("XDG_CACHE_HOME", str(cache_root))
    environment["PYTHON_RESULTS_DIR"] = str(output_dir)

    for index, combination in enumerate(combinations, 1):
        command = build_command(combination, truth_path, no_plots=args.no_plots)
        print("=" * 78)
        print(
            f"Combination {index}/{len(combinations)}: "
            f"{combination.imu_id.upper()} + {combination.gnss_id.upper()} + "
            f"{combination.method}"
        )
        print(_display_command(command))
        subprocess.run(command, cwd=ROOT, env=environment, check=True)
        if not args.no_plots and matlab_binary is not None:
            export_native_figures(output_dir, matlab_binary)

    print(f"Completed {len(combinations)} combination(s). Results: {environment['PYTHON_RESULTS_DIR']}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except InputValidationError as exc:
        print(f"Input validation failed: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
