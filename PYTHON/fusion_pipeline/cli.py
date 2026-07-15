"""Command-line interface for the canonical fusion pipeline."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from .attitude import METHODS
from .contracts import InputContractError, load_gnss, load_imu, load_truth, validation_summary
from .pipeline import PipelineConfig, run_methods, run_pipeline


def _load_config(path: str | None) -> dict[str, Any]:
    if not path:
        return {}
    resolved = Path(path).expanduser().resolve()
    if not resolved.is_file():
        raise ValueError(f"Configuration file does not exist: {resolved}")
    text = resolved.read_text(encoding="utf-8")
    if resolved.suffix.lower() == ".json":
        data = json.loads(text)
    else:
        try:
            import yaml
        except ImportError as exc:
            raise ValueError("PyYAML is required for YAML config files; use JSON or install requirements.txt") from exc
        data = yaml.safe_load(text)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ValueError("Configuration root must be an object/mapping")
    return data


def _pick(argument: Any, section: dict[str, Any], key: str, default: Any = None) -> Any:
    return argument if argument is not None else section.get(key, default)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="run_pipeline.py",
        description="Run the contract-driven Tasks 1-7 GNSS/IMU fusion pipeline.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--config", help="JSON/YAML file with input, run and pipeline sections")
    parser.add_argument("--imu", help="IMU whitespace file: sample,time,gx,gy,gz,ax,ay,az[,..]")
    parser.add_argument("--gnss", help="GNSS CSV with time, ECEF position and ECEF velocity")
    parser.add_argument("--truth", help="Optional truth file: count,time,ECEF pos/vel[,quaternion wxyz]")
    parser.add_argument("--method", help="TRIAD, Davenport, SVD, or ALL (case-insensitive)")
    parser.add_argument("--tasks", help="Task prefix/range, e.g. 3, 1-5 or 1-7")
    parser.add_argument("--output", help="Output root directory")
    parser.add_argument("--run-id", help="Stable output folder name")
    parser.add_argument("--validate-only", action="store_true", help="Validate inputs and exit without running tasks")
    parser.add_argument("--no-plots", action="store_true", help="Write numeric artifacts only")
    parser.add_argument("--print-contract", action="store_true", help="Print accepted input structures and exit")
    return parser


def _print_contract() -> None:
    print(
        """Accepted input contracts

IMU (.dat/.txt, whitespace numeric, >=3 rows, >=8 columns)
  0 sample/count | 1 time_s | 2:5 gyro xyz | 5:8 accel xyz
  Default gyro/accel values are delta-angle [rad] and delta-velocity [m/s].
  Set pipeline.imu_measurement_type: rate for rad/s and m/s^2 values.

GNSS (.csv, header + >=2 rows)
  Posix_Time, X_ECEF_m, Y_ECEF_m, Z_ECEF_m,
  VX_ECEF_mps, VY_ECEF_mps, VZ_ECEF_mps
  Common case/short aliases documented in docs/INPUT_OUTPUT_CONTRACTS.md are accepted.

Truth (optional, whitespace numeric, >=2 rows, >=8 columns)
  count, time_s, ECEF xyz [m], ECEF velocity xyz [m/s],
  optional quaternion columns; declare order/frame in configuration.
  Bundled STATE_X001 defaults: [qx,qy,qz,qw], Body-to-ECEF.
  Canonical output after conversion: [qw,qx,qy,qz], Body-to-NED.

All numeric values must be finite. GNSS/truth time must be strictly increasing.
The known one-second-resetting IMU clock is repaired automatically.
"""
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.print_contract:
        _print_contract()
        return 0
    try:
        config_file = _load_config(args.config)
        input_cfg = config_file.get("input", {}) or {}
        run_cfg = config_file.get("run", {}) or {}
        pipeline_cfg = config_file.get("pipeline", {}) or {}
        if not all(isinstance(section, dict) for section in (input_cfg, run_cfg, pipeline_cfg)):
            raise ValueError("input, run and pipeline configuration sections must be mappings")
        imu_path = _pick(args.imu, input_cfg, "imu")
        gnss_path = _pick(args.gnss, input_cfg, "gnss")
        truth_path = _pick(args.truth, input_cfg, "truth")
        if not imu_path or not gnss_path:
            parser.error("--imu and --gnss are required (directly or through --config)")
        method = _pick(args.method, run_cfg, "method", "TRIAD")
        tasks = _pick(args.tasks, run_cfg, "tasks", "1-7")
        output = _pick(args.output, run_cfg, "output", "results")
        run_id = _pick(args.run_id, run_cfg, "run_id")
        if args.no_plots:
            pipeline_cfg["plots"] = False
        cfg = PipelineConfig.from_mapping(pipeline_cfg)
        if args.validate_only:
            imu = load_imu(imu_path, cfg.imu_measurement_type)
            gnss = load_gnss(gnss_path)
            truth = load_truth(truth_path, cfg.truth_quaternion_order) if truth_path else None
            print(json.dumps(validation_summary(imu, gnss, truth), indent=2))
            return 0
        if str(method).upper() == "ALL":
            result = run_methods(
                imu_path,
                gnss_path,
                truth_path,
                methods=METHODS,
                tasks=tasks,
                output_root=output,
                run_id=run_id,
                config=cfg,
            )
            print(f"All methods complete: {result['comparison_dir']}")
            for name, method_result in result["results"].items():
                print(f"  {name}: {method_result['run_dir']}")
        else:
            result = run_pipeline(
                imu_path,
                gnss_path,
                truth_path,
                method=method,
                tasks=tasks,
                output_root=output,
                run_id=run_id,
                config=cfg,
            )
            print(f"{result['manifest']['method']} complete: {result['run_dir']}")
            if "metrics" in result["manifest"]:
                print(json.dumps(result["manifest"]["metrics"], indent=2))
        return 0
    except InputContractError as exc:
        print(f"INPUT ERROR: {exc}", file=sys.stderr)
        return 2
    except (ValueError, RuntimeError) as exc:
        print(f"CONFIGURATION/PIPELINE ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
