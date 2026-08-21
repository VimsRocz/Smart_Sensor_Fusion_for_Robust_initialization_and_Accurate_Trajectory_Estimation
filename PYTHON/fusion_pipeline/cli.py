"""Command-line interface for the canonical fusion pipeline."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from .attitude import METHODS
from .catalog import describe_catalog
from .contracts import InputContractError, load_gnss, load_imu, load_truth, validation_summary
from .datasets import BUNDLED, describe_datasets, resolve_dataset
from .matlab_fig import export_native_figures, find_matlab
from .pipeline import ALL_METHODS_TOKEN, PipelineConfig, parse_methods, run_methods, run_pipeline
from .report import REPORT_MODES, Progress, comparison_report, run_report


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
            raise ValueError(
                "PyYAML is required for YAML config files; use JSON or install requirements.txt"
            ) from exc
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
        prog="main.py",
        description="Run the contract-driven Tasks 1-7 GNSS/IMU fusion pipeline.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        epilog=(
            "Method options (one method is the default):\n"
            "  --method TRIAD           run TRIAD only  [default]\n"
            "  --method Davenport       run Davenport's Q-method only\n"
            "  --method SVD             run SVD/Wahba only\n"
            "  --method TRIAD,SVD       run any subset and compare them\n"
            "  --method ALL             run all three and write the comparison\n"
            "\n"
            "Dataset options:\n"
            "  --dataset x003           IMU_X003 + GNSS_X002, no truth\n"
            "  --dataset x002           IMU_X002 + GNSS_X002, no truth\n"
            "  --dataset x001           IMU_X001 + GNSS_X001 + STATE_X001\n"
            "  --dataset x001_small     short baseline for smoke tests and previews\n"
            "  --list-datasets          show every bundled dataset and its pairing\n"
            "  --no-truth               ignore the reference even if the dataset has one\n"
        ),
    )
    parser.add_argument("--config", help="JSON/YAML file with input, run and pipeline sections")
    parser.add_argument(
        "--dataset",
        choices=sorted(BUNDLED),
        help="Bundled dataset to run; resolves the verified IMU/GNSS/truth pairing",
    )
    parser.add_argument("--imu", help="IMU numeric text file (overrides --dataset)")
    parser.add_argument("--gnss", help="GNSS delimited file with ECEF position and velocity (overrides --dataset)")
    parser.add_argument("--truth", help="Optional reference-trajectory file (overrides --dataset)")
    parser.add_argument(
        "--no-truth",
        action="store_true",
        help="Run without a reference trajectory even if one is configured",
    )
    parser.add_argument(
        "--method",
        help=f"{', '.join(METHODS)}, a comma-separated subset, or {ALL_METHODS_TOKEN} (case-insensitive)",
    )
    parser.add_argument("--tasks", help="Task prefix/range, e.g. 3, 1-5 or 1-7")
    parser.add_argument("--output", help="Output root directory")
    parser.add_argument("--run-id", help="Stable output folder name")
    parser.add_argument(
        "--report",
        choices=REPORT_MODES,
        default="full",
        help="full = datasets, per-subtask figure tables and output tree; "
        "summary = task-level tables only; none = one line per run",
    )
    parser.add_argument(
        "--progress",
        choices=("on", "off"),
        default="on",
        help="Stream each task, subtask, result and figure while the run works",
    )
    parser.add_argument("--validate-only", action="store_true", help="Validate inputs and exit without running tasks")
    parser.add_argument("--no-plots", action="store_true", help="Write numeric artifacts only")
    parser.add_argument(
        "--fig",
        choices=("auto", "on", "off"),
        default="auto",
        help="auto = create editable FIGs when MATLAB is found; on = require MATLAB; off = skip FIG conversion",
    )
    parser.add_argument("--matlab-bin", help="Path to MATLAB executable (otherwise auto-detected)")
    parser.add_argument("--print-contract", action="store_true", help="Print accepted input structures and exit")
    parser.add_argument("--list-tasks", action="store_true", help="Print the task/subtask/figure catalog and exit")
    parser.add_argument("--list-datasets", action="store_true", help="Print the bundled datasets and exit")
    return parser


def _print_contract() -> None:
    print(
        """Accepted input contracts
=========================

Every layout below is the DEFAULT. If your files differ, do not rewrite them —
declare the layout in the `pipeline:` section of your configuration file.
Column indices are ZERO-BASED.

IMU (numeric text, >= 3 rows)
  default columns   0 sample/count | 1 time | 2,3,4 gyro xyz | 5,6,7 accel xyz
  config keys       imu_time_column, imu_gyro_columns, imu_accel_columns,
                    imu_delimiter, imu_time_unit (s|ms|us|ns),
                    imu_gyro_unit (rad|deg), imu_accel_unit (mps2|g|mg),
                    imu_measurement_type (delta|rate)
  default units     delta-angle [rad] and delta-velocity [m/s] per sample.
                    Use imu_measurement_type: rate for rad/s and m/s^2.

GNSS (delimited text with a header row, >= 2 data rows)
  default headers   Posix_Time, X_ECEF_m, Y_ECEF_m, Z_ECEF_m,
                    VX_ECEF_mps, VY_ECEF_mps, VZ_ECEF_mps
  config keys       gnss_column_overrides {time,x,y,z,vx,vy,vz -> your header},
                    gnss_delimiter, gnss_time_unit, gnss_position_unit (m|km|cm),
                    gnss_velocity_unit (mps|kmph|kn)
  note              Position must be true ECEF metres (norm >= 6,000 km).

Truth (optional numeric text, >= 2 rows)
  default columns   0 count | 1 time | 2,3,4 ECEF xyz | 5,6,7 ECEF velocity
                    | 8,9,10,11 quaternion
  config keys       truth_time_column, truth_position_columns,
                    truth_velocity_columns, truth_quaternion_columns (or null),
                    truth_quaternion_order (xyzw|wxyz),
                    truth_quaternion_frame (body_to_ecef|body_to_ned),
                    truth_attitude_time_offset_s, truth_delimiter,
                    truth_time_unit, truth_position_unit, truth_velocity_unit
  bundled defaults  STATE_X001 uses [qx,qy,qz,qw], Body-to-ECEF, -0.05 s offset.
  canonical output  [qw,qx,qy,qz], Body-to-NED, unit norm.

All used values must be finite. GNSS/truth time must be strictly increasing.
The known one-second-resetting IMU clock is repaired automatically and counted.
"""
    )


def _report_single(result: dict[str, Any], mode: str) -> None:
    manifest = result["manifest"]
    if mode == "none":
        figures = manifest.get("figures", {})
        print(
            f"{manifest['method']} complete: {result['run_dir']}"
            f" ({figures.get('written', 0)} figures)"
        )
        return
    print(run_report(result, mode))


def _finish_fig_export(
    export_root: Path,
    matlab_binary: Path | None,
    fig_mode: str,
) -> None:
    if fig_mode == "off":
        return
    if matlab_binary is not None:
        export_native_figures(export_root, matlab_binary)
        return
    print(
        "[INFO] MATLAB not found: PNG/PDF and editable MAT data were written; "
        "native FIG conversion is deferred.",
        file=sys.stderr,
    )


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.print_contract:
        _print_contract()
        return 0
    if args.list_tasks:
        print(describe_catalog())
        return 0
    if args.list_datasets:
        print(describe_datasets())
        return 0
    try:
        config_file = _load_config(args.config)
        input_cfg = config_file.get("input", {}) or {}
        run_cfg = config_file.get("run", {}) or {}
        pipeline_cfg = config_file.get("pipeline", {}) or {}
        if not all(isinstance(section, dict) for section in (input_cfg, run_cfg, pipeline_cfg)):
            raise ValueError("input, run and pipeline configuration sections must be mappings")
        dataset_cfg: dict[str, str | None] = {}
        dataset_name = args.dataset or run_cfg.get("dataset")
        if dataset_name:
            dataset_cfg = resolve_dataset(dataset_name).resolve()
        # Explicit paths win over --dataset, which wins over the config file.
        imu_path = args.imu or dataset_cfg.get("imu") or input_cfg.get("imu")
        gnss_path = args.gnss or dataset_cfg.get("gnss") or input_cfg.get("gnss")
        truth_path = args.truth or dataset_cfg.get("truth") or (
            input_cfg.get("truth") if not dataset_cfg else None
        )
        if args.no_truth:
            truth_path = None
        if not imu_path or not gnss_path:
            parser.error(
                "--imu and --gnss are required. Supply them directly, "
                "or use --dataset NAME (see --list-datasets), or --config FILE."
            )
        method_selection = _pick(args.method, run_cfg, "method", "TRIAD")
        tasks = _pick(args.tasks, run_cfg, "tasks", "1-7")
        output = _pick(args.output, run_cfg, "output", "results")
        run_id = _pick(args.run_id, run_cfg, "run_id")
        if run_id is None and dataset_name:
            run_id = str(dataset_name) + ("_no_truth" if truth_path is None else "")
        if args.no_plots:
            pipeline_cfg["plots"] = False
        cfg = PipelineConfig.from_mapping(pipeline_cfg)
        matlab_binary = None
        if cfg.plots and args.fig != "off":
            matlab_binary = find_matlab(args.matlab_bin)
            if matlab_binary is None and args.fig == "on":
                raise RuntimeError(
                    "Editable .fig creation was requested but MATLAB was not found. "
                    "Pass --matlab-bin /path/to/matlab or use --fig auto/off."
                )
        selected_methods = parse_methods(method_selection)
        progress = Progress(enabled=args.progress == "on")

        if args.validate_only:
            imu = load_imu(imu_path, cfg.imu_layout())
            gnss = load_gnss(gnss_path, cfg.gnss_layout())
            truth = load_truth(truth_path, cfg.truth_layout()) if truth_path else None
            print(json.dumps(validation_summary(imu, gnss, truth), indent=2))
            return 0

        if len(selected_methods) == 1:
            result = run_pipeline(
                imu_path,
                gnss_path,
                truth_path,
                method=selected_methods[0],
                tasks=tasks,
                output_root=output,
                run_id=run_id,
                config=cfg,
                progress=progress,
            )
            export_root = Path(result["run_dir"])
            if cfg.plots:
                _finish_fig_export(export_root, matlab_binary, args.fig)
            _report_single(result, args.report)
        else:
            result = run_methods(
                imu_path,
                gnss_path,
                truth_path,
                methods=selected_methods,
                tasks=tasks,
                output_root=output,
                run_id=run_id,
                config=cfg,
                progress=progress,
            )
            export_root = Path(result["comparison_dir"]).parent
            if cfg.plots:
                _finish_fig_export(export_root, matlab_binary, args.fig)
            for method_result in result["results"].values():
                _report_single(method_result, args.report)
            if args.report == "none":
                print(f"Comparison: {result['comparison_dir']}")
            else:
                print(comparison_report(result, args.report))
        return 0
    except InputContractError as exc:
        print(f"INPUT ERROR: {exc}", file=sys.stderr)
        return 2
    except (ValueError, RuntimeError) as exc:
        print(f"CONFIGURATION/PIPELINE ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
