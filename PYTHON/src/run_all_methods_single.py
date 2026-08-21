#!/usr/bin/env python3
"""Run all attitude initialisation methods on a single dataset.

This helper executes ``run_triad_only.py``, ``run_davenport_only.py`` and
``run_svd_only.py`` sequentially for the provided IMU and GNSS files. All
subprocess output is forwarded to the console in real time. Lines beginning
with ``[SUMMARY]`` are collected and printed again at the end so results from
all methods can be compared easily.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path
import os

from dependency_bootstrap import ensure_dependencies

ensure_dependencies()

SCRIPT_MAP = {
    "TRIAD": "run_triad_only.py",
    "Davenport": "run_davenport_only.py",
    "SVD": "run_svd_only.py",
}

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")


def run_method(
    script: str,
    imu: str,
    gnss: str,
    dataset: str,
    truth: str | None,
    allow_truth_mismatch: bool = False,
    auto_truth: bool = True,
):
    cmd = [
        sys.executable,
        "-u",  # unbuffered to stream child output in real time
        str(Path(__file__).resolve().parent / script),
        "--imu",
        imu,
        "--gnss",
        gnss,
        "--dataset",
        dataset,
    ]
    if truth:
        cmd += ["--truth", truth]
        if allow_truth_mismatch:
            cmd += ["--allow-truth-mismatch"]
    else:
        if not auto_truth:
            cmd += ["--no-auto-truth"]

    import time as _time
    t0 = _time.time()
    env = os.environ.copy()
    # Suppress repeated results note in child scripts
    env["RESULTS_NOTE_PRINTED"] = "1"
    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        env=env,
    )
    summaries: list[str] = []
    assert proc.stdout is not None
    for line in proc.stdout:
        print(line, end="")
        m = SUMMARY_RE.search(line)
        if m:
            summaries.append(m.group(1))
    proc.wait()
    elapsed = _time.time() - t0
    return proc.returncode, summaries, elapsed


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run TRIAD, Davenport and SVD on one dataset",
    )
    parser.add_argument("--imu", required=True, help="Path to IMU .dat file")
    parser.add_argument("--gnss", required=True, help="Path to GNSS .csv file")
    parser.add_argument("--dataset", default="X002", help="Dataset identifier")
    parser.add_argument("--truth", help="Optional truth file for evaluation")
    parser.add_argument(
        "--allow-truth-mismatch",
        action="store_true",
        help="Allow using a truth file from a different dataset (use with care)",
    )
    parser.add_argument(
        "--auto-truth",
        dest="auto_truth",
        action="store_true",
        default=True,
        help="Auto-detect truth in child scripts if --truth not provided (default on)",
    )
    parser.add_argument(
        "--no-auto-truth",
        dest="auto_truth",
        action="store_false",
        help="Do not auto-detect truth in child scripts when --truth is omitted",
    )
    args = parser.parse_args(argv)

    # Determine dataset id and choose summary output directory
    import re as _re
    ds_match = None
    for s in (args.imu or "", args.gnss or ""):
        m = _re.search(r"(X\d{3})", Path(s).name)
        if m:
            ds_match = m.group(1)
            break
    ds_id = ds_match or args.dataset or "X002"
    base_results = Path(__file__).resolve().parents[2] / "PYTHON" / "results"
    summary_dir = base_results / "AllMethods" / ds_id
    summary_dir.mkdir(parents=True, exist_ok=True)

    # Print a concise header similar to individual scripts
    print("=== ALL METHODS ===")
    print(
        "Resolved input files: imu=%s gnss=%s truth=%s"
        % (args.imu, args.gnss, args.truth if args.truth else "(none)")
    )
    print("Note: Python saves to results/ ; MATLAB saves to MATLAB/results/ (independent).")

    all_summaries: list[tuple[str, str]] = []
    rows = []  # for final pretty summary
    for method, script in SCRIPT_MAP.items():
        print(f"\n=== {method} ===")
        ret, summaries, elapsed = run_method(
            script,
            args.imu,
            args.gnss,
            args.dataset,
            args.truth,
            allow_truth_mismatch=args.allow_truth_mismatch,
            auto_truth=args.auto_truth,
        )
        if ret != 0:
            print(f"\n{method} run failed with exit code {ret}")
        # Collect raw summary lines
        for s in summaries:
            all_summaries.append((method, s))
        # Parse metrics for table if available (prefer last GNSS_IMU_Fusion summary)
        kv = {}
        for s in summaries:
            for k, v in re.findall(r"(\w+)=\s*([^\s]+)", s):
                kv[k] = v
        def _to_float(val):
            try:
                return float(str(val).rstrip('m').rstrip('s'))
            except Exception:
                return float("nan")
        rows.append({
            "method": method,
            "rmse_pos": _to_float(kv.get("rmse_pos", "nan")),
            "final_pos": _to_float(kv.get("final_pos", "nan")),
            "rms_resid_pos": _to_float(kv.get("rms_resid_pos", "nan")),
            "max_resid_pos": _to_float(kv.get("max_resid_pos", "nan")),
            "rms_resid_vel": _to_float(kv.get("rms_resid_vel", "nan")),
            "max_resid_vel": _to_float(kv.get("max_resid_vel", "nan")),
            "accel_bias": _to_float(kv.get("accel_bias", "nan")),
            "gyro_bias": _to_float(kv.get("gyro_bias", "nan")),
            "att_err_deg": _to_float(kv.get("att_err_deg", "nan")),
            "zupt": _to_float(kv.get("ZUPT_count", "nan")),
            "grav_mean": _to_float(kv.get("GravErrMean_deg", "nan")),
            "grav_max": _to_float(kv.get("GravErrMax_deg", "nan")),
            "earth_mean": _to_float(kv.get("EarthRateErrMean_deg", "nan")),
            "earth_max": _to_float(kv.get("EarthRateErrMax_deg", "nan")),
            "elapsed": float(elapsed),
        })

    if all_summaries:
        print("\n=== Summary (raw) ===")
        for method, line in all_summaries:
            print(f"{method}: {line}")

    # Pretty table with key metrics if any were parsed
    if rows:
        print("\n=== Summary (metrics) ===")
        # Compute widths
        def fmt(x, w):
            try:
                return f"{x:.2f}".rjust(w)
            except Exception:
                return str(x).rjust(w)
        m_w = max(6, max(len(r["method"]) for r in rows))
        print(" ".join([
            "Method".ljust(m_w),
            "RMSE[m]".rjust(8),
            "Final[m]".rjust(9),
            "RMSrPos".rjust(8),
            "MaxrPos".rjust(8),
            "RMSrVel".rjust(8),
            "MaxrVel".rjust(8),
            "AccelBias".rjust(9),
            "GyroBias".rjust(8),
            "ZUPT".rjust(6),
            "GravMean".rjust(9),
            "GravMax".rjust(8),
            "EarthMean".rjust(10),
            "EarthMax".rjust(9),
            "AttErr[deg]".rjust(12),
            "Elapsed[s]".rjust(11),
        ]))
        for r in rows:
            print(" ".join([
                r["method"].ljust(m_w),
                fmt(r["rmse_pos"], 8),
                fmt(r["final_pos"], 9),
                fmt(r["rms_resid_pos"], 8),
                fmt(r["max_resid_pos"], 8),
                fmt(r["rms_resid_vel"], 8),
                fmt(r["max_resid_vel"], 8),
                fmt(r["accel_bias"], 9),
                fmt(r["gyro_bias"], 8),
                fmt(r["zupt"], 6),
                fmt(r["grav_mean"], 9),
                fmt(r["grav_max"], 8),
                fmt(r["earth_mean"], 10),
                fmt(r["earth_max"], 9),
                fmt(r["att_err_deg"], 12),
                fmt(r["elapsed"], 11),
            ]))

        # Save CSV summary under AllMethods/<dataset>
        try:
            import csv as _csv
            csv_path = summary_dir / f"all_methods_{ds_id}_summary.csv"
            with csv_path.open("w", newline="") as f:
                w = _csv.writer(f)
                w.writerow([
                    "method","rmse_pos","final_pos","rms_resid_pos","max_resid_pos",
                    "rms_resid_vel","max_resid_vel","accel_bias","gyro_bias","zupt",
                    "grav_mean","grav_max","earth_mean","earth_max","att_err_deg","elapsed_s"
                ])
                for r in rows:
                    w.writerow([
                        r["method"], r["rmse_pos"], r["final_pos"], r["rms_resid_pos"], r["max_resid_pos"],
                        r["rms_resid_vel"], r["max_resid_vel"], r["accel_bias"], r["gyro_bias"], r["zupt"],
                        r["grav_mean"], r["grav_max"], r["earth_mean"], r["earth_max"], r["att_err_deg"], r["elapsed"],
                    ])
            print(f"Saved metrics CSV -> {csv_path}")
        except Exception as ex:
            print(f"[WARN] Failed to save metrics CSV: {ex}")

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
