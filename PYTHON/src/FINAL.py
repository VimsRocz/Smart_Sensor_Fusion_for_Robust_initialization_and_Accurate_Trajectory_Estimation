#!/usr/bin/env python3
"""
Run all attitude-initialisation methods for a single IMU/GNSS pair.
The script writes one log file per run in ``./logs`` and prints ``[SUMMARY]``
lines compatible with ``summarise_runs.py``.
"""

import pathlib
import datetime
import sys
import importlib
import io
import contextlib
import argparse
import re
import time

from dependency_bootstrap import ensure_dependencies

ensure_dependencies()

import pandas as pd
import numpy as np
import yaml
import os
import logging

from tabulate import tabulate
from tqdm import tqdm

logging.basicConfig(level=logging.INFO, format="%(message)s")

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

LOG_DIR  = HERE / "logs"
LOG_DIR.mkdir(exist_ok=True)

DEFAULT_DATASETS = [
    ("IMU_X001.dat", "GNSS_X001.csv"),
]

DEFAULT_METHODS  = ["TRIAD", "Davenport", "SVD"]

DATASETS = DEFAULT_DATASETS.copy()
METHODS = DEFAULT_METHODS.copy()

SUMMARY_RE = re.compile(r"\[SUMMARY\]\s+(.*)")

def run_one(imu, gnss, method, verbose=False):
    """Run the fusion script for one IMU/GNSS pair and capture summary output."""
    ts  = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    log = LOG_DIR / f"{pathlib.Path(imu).stem}_{pathlib.Path(gnss).stem}_{method}_{ts}.log"

    fusion = importlib.import_module("GNSS_IMU_Fusion")
    argv = [
        "GNSS_IMU_Fusion.py",
        "--imu-file", str(ROOT / imu),
        "--gnss-file", str(ROOT / gnss),
        "--method", method,
    ]
    summary_lines = []

    with log.open("w") as fh:
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            old_argv = sys.argv
            sys.argv = argv
            try:
                fusion.main()
            finally:
                sys.argv = old_argv
        output = buf.getvalue()
        fh.write(output)
        print(output, end="")
        for line in output.splitlines():
            m = SUMMARY_RE.search(line)
            if m:
                summary_lines.append(m.group(1))
    return summary_lines

def main():
    os.makedirs('results', exist_ok=True)
    logging.info("Ensured 'results/' directory exists.")
    parser = argparse.ArgumentParser()
    parser.add_argument("--verbose", action="store_true", help="Print detailed debug info")
    parser.add_argument("--datasets", default="ALL",
                        help="Comma separated dataset IDs (e.g. X001,X002) or ALL")
    parser.add_argument('--method', choices=['TRIAD','Davenport','SVD','ALL'],
                        default='ALL')
    parser.add_argument('--config', help='YAML configuration file')
    args = parser.parse_args()

    root_files = {p.name for p in ROOT.iterdir()}

    if args.config:
        with open(args.config) as fh:
            cfg = yaml.safe_load(fh) or {}
        global DATASETS, METHODS
        if 'datasets' in cfg:
            DATASETS = [(ds['imu'], ds['gnss']) for ds in cfg['datasets']]
        if 'methods' in cfg:
            METHODS = cfg['methods']

    if args.datasets.upper() == "ALL":
        datasets = DATASETS
    else:
        ids = {d.strip() for d in args.datasets.split(',')}
        datasets = [p for p in DATASETS if pathlib.Path(p[0]).stem.split('_')[1] in ids]

    if args.method.upper() == 'ALL':
        method_list = METHODS
    else:
        method_list = [args.method]
    cases = [(imu, gnss, m) for (imu, gnss) in datasets for m in method_list]
    fusion_results = []

    results_dir = pathlib.Path.cwd() / "results"
    results_dir.mkdir(exist_ok=True)

    for imu, gnss, method in tqdm(cases, desc="All cases"):
        if args.verbose:
            # Debugging information for file pairing and timestamps
            print("==== DEBUG: File Pairing ====")
            print("IMU file:", imu)
            print("GNSS file:", gnss)
            gnss_df = pd.read_csv(gnss)
            imu_data = np.loadtxt(imu)
            print("GNSS shape:", gnss_df.shape)
            print("IMU shape:", imu_data.shape)
            print("GNSS time [start, end]:", gnss_df['Posix_Time'].iloc[0], gnss_df['Posix_Time'].iloc[-1])
            print("IMU time [start, end]:", imu_data[0,0], imu_data[-1,0])
            print("Any NaNs in GNSS?", gnss_df.isna().sum().sum())
            print("Any NaNs in IMU?", np.isnan(imu_data).sum())
            print("GNSS Head:\n", gnss_df.head())
            print("IMU Head:\n", imu_data[:5])
            print("============================")
        if imu not in root_files or gnss not in root_files:
            raise FileNotFoundError(f"Missing {imu} or {gnss} in {ROOT}")
        start = time.time()
        summaries = run_one(imu, gnss, method, verbose=args.verbose)
        runtime = time.time() - start
        for summary in summaries:
            kv = dict(re.findall(r"(\w+)=\s*([^\s]+)", summary))
            fusion_results.append({
                "dataset"  : pathlib.Path(imu).stem.split("_")[1],
                "method"   : kv.get("method", method),
                "rmse_pos" : float(kv.get("rmse_pos", "nan").replace("m", "")),
                "final_pos": float(kv.get("final_pos", "nan").replace("m", "")),
                "rms_resid_pos": float(kv.get("rms_resid_pos", "nan").replace("m", "")),
                "max_resid_pos": float(kv.get("max_resid_pos", "nan").replace("m", "")),
                "rms_resid_vel": float(kv.get("rms_resid_vel", "nan").replace("m", "")),
                "max_resid_vel": float(kv.get("max_resid_vel", "nan").replace("m", "")),
                "accel_bias": float(kv.get("accel_bias", "nan")),
                "gyro_bias": float(kv.get("gyro_bias", "nan")),
                "grav_mean_deg": float(kv.get("GravErrMean_deg", "nan")),
                "grav_max_deg": float(kv.get("GravErrMax_deg", "nan")),
                "earth_mean_deg": float(kv.get("EarthRateErrMean_deg", "nan")),
                "earth_max_deg": float(kv.get("EarthRateErrMax_deg", "nan")),
                "q0_w"      : float(kv.get("q0_w", "nan")),
                "q0_x"      : float(kv.get("q0_x", "nan")),
                "q0_y"      : float(kv.get("q0_y", "nan")),
                "q0_z"      : float(kv.get("q0_z", "nan")),
                "Pxx"       : float(kv.get("Pxx", "nan")),
                "Pyy"       : float(kv.get("Pyy", "nan")),
                "Pzz"       : float(kv.get("Pzz", "nan")),
                "ZUPT_count": int(kv.get("ZUPT_count", "0")),
                "runtime"  : runtime,
            })

    # --- nicely formatted summary table --------------------------------------
    key_order = {m: i for i, m in enumerate(["TRIAD", "Davenport", "SVD"])}
    fusion_results.sort(key=lambda r: (r["dataset"], key_order[r["method"]]))
    rows = [
        [
            e["dataset"],
            e["method"],
            e["rmse_pos"],
            e["final_pos"],
            e["rms_resid_pos"],
            e["max_resid_pos"],
            e["rms_resid_vel"],
            e["max_resid_vel"],
            e["accel_bias"],
            e["gyro_bias"],
            e.get("grav_mean_deg", float('nan')),
            e.get("grav_max_deg", float('nan')),
            e.get("earth_mean_deg", float('nan')),
            e.get("earth_max_deg", float('nan')),
            e.get("q0_w", float('nan')),
            e.get("q0_x", float('nan')),
            e.get("q0_y", float('nan')),
            e.get("q0_z", float('nan')),
            e.get("Pxx", float('nan')),
            e.get("Pyy", float('nan')),
            e.get("Pzz", float('nan')),
            e.get("ZUPT_count", float('nan')),
            e["runtime"],
        ]
        for e in fusion_results
    ]
    print(tabulate(
        rows,
        headers=[
            "Dataset",
            "Method",
            "RMSEpos [m]",
            "End-Error [m]",
            "RMSresidPos [m]",
            "MaxresidPos [m]",
            "RMSresidVel [m/s]",
            "MaxresidVel [m/s]",
            "AccelBiasNorm",
            "GyroBiasNorm",
            "GravErrMean [deg]",
            "GravErrMax [deg]",
            "EarthRateMean [deg]",
            "EarthRateMax [deg]",
            "q0_w",
            "q0_x",
            "q0_y",
            "q0_z",
            "Pxx",
            "Pyy",
            "Pzz",
            "ZUPTcnt",
            "Runtime [s]",
        ],
        floatfmt=".2f",
    ))

    # Optional CSV export for easier analysis
    df = pd.DataFrame(
        rows,
        columns=[
            "Dataset",
            "Method",
            "RMSEpos_m",
            "EndErr_m",
            "RMSresidPos_m",
            "MaxresidPos_m",
            "RMSresidVel_mps",
            "MaxresidVel_mps",
            "AccelBiasNorm",
            "GyroBiasNorm",
            "GravErrMean_deg",
            "GravErrMax_deg",
            "EarthRateErrMean_deg",
            "EarthRateErrMax_deg",
            "q0_w",
            "q0_x",
            "q0_y",
            "q0_z",
            "Pxx",
            "Pyy",
            "Pzz",
            "ZUPT_count",
            "Runtime_s",
        ],
    )
    df.to_csv(results_dir / "summary.csv", index=False)


if __name__ == "__main__":
    main()
