#!/usr/bin/env python3
"""Minimal entry point: process one dataset from a YAML file.

Usage
-----
python src/process_one.py --config config/single_run.yml [--verbose]

The YAML must define:

imu: DATA/IMU/IMU_X123.dat
gnss: DATA/GNSS/GNSS_X123.csv
method: TRIAD      # or Davenport/SVD (optional, default TRIAD)
init:              # initial position (geodetic)
  lat_deg: 47.3977
  lon_deg: 8.5456
  alt_m: 488.0

Writes plots and data to PYTHON/results/ and echoes the key outputs paths.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from dependency_bootstrap import ensure_dependencies

ensure_dependencies()

import yaml


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/single_run.yml", help="YAML config path")
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args()

    cfg_path = Path(args.config)
    if not cfg_path.exists():
        print(f"Config not found: {cfg_path}")
        return 2

    with cfg_path.open() as fh:
        cfg = yaml.safe_load(fh) or {}

    imu = cfg.get("imu")
    gnss = cfg.get("gnss")
    method = cfg.get("method", "TRIAD")
    init = cfg.get("init", {}) or {}

    if not imu or not gnss:
        print("Config must define 'imu' and 'gnss' paths.")
        return 2

    here = Path(__file__).resolve().parent
    script = here / "GNSS_IMU_Fusion.py"
    cmd = [
        sys.executable,
        str(script),
        "--imu-file", str(imu),
        "--gnss-file", str(gnss),
        "--method", str(method),
    ]

    if "lat_deg" in init:
        cmd += ["--init-lat-deg", str(init["lat_deg"])]
    if "lon_deg" in init:
        cmd += ["--init-lon-deg", str(init["lon_deg"])]
    if "alt_m" in init:
        cmd += ["--init-alt-m", str(init["alt_m"])]
    if args.verbose:
        cmd.append("--verbose")

    print("Executing:", " ".join(str(c) for c in cmd))
    ret = subprocess.call(cmd)
    if ret != 0:
        print(f"Fusion failed with exit code {ret}")
        return ret

    # Echo key results filenames (constructed in GNSS_IMU_Fusion)
    imu_stem = Path(str(imu)).stem
    gnss_stem = Path(str(gnss)).stem
    tag = f"{imu_stem}_{gnss_stem}_{method}"
    out_dir = Path("results")
    print("\nOutputs:")
    print("- Summary MAT:     ", out_dir / f"{tag}_kf_output.mat")
    print("- Summary NPZ:     ", out_dir / f"{tag}_kf_output.npz")
    print("- Plots (PNG/FIG): ", out_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
