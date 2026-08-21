#!/usr/bin/env python3
"""Run any local fusion script after preparing its Python environment.

Examples
--------
    python3 run.py GNSS_IMU_Fusion.py --imu-file IMU_X001.dat \
        --gnss-file GNSS_X001.csv --method TRIAD
    python3 run.py run_all_datasets.py --method SVD

The child script receives the same interpreter used by this launcher, so a
virtual environment or user-site installation is respected consistently.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

from dependency_bootstrap import ensure_dependencies


HERE = Path(__file__).resolve().parent


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if not args or args[0] in {"-h", "--help"}:
        print(__doc__.strip())
        return 0 if args else 2

    script = Path(args.pop(0)).expanduser()
    if not script.is_absolute():
        candidates = [Path.cwd() / script, HERE / script]
        script = next((candidate for candidate in candidates if candidate.is_file()), candidates[0])
    script = script.resolve()

    if script == Path(__file__).resolve():
        raise SystemExit("run.py cannot launch itself")
    if script.suffix != ".py" or not script.is_file():
        raise SystemExit(f"Python script not found: {script}")

    ensure_dependencies()
    command = [sys.executable, str(script), *args]
    env = os.environ.copy()
    env.setdefault("PYTHONPATH", str(HERE))
    return subprocess.call(command, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
