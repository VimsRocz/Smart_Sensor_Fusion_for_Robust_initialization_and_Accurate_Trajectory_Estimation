# Python implementation

`run_pipeline.py` is the stable entry point. The `fusion_pipeline/` package contains the task catalog, input contracts, attitude methods, Tasks 1–7, figure generation, comparison logic, and the CLI.

## Setup

From the repository root:

```bash
make venv
make doctor
```

`make venv` creates `.venv/` and installs the package into it. Every `make` target then uses `.venv/bin/python` automatically, so you never need to activate anything.

> Use `python3`, never a bare `python` or `pip`. With pyenv set to `system`, or on a stock macOS, those commands do not exist and you get `command not found`. See [Troubleshooting](../README.md#troubleshooting-command-not-found) in the root README.

## Running

```bash
# via make (recommended - no activation needed)
make run-all
make run-triad
make list-tasks

# or directly
.venv/bin/python PYTHON/run_pipeline.py --config config/pipeline_small.yaml
.venv/bin/python PYTHON/run_pipeline.py --method SVD --tasks 1-5 \
  --imu DATA/IMU/IMU_X001_small.dat \
  --gnss DATA/GNSS/GNSS_X001_small.csv
```

`--method` accepts `TRIAD`, `Davenport`, `SVD`, a comma-separated subset such as `TRIAD,SVD`, or `ALL`.

## Module map

| File | Responsibility |
|---|---|
| `fusion_pipeline/catalog.py` | Task, subtask and figure catalog — the single source of truth for names, directories, frames and datasets |
| `fusion_pipeline/contracts.py` | Input readers, configurable column layouts and unit conversion |
| `fusion_pipeline/attitude.py` | TRIAD, Davenport's Q-method and SVD/Wahba solvers |
| `fusion_pipeline/math3d.py` | Frame and quaternion utilities |
| `fusion_pipeline/pipeline.py` | Tasks 1–7 and run orchestration |
| `fusion_pipeline/figures.py` | Figure generation, naming, stamping and indexing |
| `fusion_pipeline/report.py` | Terminal tables: datasets, tasks, subtasks, figures, save locations |
| `fusion_pipeline/cli.py` | Command-line interface |

Control the terminal output with `--report full` (default), `--report summary` or `--report none`.

## Tests

```bash
make test
```

`pytest` is scoped to `PYTHON/tests/pipeline` by `pyproject.toml`. The legacy scripts under `src/` are not part of Tasks 1–7 and are not covered; they need optional dependencies:

```bash
.venv/bin/python -m pip install -e '.[legacy]'
```

See the repository [README](../README.md), the [task and subtask map](../docs/TASKS_AND_SUBTASKS.md), and the [input/output contracts](../docs/INPUT_OUTPUT_CONTRACTS.md).
