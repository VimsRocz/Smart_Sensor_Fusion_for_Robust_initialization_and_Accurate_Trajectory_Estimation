# Python implementation

`main.py` is the canonical entry point. The `fusion_pipeline/` package contains the task catalog, input contracts, attitude methods, Tasks 1–7, figure generation, comparison logic, and the CLI. `run_pipeline.py` remains as a compatibility wrapper.

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
.venv/bin/python PYTHON/main.py --config config/pipeline_small.yaml
.venv/bin/python PYTHON/main.py --method SVD --tasks 1-5 \
  --imu DATA/IMU/IMU_X001_small.dat \
  --gnss DATA/GNSS/GNSS_X001_small.csv
```

`--method` accepts `TRIAD`, `Davenport`, `SVD`, a comma-separated subset such as `TRIAD,SVD`, or `ALL`.

Run one document-numbered task independently; prerequisites are inserted automatically:

```bash
./scripts/run_task.sh 4 --dataset x001 --method TRIAD
./scripts/run_task.sh 5 --dataset x001 --method SVD
./scripts/run_task.sh 7 --dataset x001 --method Davenport
```

Task 4 includes 4.6, Task 5 includes 5.10, and Task 7 includes 7.6. Every
canonical plot has same-stem PNG, PDF and MAT artifacts. Native MATLAB FIG is
also produced when MATLAB is available.

## Module map

| File | Responsibility |
|---|---|
| `fusion_pipeline/catalog.py` | Task, subtask and figure catalog — the single source of truth for names, directories, frames and datasets |
| `fusion_pipeline/contracts.py` | Input readers, configurable column layouts and unit conversion |
| `fusion_pipeline/attitude.py` | TRIAD, Davenport's Q-method and SVD/Wahba solvers |
| `fusion_pipeline/math3d.py` | Frame and quaternion utilities |
| `fusion_pipeline/pipeline.py` | Tasks 1–7 and run orchestration |
| `fusion_pipeline/figures.py` | Figure generation, naming, stamping and indexing |
| `fusion_pipeline/figure_export.py` | PNG/PDF/MAT export and optional native FIG handling |
| `fusion_pipeline/report.py` | Terminal tables: datasets, tasks, subtasks, figures, save locations |
| `fusion_pipeline/cli.py` | Command-line interface |
| `fusion_pipeline/tasks/task_01.py` … `task_07.py` | Independent task entry points |

## GUI and batch files

Run `make gui` or `./scripts/run_gui.sh`. The GUI accepts one or more IMU,
GNSS and optional truth files; supports by-index or all-combination pairing;
selects Tasks 1–7 and any method subset; validates custom layouts; streams run
logs; and previews the generated figures.

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
