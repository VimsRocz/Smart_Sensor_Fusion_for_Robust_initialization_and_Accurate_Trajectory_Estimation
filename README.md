# Smart Sensor Fusion for Robust Initialization and Accurate Trajectory Estimation

A reproducible seven-task GNSS/IMU pipeline with matching Python and MATLAB entry points. Initial attitude can be solved with **TRIAD**, **Davenport's Q-method**, or **SVD/Wahba**. Each method can run independently, or all three can run against the same validated inputs and produce a comparison.

The canonical v2 workflow replaces the repository's overlapping experimental runners. Those scripts remain under `PYTHON/src/` for historical compatibility, but new work should use `PYTHON/run_pipeline.py` or `MATLAB/run_pipeline.m`.

## What is reliable in v2

- Tasks 1–7 have explicit inputs, subtasks, outputs, and dependency rules.
- Input files fail early with row, column, timestamp, unit, and finite-value checks.
- The known one-second-resetting IMU clock is repaired and reported.
- TRIAD, Davenport, and SVD can run alone or together.
- Every executed task writes its own JSON plus numeric and plot artifacts where applicable.
- Outputs use one quaternion convention: scalar-first `[w,x,y,z]`, Body→NED, unit norm. Input truth order/frame are declared in configuration and converted.
- Truth and estimated position/velocity use one fixed ECEF origin/NED rotation; attitude truth uses its declared moving local frame.
- Height is always `−NED Down`; it is never inferred from ECEF Z or a quaternion component.
- Truth is optional. Tasks 1–5 still run without it; Task 6 is marked skipped and Task 7 reports GNSS innovation statistics.

## Installation

Python 3.10 or newer:

```bash
python3 -m venv .venv
source .venv/bin/activate                 # Windows: .venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install -e '.[tests]'
```

MATLAB requires a release with `readtable`, `readmatrix`, `tiledlayout`, and `jsonencode` (R2021a or newer is recommended). The canonical MATLAB implementation uses no third-party toolbox.

## Quick start

Validate the bundled files without processing:

```bash
python PYTHON/run_pipeline.py --config config/pipeline_small.yaml --validate-only
```

Run all tasks with all three methods and create comparison artifacts:

```bash
python PYTHON/run_pipeline.py --config config/pipeline_small.yaml
```

Run only TRIAD through Task 5:

```bash
python PYTHON/run_pipeline.py \
  --imu DATA/IMU/IMU_X001_small.dat \
  --gnss DATA/GNSS/GNSS_X001_small.csv \
  --truth DATA/Truth/STATE_X001_small.txt \
  --method TRIAD --tasks 1-5 --output results
```

Run Davenport or SVD by changing `--method`. Use `--method ALL` for a direct comparison. Requesting a downstream task automatically executes its upstream dependencies; for example, `--tasks 5` executes Tasks 1–5 and records Tasks 1–4 as automatic dependencies.

MATLAB equivalents:

```matlab
addpath('MATLAB');

% One method
r = run_pipeline( ...
    'imu', 'DATA/IMU/IMU_X001_small.dat', ...
    'gnss', 'DATA/GNSS/GNSS_X001_small.csv', ...
    'truth', 'DATA/Truth/STATE_X001_small.txt', ...
    'method', 'TRIAD', 'tasks', '1-7');

% All three methods
c = run_all_methods( ...
    'imu', 'DATA/IMU/IMU_X001_small.dat', ...
    'gnss', 'DATA/GNSS/GNSS_X001_small.csv', ...
    'truth', 'DATA/Truth/STATE_X001_small.txt', ...
    'tasks', '1-7');
```

## Python GUI

```bash
python gui.py
```

The GUI selects IMU, GNSS, and optional truth files; validates them before a run; selects one method or all methods; accepts task ranges; streams logs; and lists every generated PNG, JSON, CSV, and NPZ artifact. Double-click an artifact to open it.

## Task and subtask map

| Task | Subtasks | Primary output |
|---|---|---|
| 1 — Inputs/reference | 1.1 validate files; 1.2 derive WGS-84 origin; 1.3 gravity/Earth-rate vectors | `reference.json` |
| 2 — Static IMU | 2.1 convert units; 2.2 select quiet window; 2.3 average body vectors | `body_vectors.json` |
| 3 — Attitude | 3.1 solve method; 3.2 normalize quaternion; 3.3 estimate biases | `initial_attitude.json` |
| 4 — Inertial propagation | 4.1 correct IMU; 4.2 propagate quaternion; 4.3 integrate NED state | `inertial_solution.npz/.mat` |
| 5 — Fusion | 5.1 predict; 5.2 GNSS update; 5.3 save innovations | `fused_solution.npz/.mat` |
| 6 — Truth overlay | 6.1 align time; 6.2 common-frame NED; 6.3 height=`−Down`; 6.4 quaternion alignment | `truth_overlay.npz/.mat` |
| 7 — Evaluation | 7.1 position; 7.2 velocity; 7.3 attitude; 7.4 scalar metrics | `metrics.json` |

See [Tasks and execution model](docs/TASKS_AND_SUBTASKS.md) and [input/output contracts](docs/INPUT_OUTPUT_CONTRACTS.md) for the full schemas.

## Output structure

```text
results/<run-id>/
├── triad/
│   ├── manifest.json
│   ├── task_01_inputs_reference/
│   ├── task_02_static_imu/
│   ├── task_03_attitude/
│   ├── task_04_inertial/
│   ├── task_05_fusion/
│   ├── task_06_truth/
│   └── task_07_evaluation/
├── davenport/
├── svd/
└── comparison/
    ├── method_comparison.json
    ├── method_comparison.csv
    └── method_comparison.png
```

Every method folder is self-contained. `manifest.json` records resolved inputs, configuration, requested tasks, automatic dependencies, timing, status, and Task 7 metrics.

## Configuration

Copy `config/pipeline_small.yaml` and change only the `input`, `run`, or `pipeline` values. Unknown keys and invalid numeric settings are rejected instead of silently ignored. Command-line options override the matching YAML values.

Print the accepted file structures at any time:

```bash
python PYTHON/run_pipeline.py --print-contract
```

## Verification

```bash
pytest -q
make smoke
```

CI now treats validation, tests, and the all-method small-data smoke run as required checks. Generated result files remain ignored by Git.

The full 500,000-sample X001 TRIAD verification run completes in about 39 seconds on the development machine and produced 0.101 m position RMSE, 0.156 m/s velocity RMSE, 0.088 m height RMSE, and 0.189° attitude RMSE. Results depend on the selected data and configuration; every run records its exact values in `manifest.json`.

## Repository layout

```text
PYTHON/fusion_pipeline/   canonical Python package
PYTHON/run_pipeline.py    stable Python CLI
MATLAB/+fusion/           separate MATLAB task functions
MATLAB/run_pipeline.m     stable MATLAB single-method runner
MATLAB/run_all_methods.m  MATLAB all-method comparison
DATA/                     bundled IMU, GNSS, and truth datasets
config/                   versioned run configurations
docs/                     task and data-contract documentation
PYTHON/src/               legacy/experimental scripts
```

The vibration-detection experiment is retained as a separate legacy utility in `run_vibration_detection.py`; it is not part of the canonical GNSS/IMU Tasks 1–7 pipeline.

## License

MIT — see [LICENSE](LICENSE).
