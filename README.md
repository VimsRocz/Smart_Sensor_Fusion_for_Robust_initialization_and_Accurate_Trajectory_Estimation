# Vibration Detection and Rejection from IMU Data

> **Project Focus**: This repository contains the solution for the **Vibration Detection and Rejection from IMU Data** project.
>
> For complete project documentation, see [VIBRATION_PROJECT_README.md](VIBRATION_PROJECT_README.md)

## Quick Start

Run the complete vibration detection and compensation pipeline:

```bash
python run_vibration_detection.py
```

This will generate comprehensive results and plots in the `vibration_results/` directory.

## Project Overview

This project implements detection and compensation algorithms for vibration artifacts in IMU data, supporting:
- Multiple vibration types (sinusoidal, motor, rotor, random)
- Frequency domain analysis and detection
- Adaptive compensation methods
- Real-time capable processing

See [VIBRATION_PROJECT_README.md](VIBRATION_PROJECT_README.md) for complete documentation including API reference, usage examples, and results.

---

# Additional Repository Content

This repository was originally forked from a larger project hub. Below is the documentation for the original content:

## IMU — GNSS + IMU Data Processing (Python)

[![GitHub release](https://img.shields.io/github/v/release/VimsRocz/IMU?logo=github)](https://github.com/VimsRocz/IMU/releases) [![GitHub stars](https://img.shields.io/github/stars/VimsRocz/IMU?style=social)](https://github.com/VimsRocz/IMU/stargazers) [![GitHub forks](https://img.shields.io/github/forks/VimsRocz/IMU?style=social)](https://github.com/VimsRocz/IMU/network/members) [![Follow on GitHub](https://img.shields.io/github/followers/VimsRocz?style=social)](https://github.com/VimsRocz) [![License](https://img.shields.io/github/license/VimsRocz/IMU)](LICENSE) [![Python CI](https://github.com/VimsRocz/IMU/actions/workflows/python-ci.yml/badge.svg)](https://github.com/VimsRocz/IMU/actions/workflows/python-ci.yml) [![PyPI](https://img.shields.io/pypi/v/imu_gnss_fusion)](https://pypi.org/project/imu_gnss_fusion/) [![CodeQL](https://github.com/VimsRocz/IMU/actions/workflows/codeql.yml/badge.svg)](https://github.com/VimsRocz/IMU/actions/workflows/codeql.yml)

### Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Quick Start](#quick-start)
- [Running the Pipeline](#running-the-pipeline)
  - [run_all_methods.py](#run_all_methodspy)
  - [run_all_datasets.py](#run_all_datasetspy)
  - [run_triad_only.py](#run_triad_onlypy)
  - [run_method_only.py](#run_method_onlypy)
  - [GNSS_IMU_Fusion_single](#gnss_imu_fusion_singleimu_file-gnss_file)
  - [FINAL.py](#finalpy)
- [Per-Task Overview](#per-task-overview)
- [Datasets](#datasets)
- [Running validation](#running-validation)
- [Aligning Datasets with DTW](#aligning-datasets-with-dtw)
- [Sample Processing Report](#sample-processing-report)
- [Task 6: State Overlay](#task-6-state-overlay)
- [Task 7: Evaluation of Filter Results](#task-7-evaluation-of-filter-results)
- [Verifying Earth Rotation Rate](#verifying-earth-rotation-rate)
- [Automated figure generation](#automated-figure-generation)
- [Summary CSV format](#summary-csv-format)
- [Tests](#tests)
- [Linting](#linting)
- [Static Analysis with CodeQL](#static-analysis-with-codeql)
- [Additional scripts](#additional-scripts)
- [Next Steps](#next-steps)
- [Appendix: Advanced Topics](#appendix-advanced-topics)
- [Documentation](#documentation)
- [Repository Layout](#repository-layout)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
 - [Quaternion Convention](#quaternion-convention)

### Installation

`PYTHON/src/run_all_datasets.py` ensures dependencies are installed the first time you run it (via `utils.ensure_dependencies`). To set them up beforehand, install from the repository root:

```bash
pip install -r requirements.txt
```

For a minimal setup you can also install the core packages individually:

```bash
pip install numpy matplotlib scipy filterpy
```

If the `filterpy` wheel fails to build, force a source install:

```bash
pip install filterpy --no-binary :all:
```

The tests require all packages from `requirements.txt` together with the optional test extras defined in the root `pyproject.toml`. These pull in heavier libraries such as `cartopy`. Installing them in a virtual environment keeps your base Python setup clean. Running `make test` installs the optional extras automatically before executing `pytest`.

If you run into issues with `filterpy` on Ubuntu:

```bash
pip install --upgrade pip setuptools
pip install filterpy
pip install filterpy --no-binary :all:
```

#### Installing FilterPy on Ubuntu

If installation of `filterpy` fails (for example due to missing build tools) run:

```bash
sudo apt update && sudo apt install python3-pip build-essential python3-dev
pip3 install filterpy --no-binary :all:
```

#### Installing test requirements

To run the unit tests install the package in editable mode with the `tests` extras (repository root):

```bash
pip install -e .[tests]
```

This pulls in `pytest` together with the runtime libraries listed under `[project.optional-dependencies] tests` in `pyproject.toml` — notably `numpy`, `scipy`, `matplotlib`, `pandas`, `filterpy`, and `cartopy`.

#### Offline installation (optional)

If you have internet access, simply install dependencies from PyPI with `pip install -r requirements.txt`.

If your CI environment cannot reach PyPI, you can populate a local `vendor/` directory with all required packages by running:

```bash
scripts/download_vendor.sh
```

Install from that folder with:

```bash
pip install --no-index --find-links vendor -r requirements.txt
```

If the build error complains about Cython, install it explicitly first:

```bash
pip3 install cython
pip3 install filterpy
```



```matlab
ver('signal')
```

If the command prints an entry for the toolbox, it is available for use.

### Quick Start

Repository Structure (Simple View)

- `DATA/IMU`: raw IMU logs (IMU_X00{1,2,3}.dat, + `_small`)
- `DATA/GNSS`: raw GNSS logs (GNSS_X00{1,2}.csv, + `_small`)
- `DATA/Truth`: reference trajectory (STATE_X001.txt, STATE_X001_small.txt)
- `PYTHON/src`: Python helpers and library code
- `PYTHON`: top-level Python runner scripts and utilities
- `PYTHON/results`: Python outputs

Run — Python (from repo root)

```bash
python PYTHON/src/run_triad_only.py   # uses DATA/... and writes to PYTHON/results/
```


```matlab
run_triad_only(struct('dataset_id','X002','method','TRIAD'));
```

Task 1 outputs are cached for reuse:

- Python Task 1 saves an interactive HTML map under `PYTHON/results/`.

Notes

- Older scripts assumed data in the repo root; all data now lives under `DATA/`.

### Quaternion Convention

- Storage: all saved quaternions are scalar-first `[w, x, y, z]`, Body→NED.
- SciPy interop: SciPy `Rotation` expects `[x, y, z, w]`. Use helpers in `PYTHON/src/utils/quaternion_tools.py`:
  - `to_xyzw(q_wxyz)` before `Rotation.from_quat(...)`
  - `to_wxyz(q_xyzw)` for results from `Rotation.as_quat()`
- Sign hemisphere: align estimate to reference with `align_sign_to_ref(q_est, q_ref)` before comparison/plotting.
- Continuity (no ref): use `ensure_continuity(q_series)`.

### Usage


```matlab
run_all_datasets_matlab;                      % all methods
% or
run_all_datasets_matlab('TRIAD');
```


### Per-Task Overview


| Task/Subtask | Input | Output | Link |
|--------------|-------|--------|------|
| Task 1 – Reference Vectors | First GNSS fix | Gravity and Earth rotation reference vectors | [docs/TRIAD_Task1_Wiki.md](docs/TRIAD_Task1_Wiki.md) |
| Task 2 – Body Vectors | Raw IMU measurements near initial time | Measured gravity and Earth-rate in the IMU frame plus estimated IMU biases | [docs/TRIAD_Task2_Wiki.md](docs/TRIAD_Task2_Wiki.md) |
| Task 3 – Initial Attitude | Reference vectors from Task 1 and body vectors from Task 2 | Initial attitude using the TRIAD algorithm | [docs/TRIAD_Task3_Wiki.md](docs/TRIAD_Task3_Wiki.md) |
| Task 4 – IMU-Only Integration | Corrected IMU data and initial attitude | Integrated trajectory compared with GNSS solution | [docs/TRIAD_Task4_Wiki.md](docs/TRIAD_Task4_Wiki.md) |
| Task 5 – Kalman Fusion | IMU-only trajectory and GNSS updates | Fused trajectory from a simple Kalman filter | [docs/TRIAD_Task5_Wiki.md](docs/TRIAD_Task5_Wiki.md) |
| Task 6 – Truth Overlay | Task 5 results and recorded reference trajectory | Figures with truth overlaid on fused trajectory | [docs/TRIAD_Task6_Wiki.md](docs/TRIAD_Task6_Wiki.md) |

### Datasets

The repository includes three IMU logs and two GNSS traces:

- `DATA/IMU/IMU_X001.dat` with `DATA/GNSS/GNSS_X001.csv` – baseline run without additional errors.
- `DATA/IMU/IMU_X002.dat` with `DATA/GNSS/GNSS_X002.csv` – same trajectory with extra measurement noise.
- `DATA/IMU/IMU_X003.dat` with `DATA/GNSS/GNSS_X002.csv` – reuses the X002 GNSS log but adds a constant bias to the IMU.

The GNSS CSV files record only Earth-centred Earth-fixed (ECEF) coordinates and
velocities; the latitude, longitude and altitude columns are left at zero.  Any
geodetic coordinates used by the pipeline are derived from the ECEF values.

For quick tests the repository also provides truncated versions of each file:

- `DATA/IMU/IMU_X001_small.dat`, `DATA/IMU/IMU_X002_small.dat`, `DATA/IMU/IMU_X003_small.dat` contain the first 1,000 IMU samples
- `DATA/GNSS/GNSS_X001_small.csv`, `DATA/GNSS/GNSS_X002_small.csv` contain the first ten GNSS epochs
- `DATA/Truth/STATE_X001_small.txt` holds the first 100 reference states

These mini logs drastically reduce runtimes when validating the pipeline.

### Running validation

Run the following commands from the repository root so the helper scripts can locate the bundled data and truth file automatically.

First process all datasets with every initialization method:

```bash
python PYTHON/src/run_all_methods.py
```

Once the `*_kf_output.mat` files are created, validate them with:

```bash
python PYTHON/validate_all_outputs.py
```

The validator searches the `PYTHON/results/` folder for saved outputs, compares them to the common `DATA/Truth/STATE_X001.txt` ground truth and writes overlay figures and a CSV summary alongside the logs.

If your logs or truth file live elsewhere, provide the paths explicitly:

```bash
python PYTHON/validate_all_outputs.py --results-dir path/to/logs \
    --truth-dir path/to/truth
```

To validate an individual estimator output manually call `PYTHON/src/validate_with_truth.py` with the filter result file and the reference trajectory:

```bash
python PYTHON/src/validate_with_truth.py --est-file <kf.mat> \
    --truth-file DATA/Truth/STATE_X001.txt --output PYTHON/results
```

For a quick look at ±3σ bounds stored in the covariance matrix run:

```bash
python PYTHON/src/validate_3sigma.py --est-file <kf.npz> --truth-file DATA/Truth/STATE_X001.txt \
    --output-dir PYTHON/results
```

If the estimator output and truth file do not overlap, the script prints a detailed error showing both time ranges. A constant offset can be applied to the truth timestamps via the optional `--time-shift` argument when alignment is required. Passing `--auto-time-shift` will automatically apply the offset `est_start - truth_start` when no manual shift is provided.

### Aligning Datasets with DTW

The helper script `PYTHON/align_datasets_dtw.py` aligns an estimator output with the reference trajectory using Dynamic Time Warping.

```bash
pip install fastdtw
python PYTHON/align_datasets_dtw.py \
    --est-file PYTHON/results/my_kf_output.npz \
    --truth-file DATA/Truth/STATE_X001.txt \
    --ref-lat 51.0 --ref-lon -0.1
```

It prints the mean and standard deviation of the position error and writes the per‑sample errors to `PYTHON/aligned_position_errors.txt`.

Both filenames should include a dataset identifier such as `IMU_X001` or `STATE_X001`. The script checks that these match before loading the files and aborts with an error if they differ.

### Sample Processing Report

A sample run of `PYTHON/src/run_triad_only.py` is documented in `Report/index.md`. Each page lists the equations and the figures generated in the `PYTHON/results/` directory.

Typical result PDFs:

- `<tag>_task1_location_map.pdf` – initial location map
- `task3_errors_comparison.pdf` – attitude initialization error comparison
- `task3_quaternions_comparison.pdf` – quaternion components for initialization
- `task4_comparison_ned.png` – Derived GNSS vs Derived IMU in NED frame
- `task4_mixed_frames.png` – GNSS/IMU data in mixed frames
- `task4_all_ned.pdf` – integrated data in NED frame
- `task4_all_ecef.pdf` – integrated data in ECEF frame
- `task4_all_body.pdf` – integrated data in body frame
- `task5_results_<method>.pdf` – Kalman filter results for each method
- `task5_all_ned.pdf` – Kalman filter results in NED frame
- `task5_all_ecef.pdf` – Kalman filter results in ECEF frame
- `task5_all_body.pdf` – Kalman filter results in body frame
- `<method>_residuals.pdf` – position and velocity residuals
- `<tag>_task6_attitude_angles.pdf` – attitude angles over time
- `<tag>_<frame>_overlay_truth.pdf` – fused output vs reference (e.g. `IMU_X002_GNSS_X002_Davenport_task6_ECEF_overlay_state.pdf`)
- `<tag>_task7_3_residuals_position_velocity.pdf` – Task 7 position/velocity residuals
- `task7_4_attitude_angles_euler.pdf` – Task 7 Euler angle plots
- `task7_fused_vs_truth_error.pdf` – Task 7 fused minus truth velocity error
- `<tag>_task7_5_diff_truth_fused_over_time_<frame>.pdf` – Task 7 truth minus fused difference (NED/ECEF/Body)

### Task 6: State Overlay

This step recreates the Task 5 plots but overlays the fused trajectory with the raw `STATE_X` data.

How to run:

```bash
python PYTHON/src/task6_plot_truth.py --est-file PYTHON/results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat \
    --output PYTHON/results
```

The script attempts to locate `DATA/Truth/STATE_X001.txt` automatically based on the dataset identifier in `--est-file`. Pass `--truth-file` only when the file lives outside the repository root or has a different name.

Passing `--show-measurements` adds the raw IMU and GNSS curves. Figures are written as `PYTHON/results/<tag>_task6_overlay_state_<frame>.(pdf|png)`.

### Task 7: Evaluation of Filter Results


Includes:

- Position and velocity residuals (GNSS − filter prediction)
- Roll, pitch, yaw plots derived from quaternion
- Optional acceleration error statistics when truth data is supplied
- Truth minus fused difference plots (Subtask 7.5)

How to run:

```bash
python PYTHON/src/run_all_methods.py --task 7
```

Output examples (under `PYTHON/results/`):

- `<tag>_task7_3_residuals_position_velocity.pdf`
- `<tag>_task7_4_attitude_angles_euler.pdf`
- `task7_fused_vs_truth_error.pdf`
- `<tag>_task7_5_diff_truth_fused_over_time_<frame>.pdf`

Notes:

`PYTHON/src/GNSS_IMU_Fusion.py` detects a low-motion interval in the IMU data to estimate initial accelerometer and gyroscope biases, and applies a simple gravity scale factor so the measured gravity is ~9.81 m/s². It also accepts an optional `--truth-file` to overlay truth curves.

### Verifying Earth Rotation Rate

Run any dataset with the `--verbose` flag to print the Earth rotation magnitude measured from static gyroscope data. It should be close to `7.29e-5` rad/s. Compute the nominal value with:

```bash
python - <<"EOF"
import numpy as np; print(2*np.pi/86400)
EOF
```

### Running the Pipeline

#### run_all_methods.py

Run all methods (TRIAD, Davenport, SVD) for one or more datasets. Provide a YAML configuration or rely on defaults (`X001`–`X003`):

```bash
python PYTHON/src/run_all_methods.py --config PYTHON/config/your_config.yml
```

Task 6 (truth overlay) and Task 7 (evaluation) run automatically. All figures are written to `PYTHON/results/`. Naming utilities live in `PYTHON/src/naming.py`.

#### run_all_datasets.py

Process every IMU/GNSS pair with selected methods:

```bash
python PYTHON/src/run_all_datasets.py
```

Use `--method` to restrict to a single method (e.g. TRIAD). A summary table prints at the end and `summary.csv` is written to `PYTHON/results/`.

#### run_triad_only.py

Run only the TRIAD initialization across datasets:

```bash
python PYTHON/src/run_triad_only.py
```


```matlab
run_triad_only('TRIAD')
```

#### run_method_only.py

Select a single method via CLI:

```bash
python PYTHON/src/run_method_only.py --method SVD
```


```matlab
run_method_only('SVD')
```

Wrappers:

- Python: `PYTHON/src/run_triad_only.py`, `PYTHON/src/run_svd_only.py`, `PYTHON/src/run_davenport_only.py`


#### GNSS_IMU_Fusion_single(imu_file, gnss_file)


```matlab
GNSS_IMU_Fusion_single('IMU_X001.dat', 'GNSS_X001.csv')
```

#### FINAL.py

Run the first IMU/GNSS pair with all methods:

```bash
python PYTHON/src/FINAL.py
```

### Automated figure generation

Helpers to export standard plots and metrics:

```python
from PYTHON.src.auto_plots import run_batch

datasets = [("IMU_X001.dat", "GNSS_X001.csv")]
methods = ["TRIAD"]
run_batch(datasets, methods)
```

Figures are saved to `PYTHON/results/`. Interactive exploration lives in `notebooks/demo.ipynb`. See `docs/PlottingExamples.md` and `docs/PlottingChecklist.md`.

### Summary CSV format

`PYTHON/src/summarise_runs.py` parses logs and writes `PYTHON/results/summary.csv`. Columns include: `method`, `imu`, `gnss`, `rmse_pos`, and `final_pos`. `PYTHON/src/generate_summary.py` builds a PNG summary image from this CSV.

### Tests

Run the unit tests with `pytest`. Installing the required Python packages is mandatory. Use a virtual environment.

```bash
make test           # installs extras and runs pytest
# or
./scripts/setup_tests.sh && pytest -q
```

### Linting

Static checks run with Ruff:

```bash
ruff check PYTHON/src PYTHON/tests src tests
```

E402 is ignored in `pyproject.toml` as some modules rely on runtime imports.

### Static Analysis with CodeQL

GitHub CodeQL analyzes Python and Actions code. See `.github/workflows/codeql.yml`.

Example queries:

```ql
// Detect usage of Python eval()
from CallExpr call
where call.getCallee().(NameExpr).getId() = "eval"
select call, "Avoid using eval() — it's a security risk."
```

```ql
// Detect deprecated numpy functions (e.g., matrix)
from CallExpr call
where call.getCallee().(NameExpr).getId() = "matrix"
select call, "Consider using numpy.array() instead of numpy.matrix, which is deprecated."
```

### Additional scripts

Helper utilities not part of the regular pipeline:

- `PYTHON/src/fusion_single.py` – early stand‑alone prototype of the IMU–GNSS fusion routine; exposes bias estimation and Kalman logic.
- `PYTHON/src/validate_filter.py` – compare a saved filter state against GNSS measurements and plot residuals.
- `PYTHON/src/validate_with_truth.py` – compare a filter output directly with `DATA/Truth/STATE_X001.txt` and generate overlays.
- `PYTHON/src/validate_3sigma.py` – plot errors with ±3σ bounds from the covariance matrix.
- `PYTHON/align_and_validate_dtw_task6_task7.py` – combined alignment and validation workflow.

These utilities are optional and not exercised by the unit tests.

### Next Steps

- Logging: Consider adding the `rich` console handler for colorful status messages.
- Documentation: Generate API docs with Sphinx or MkDocs as desired.
- CI: See `.github/workflows/python-ci.yml` for the Python workflow.
- Security: See `docs/disable-default-codeql.md`.

### Appendix: Advanced Topics


### Documentation

- [docs/TRIAD_Task1_Wiki.md](docs/TRIAD_Task1_Wiki.md) – defining reference vectors
- [docs/TRIAD_Task2_Wiki.md](docs/TRIAD_Task2_Wiki.md) – measuring body-frame vectors
- [docs/TRIAD_Task3_Wiki.md](docs/TRIAD_Task3_Wiki.md) – initial attitude determination
- [docs/TRIAD_Task4_Wiki.md](docs/TRIAD_Task4_Wiki.md) – GNSS/IMU integration
- [docs/TRIAD_Task5_Wiki.md](docs/TRIAD_Task5_Wiki.md) – Kalman filter fusion
- [docs/TRIAD_Task6_Wiki.md](docs/TRIAD_Task6_Wiki.md) – truth overlay plots
- [docs/Python/Task7_Python.md](docs/Python/Task7_Python.md) – evaluation pipeline (Python)
- [Report/](Report/) – summary of a typical run
- [docs/CHANGELOG.md](docs/CHANGELOG.md) – list of recent features

## Repository Layout

```
IMU/
├── DATA/                 # shared inputs (IMU, GNSS, Truth)
├── PYTHON/               # Python environment
│   ├── src/              # code
│   ├── results/          # generated artifacts
│   ├── config/           # YAML/JSON configs
│   └── docs/             # (empty placeholder; see top-level docs/)
    ├── src/              # utils
    ├── results/          # generated artifacts
    ├── config/           # configs (if any)
```

## Troubleshooting

- Imports: All entry points auto-handle `sys.path` so no `PYTHONPATH` is needed.
- Data: Defaults point to `DATA/IMU/IMU_<ID>.dat`, `DATA/GNSS/GNSS_<ID>.csv`, and `DATA/Truth/STATE_X001.txt`.

### Contributing

Work on a feature branch (e.g., `TASK_1`), push, and open a PR into main. Do not commit generated outputs outside the `results/` folders.

MIT License
