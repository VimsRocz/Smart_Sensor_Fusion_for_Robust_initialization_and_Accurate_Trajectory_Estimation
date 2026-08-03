## Changelog

### 3.2.0 — 2026-07-29

- Added live progress. While a run works it now streams the task it has reached,
  each subtask by number and name, the numeric **result** of that subtask as soon
  as it is computed, and each figure as it is written — with per-task figure
  counts and elapsed time. Controlled by `--progress on|off`.
- Each method opens with a header naming the method and the exact IMU, GNSS and
  truth files with their row counts, rates and durations, so an all-method run
  over several datasets is unambiguous. Every task line repeats the method and
  names the datasets that task consumes; every figure line carries its
  coordinate frame and dataset codes; the closing line names the method and its
  inputs.
- Skipped subtasks and figures state their reason inline instead of only
  appearing in the end-of-run tables.

### 3.1.0 — 2026-07-29

- Replaced the plain CLI output with aligned tables: input datasets (role, name,
  file, size), tasks executed (name, subtasks, figure counts, sub-folder),
  figures grouped by task and subtask (frame, sensor codes, status, save folder),
  the output folder tree, and the Task 7 metrics. An all-method run adds a
  metric-by-method comparison table. New `--report full|summary|none`.
- `make` now resolves the interpreter itself: it uses `.venv/bin/python` when
  present and `python3` otherwise, never a bare `python`/`pip`. This fixes
  `pyenv: python: command not found` on pyenv setups where the global version is
  `system`. Added `make venv`, `make doctor` and `make gui`.
- Corrected every README and doc to use `make`, `python3` or `.venv/bin/python`,
  and added a Quick start and a Troubleshooting section to the root README.

### 3.0.0 — 2026-07-28

- Added `fusion_pipeline/catalog.py` as the single source of truth for task names,
  subtask names, output directories and figures. Directory names, JSON summaries,
  figure filenames and `docs/TASKS_AND_SUBTASKS.md` are all generated from it, and
  a test fails if the document has drifted.
- Expanded plotting from 8 to **38 figures per method run** plus 5 cross-method
  comparison figures, so every one of the 23 subtasks produces at least one figure.
- Figure filenames now carry the run id, method, task number, subtask number, task
  name, figure name, **coordinate frame** and **source sensor datasets**. The same
  metadata is stamped inside each image as a title and footer.
- Added `figures_index.json` / `figures_index.csv` per run, listing every catalog
  figure with its status (`written`, `skipped`, `not_implemented`) and reason.
- Made the input format configurable instead of fixed: IMU/GNSS/truth column
  indices, delimiters, and time/angle/acceleration/length/velocity units are all
  declarable in configuration. Added `config/pipeline_custom_data_template.yaml`.
- `--method` now accepts a single method, a comma-separated subset such as
  `TRIAD,SVD`, or `ALL`.
- Added `--list-tasks`, an expanded `--print-contract`, per-run figure counts in
  the CLI output, and `make help` with one target per run option.
- Added `config/pipeline_x001_full.yaml` and `config/pipeline_x002_no_truth.yaml`.
- Renamed task directories for clarity: `task_03_attitude` → `task_03_attitude_init`,
  `task_04_inertial` → `task_04_inertial_propagation`, `task_05_fusion` →
  `task_05_gnss_imu_fusion`, `task_06_truth` → `task_06_truth_overlay`.
- Manifest schema bumped to `3.0`; it now embeds the task catalog and figure counts.
- MATLAB gained `+fusion/catalog.m` and `+fusion/figures.m` so both implementations
  produce identical directory and figure names.

### 2.0.0 — 2026-07-15

- Added canonical, separate Python and MATLAB Tasks 1–7.
- Added strict and documented IMU, GNSS, and truth input contracts.
- Added independent TRIAD, Davenport, and SVD execution plus all-method comparison.
- Added dependency-aware task prefixes and per-task JSON/numeric/plot outputs.
- Replaced ambiguous quaternion handling with normalized scalar-first Body→NED values.
- Corrected height comparisons to use `height = -NED Down` after common-origin ECEF conversion.
- Rebuilt the Python GUI around the canonical runner.
- Made canonical validation, tests, and smoke execution mandatory in CI.

### Recent Updates

- **Bias estimation fix** (`Task_2` function, `fusion_single.py`)
  - Detects a low-motion segment to compute accelerometer and gyroscope biases.
  - Scales the accelerometer magnitude to match 9.81 m/s² for more stable attitude initialisation.

- **Kalman filter with GNSS updates** (`kalman.py`, `src/GNSS_IMU_Fusion.py`)
  - Adds a bias-aware Kalman filter for fusing IMU data with GNSS measurements.
  - Provides helper functions to run the filter and to tune process noise.
- **Tunable noise parameters** (`kalman.py`, `fusion_single.py`)
  - Position/velocity process and measurement noise can now be set via
    arguments for easier experimentation.

- **Plotting helpers** (`auto_plots.py`, `summarise_runs.py`, `generate_summary.py`)
  - Automates generation of standard figures and summary tables.
  - Useful for batch processing of multiple datasets and visualising results.

These utilities were added to streamline the fusion pipeline and assist with
debugging and analysis.
