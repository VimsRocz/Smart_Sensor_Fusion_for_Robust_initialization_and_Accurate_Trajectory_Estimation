# Smart Sensor Fusion for Robust Initialization and Accurate Trajectory Estimation

A reproducible seven-task GNSS/IMU fusion pipeline with matching Python and MATLAB entry points. The initial attitude can be solved three ways — **TRIAD**, **Davenport's Q-method**, and **SVD/Wahba** — each on its own, or all three together against the same validated inputs with an automatic cross-method comparison.

Every task writes its own folder. Every subtask produces at least one figure. Every figure filename states the run, the method, the task, the subtask, the figure, the **coordinate frame**, and the **sensor files** it was drawn from.

- [**Release Task 1–7 plots**](docs/RELEASE_PLOTS.md) — the standard figure set and how to produce it
- [How to run](#how-to-run) · [Task and subtask map](#task-and-subtask-map) · [Output layout](#output-layout) · [Figure naming](#figure-naming)
- [**Dataset reference**](DATA/README.md) — what X001/X002/X003 are, every column, and what the numbers mean
- [**Requirements compliance**](docs/REQUIREMENTS_COMPLIANCE.md) — this repo validated against the project specification
- [**Project document traceability**](docs/PROJECT_DOCUMENT_TRACEABILITY.md) — PPT/PDF mapping for Tasks 4.6, 5.10 and 7.6
- [**Using your own data**](#using-your-own-data) — what to change if your files are formatted differently
- [Configuration reference](#configuration-reference) · [MATLAB](#matlab) · [GUI](#python-gui)
- [**Troubleshooting**](#troubleshooting-command-not-found) — `command not found: python` / `pip`

---

## Quick start

```bash
make venv        # one-time: creates .venv and installs everything
make doctor      # confirms interpreter, packages and MATLAB executable
make release     # Tasks 1-7 on x001 with TRIAD -> results/
```

`make venv` installs both the canonical pipeline and full release-runner
dependencies. For an already-created environment, update them with:

```bash
.venv/bin/python -m pip install -e '.[tests,release]'
```

> `pyenv: python: command not found`? Expected on pyenv; the `make` targets avoid
> it entirely. See [Troubleshooting](#troubleshooting-command-not-found).

---

## The datasets

Three datasets exist. All are the **same flight** — only the error model differs.

| Name | IMU | GNSS | Truth | What it is |
|---|---|---|---|---|
| `x001` | `IMU_X001.dat` | `GNSS_X001.csv` | `STATE_X001.txt` | Error-free baseline. The only run with a reference trajectory. |
| `x002` | `IMU_X002.dat` | `GNSS_X002.csv` | — | X001 plus zero-mean IMU and GNSS noise. |
| `x003` | `IMU_X003.dat` | `GNSS_X002.csv` | — | X002 plus a constant IMU bias. **Has no GNSS of its own** — paired with `GNSS_X002` by design. |

Full detail — every column, unit, noise and bias figure — in [DATA/README.md](DATA/README.md).

---

## How to run the full release pipeline

The release runner executes the large 500,000-sample Tasks 1–7 script. It
supports the complete cross product:

```text
3 IMUs × 2 GNSS files × 3 attitude methods = 18 combinations
```

Run any one combination with one command:

```bash
make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=SVD
```

Run all 18 sequentially:

```bash
make release-18
```

List the 18 without running, or validate the selected files and their time
coverage first:

```bash
make release-list
make release-check IMU_ID=x003 GNSS_ID=x001 METHOD=SVD
```

`make release` is the short default for X001 IMU + X001 GNSS + TRIAD. The
bundled `STATE_X001.txt` truth is used by default for Tasks 6–7 because every
sensor file represents the same flight. Add `NO_TRUTH=1` to any command to run
without truth comparisons.

### PNG and directly openable MATLAB FIG output

When MATLAB is available, every release plot gets a native same-stem `.fig`
file during the run. The runner calls MATLAB automatically after plotting,
embeds the exact rendered image in a native MATLAB figure, and audits the
result count. You can double-click the `.fig` locally or upload it to MATLAB
and open it directly—no redraw script is required. The same-stem `.mat`
companion contains the numeric plot data.

Native FIG serialization is a MATLAB feature; a `.mat` file renamed to `.fig`
is not valid and will not open. MATLAB must therefore be installed on the
machine that generates the results. If it is not on `PATH`, specify it once:

```bash
make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=SVD \
  MATLAB_BIN=/Applications/MATLAB_R2026a.app/bin/matlab
```

To add/audit FIG companions for PNGs already in `results/` without rerunning
the fusion filters:

```bash
make release-figs MATLAB_BIN=/path/to/matlab
```

Release commands automatically use deferred mode, so they compute successfully
on a machine without MATLAB instead of failing at preflight. In that case PNG,
PDF and MAT files are written while the script runs, and only native FIG
creation is deferred. The explicit two-stage aliases remain available:

```bash
make release-compute IMU_ID=x001 GNSS_ID=x001 METHOD=TRIAD
# or all combinations (requires more than 20 GiB free):
make release-18-compute OUTPUT_DIR=/path/on/a/larger/disk
```

When MATLAB is unavailable, the output directory includes
`export_release_figures.m` and `CREATE_NATIVE_FIGS.txt`. Copy that complete
directory to any system with MATLAB and run `export_release_figures(pwd)` there
once. This creates every native FIG without repeating IMU/GNSS computation.
The resulting FIG files are portable and open directly later with a
double-click or `openfig`.

### All 18 possibilities

Every row below is a valid one-line run. Values are case-insensitive.

| # | IMU | GNSS | Method | Command |
|---:|---|---|---|---|
| 1 | X001 | X001 | TRIAD | `make release-combo IMU_ID=x001 GNSS_ID=x001 METHOD=TRIAD` |
| 2 | X001 | X001 | Davenport | `make release-combo IMU_ID=x001 GNSS_ID=x001 METHOD=Davenport` |
| 3 | X001 | X001 | SVD | `make release-combo IMU_ID=x001 GNSS_ID=x001 METHOD=SVD` |
| 4 | X001 | X002 | TRIAD | `make release-combo IMU_ID=x001 GNSS_ID=x002 METHOD=TRIAD` |
| 5 | X001 | X002 | Davenport | `make release-combo IMU_ID=x001 GNSS_ID=x002 METHOD=Davenport` |
| 6 | X001 | X002 | SVD | `make release-combo IMU_ID=x001 GNSS_ID=x002 METHOD=SVD` |
| 7 | X002 | X001 | TRIAD | `make release-combo IMU_ID=x002 GNSS_ID=x001 METHOD=TRIAD` |
| 8 | X002 | X001 | Davenport | `make release-combo IMU_ID=x002 GNSS_ID=x001 METHOD=Davenport` |
| 9 | X002 | X001 | SVD | `make release-combo IMU_ID=x002 GNSS_ID=x001 METHOD=SVD` |
| 10 | X002 | X002 | TRIAD | `make release-combo IMU_ID=x002 GNSS_ID=x002 METHOD=TRIAD` |
| 11 | X002 | X002 | Davenport | `make release-combo IMU_ID=x002 GNSS_ID=x002 METHOD=Davenport` |
| 12 | X002 | X002 | SVD | `make release-combo IMU_ID=x002 GNSS_ID=x002 METHOD=SVD` |
| 13 | X003 | X001 | TRIAD | `make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=TRIAD` |
| 14 | X003 | X001 | Davenport | `make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=Davenport` |
| 15 | X003 | X001 | SVD | `make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=SVD` |
| 16 | X003 | X002 | TRIAD | `make release-combo IMU_ID=x003 GNSS_ID=x002 METHOD=TRIAD` |
| 17 | X003 | X002 | Davenport | `make release-combo IMU_ID=x003 GNSS_ID=x002 METHOD=Davenport` |
| 18 | X003 | X002 | SVD | `make release-combo IMU_ID=x003 GNSS_ID=x002 METHOD=SVD` |

The intended delivered pairings remain X001+X001, X002+X002 and X003+X002.
Cross-pairings are deliberate diagnostic experiments: for example,
X002+X001 isolates IMU noise, while X001+X002 isolates GNSS noise.

### Synchronization: same coverage, not the same row count

IMU and GNSS files must describe the same time window, but they should **not**
have the same number of rows when their rates differ. The bundled files contain:

| Sensor | Rows | Rate | Coverage |
|---|---:|---:|---:|
| IMU | 500,000 | 400 Hz | 1,250 s |
| GNSS | 1,250 | 1 Hz | 1,250 measurement epochs |

Before any task starts, `PYTHON/run_release.py` validates both layouts, infers
their rates, and rejects a pair if their coverage differs by more than one GNSS
epoch. For the bundled data it reports approximately 400 IMU samples per GNSS
epoch. Truth must extend through the common sensor window.

### Custom fixed-format release files

Use the same large Tasks 1–7 release runner on replacement files with:

```bash
make release-custom \
  IMU_FILE=/path/to/IMU_NEW.dat \
  GNSS_FILE=/path/to/GNSS_NEW.csv \
  TRUTH_FILE=/path/to/STATE_NEW.txt \
  METHOD=TRIAD
```

The release script requires the following fixed formats:

| File | Required format |
|---|---|
| IMU `.dat` | Whitespace-separated, no header, at least 8 numeric columns: counter, time in seconds, gyro x/y/z delta-angle in rad, accelerometer x/y/z delta-velocity in m/s. Optional temperature/status columns may follow. PPS time resets are accepted. |
| GNSS `.csv` | Header must contain `Posix_Time,X_ECEF_m,Y_ECEF_m,Z_ECEF_m,VX_ECEF_mps,VY_ECEF_mps,VZ_ECEF_mps`. Time is seconds and strictly increasing; position is true ECEF metres; velocity is ECEF m/s. Extra columns are allowed. |
| Truth `.txt` | Whitespace-separated, comments may start with `#`, 12 numeric columns: counter, time s, ECEF position x/y/z m, ECEF velocity x/y/z m/s, quaternion `[qx,qy,qz,qw]` Body→ECEF. Time strictly increases and must cover the sensors. |

If a new file uses different columns, delimiters, units, quaternion order or
frames, use the configurable contract-driven pipeline described under
[Using your own data](#using-your-own-data) instead of feeding it directly to
the fixed-format release script.

Output is tagged by the actual selection, for example
`SVD_IMU_X003_GNSS_X001_task6_1_fused_vs_truth_ned.png`, so combinations do
not overwrite one another.

### Where the plots go

Flat in `results/`, named
`<METHOD>_<IMU>_<GNSS>_task<N>_<subtask>_<name>.png` (and `.pdf`), e.g.
`TRIAD_IMU_X001_GNSS_X001_task5_8_4_fused_state_NED.png`.
Every plot carries its subtask number — full list in
[docs/RELEASE_PLOTS.md](docs/RELEASE_PLOTS.md).

Full runs produce large `.mat`/`.npz` intermediates. All 18 can require more
than 20 GiB, so use `OUTPUT_DIR=/path/on/a/larger/disk` when necessary. To
reclaim space while
keeping every plot:

```bash
make clean-heavy
```

---

## Canonical configurable Task 1–7 pipeline

`PYTHON/main.py` and `PYTHON/fusion_pipeline/` are the maintained implementation
for configurable inputs, independent tasks, GUI execution and project-document
figures. Each figure is exported as PNG, PDF and MAT; native FIG is added when
MATLAB is available.

```bash
.venv/bin/python PYTHON/main.py --dataset x002 --method TRIAD
.venv/bin/python PYTHON/main.py --list-datasets
.venv/bin/python PYTHON/main.py --list-tasks
```

### Choosing which tasks to run

Tasks form a strict prefix chain, `1 → 2 → 3 → 4 → 5 → 6 → 7`. Asking for a downstream task automatically runs its prerequisites and records them as `auto_dependencies` in the manifest, so a Task 5 result can never consume stale Task 1–3 files.

```bash
.venv/bin/python PYTHON/main.py --config config/pipeline_small.yaml --method TRIAD --tasks 1-5
.venv/bin/python PYTHON/main.py --config config/pipeline_small.yaml --method SVD   --tasks 3
./scripts/run_task.sh 4 --config config/pipeline_small.yaml --method Davenport
```

### Useful flags

| Flag | Effect |
|---|---|
| `--dataset NAME` | Use a bundled dataset's verified IMU/GNSS/truth pairing |
| `--list-datasets` | Print every bundled dataset and its pairing, then exit |
| `--no-truth` | Run without a reference trajectory even if one is configured |
| `--progress on` | **Default.** Stream each task, subtask, result and figure while the run works |
| `--progress off` | Silent until the run finishes |
| `--report full` | **Default.** Tables of datasets, tasks, subtasks, figures and save locations |
| `--report summary` | Task-level tables only — no per-figure listing |
| `--report none` | One line per method |
| `--list-tasks` | Print every task, subtask and figure with its frame and datasets, then exit |
| `--print-contract` | Print the accepted input file layouts and the config key for each, then exit |
| `--validate-only` | Parse and check the inputs, print a summary, run nothing |
| `--no-plots` | Numeric artifacts only — much faster |
| `--run-id NAME` | Fix the output folder name instead of deriving it from the filenames |
| `--output DIR` | Change the results root (default `results`) |

### While a run works

Progress streams live, so a long run never goes quiet.

Each method starts with a header naming **the method and the exact input files**, so an all-method run over several datasets is never ambiguous:

```text
════════════════════════════════════════════════════════════════════════════════════════════════════
 RUNNING  method TRIAD   ·   run id X001_small   ·   tasks 1-7 (7 of 7)
════════════════════════════════════════════════════════════════════════════════════════════════════
   IMU    IMU_X001_small           DATA/IMU/IMU_X001_small.dat        1000 rows @ 400.0 Hz, 2.50 s
   GNSS   GNSS_X001_small          DATA/GNSS/GNSS_X001_small.csv      9 epochs, 8.00 s
   Truth  STATE_X001_small         DATA/Truth/STATE_X001_small.txt    99 states, 9.80 s, with attitude
   Output results/X001_small/triad/
   Codes  I = IMU_X001_small · G = GNSS_X001_small · T = STATE_X001_small
```

Then every task states **the method, which datasets that task consumes, and where it writes**; every subtask prints its **result** as soon as it is computed; and every figure is listed as it is written, with its coordinate frame and the datasets behind it:

```text
▶ TRIAD · Task 3/7 — Initial attitude and IMU biases
    data: IMU_X001_small + GNSS_X001_small    →  task_03_attitude_init/
    3.1  Solve the Body-to-NED alignment with TRIAD, Davenport or SVD
         TRIAD: gravity error 0° · Earth-rate error 0.003365°
    3.2  Project onto SO(3) and normalise the scalar-first quaternion
         q_wxyz = [0.785282, -0.012399, 0.618810, 0.015936], |q| = 1.000000000000
    3.3  Estimate accelerometer and gyroscope bias in the solved attitude
         |accel bias| 0.00115 m/s² · |gyro bias| 4.305e-09 rad/s
         ✎ 3.1  rotation_matrix                    [BODY2NED]  I+G
         ✎ 3.1  vector_alignment_error             [BODY2NED]  I+G
         ✎ 3.2  initial_quaternion                 [BODY2NED]  I+G
         ✎ 3.2  initial_euler_angles               [BODY2NED]  I+G
         ✎ 3.3  imu_bias_estimates                 [BODY]      I
  ✔ TRIAD · Task 3 complete — 5 figures, 0.38 s
```

The `I` / `G` / `T` codes map to the dataset names printed in the header. `✎` marks a figure that was written and `·` one that was skipped, with the reason given on the subtask line. Each method closes with a line naming itself and its inputs:

```text
─── TRIAD on IMU_X001_small + GNSS_X001_small finished in 8.2 s — 51 figures ───
```

Turn it all off with `--progress off`.

The result reported per subtask:

| Subtask | Live result |
|---|---|
| 1.1 / 1.2 / 1.3 | row counts and rates · origin lat/lon/alt · gravity and Earth rate |
| 2.1 / 2.2 / 2.3 | sample count and units · chosen static window and its variance · averaged vector magnitudes |
| 3.1 / 3.2 / 3.3 | per-vector alignment error · the quaternion and its norm · bias magnitudes |
| 4.1 / 4.2 / 4.3 / 4.6 | samples interpolated past limits · samples propagated and final \|q\| · final IMU-only position · three-frame GNSS/IMU comparison |
| 5.1 / 5.2 / 5.3 / 5.10 | prediction steps · GNSS updates and final innovation · final fused position · three-frame final state |
| 6.1 / 6.2 / 6.3 / 6.4 | overlapping samples and time offset · truth NED extent · truth height range · quaternion alignment |
| 7.1 / 7.2 / 7.3 / 7.4 / 7.6 | position RMSE and max · velocity RMSE and max · attitude RMSE · metric count · three-frame truth and attitude comparison |

### What a run prints at the end

Every run then reports, as aligned tables:

1. **Input datasets** — role, dataset name, file path, and size (rows, rate, duration).
2. **Tasks executed** — task number, task name, its subtask numbers, figure counts, and the sub-folder each task wrote to.
3. **Figures by task and subtask** — for every task: the subtask names, each figure, its coordinate frame, which sensors it used (`I`=IMU, `G`=GNSS, `T`=Truth), and whether it was written or skipped. The full save folder is printed above each table.
4. **Output structure** — the folder tree the run created, with per-task figure counts.
5. **Task 7 metrics** — every scalar metric.

An all-method run adds a **cross-method comparison**: the run folder per method, a metric-by-method table, and the comparison figure list.

```text
  Task 6 — Truth overlay in a common frame
  Saved in: results/X001_small/triad/task_06_truth_overlay/
    ┌─────────┬──────────────────────────────────────────────┬──────────────────────────────────┬───────────┬─────────┬──────────┐
    │ Subtask │ Subtask name                                 │ Figure                           │ Frame     │ Data    │ Status   │
    ├─────────┼──────────────────────────────────────────────┼──────────────────────────────────┼───────────┼─────────┼──────────┤
    │ 6.1     │ Restrict to the common time range and apply… │ truth_time_alignment             │ NONE      │ I+T     │ written  │
    │ 6.2     │ Convert truth ECEF position/velocity with t… │ fused_vs_truth_position_velocity │ NED       │ I+G+T   │ written  │
    │         │                                              │ fused_vs_truth_ground_track      │ NED       │ I+G+T   │ written  │
    │ 6.3     │ Compare relative height defined as negative… │ height_above_origin              │ NED       │ I+G+T   │ written  │
    │ 6.4     │ Normalise and hemisphere-align the Body-to-… │ quaternion_comparison            │ BODY2NED  │ I+T     │ written  │
    │         │                                              │ euler_angle_comparison           │ BODY2NED  │ I+T     │ written  │
    └─────────┴──────────────────────────────────────────────┴──────────────────────────────────┴───────────┴─────────┴──────────┘
```

The exact filename of every figure is in `figures_index.csv` in the run folder.

---

## Task and subtask map

Seven tasks, twenty-six subtasks. `PYTHON/fusion_pipeline/catalog.py` is the single source of truth: it drives the directory names, the JSON summaries, the figure filenames, and [docs/TASKS_AND_SUBTASKS.md](docs/TASKS_AND_SUBTASKS.md), which is generated from it.

| Task | Directory | Subtasks | Figures |
|---|---|---|---|
| **1** — Inputs and navigation reference | `task_01_inputs_reference/` | 1.1 validate inputs · 1.2 derive WGS-84 origin · 1.3 gravity and Earth-rate vectors | 5 |
| **2** — Static interval and body vectors | `task_02_static_imu/` | 2.1 convert increments to SI rates · 2.2 select minimum-variance window · 2.3 average and normalise body vectors | 4 |
| **3** — Initial attitude and IMU biases | `task_03_attitude_init/` | 3.1 solve Body→NED alignment · 3.2 normalise quaternion · 3.3 estimate biases | 5 |
| **4** — IMU-only strapdown propagation | `task_04_inertial_propagation/` | 4.1 screen outliers and remove bias · 4.2 propagate quaternion · 4.3 integration · **4.6 GNSS/IMU NED/ECEF/Body comparison** | 10 |
| **5** — GNSS/IMU Kalman fusion | `task_05_gnss_imu_fusion/` | 5.1 predict · 5.2 GNSS update · 5.3 export · **5.10 fused NED/ECEF/Body state** | 8 |
| **6** — Truth overlay in a common frame | `task_06_truth_overlay/` | 6.1 align time · 6.2 convert truth to common NED · 6.3 height = −Down · 6.4 quaternion alignment | 6 |
| **7** — Residual evaluation and metrics | `task_07_evaluation/` | 7.1–7.4 residuals/metrics · **7.6 truth overlays and detailed attitude errors** | 13 |
| — Cross-method comparison | `comparison/` | C.1–C.5 | 5 |

**51 figures per method run**, plus 5 comparison figures. An all-method run therefore produces 158 figures.

Task 3 is the only algorithmic branch between the three methods; Tasks 4–7 consume that method-specific attitude, which is why each method gets a self-contained folder.

To see the full map including every figure title, frame and dataset:

```bash
make list-tasks
```

---

## Output layout

```text
results/<run-id>/
├── triad/
│   ├── manifest.json              resolved inputs, full config, timing, metrics, task catalog
│   ├── figures_index.json         every figure with task, subtask, frame, datasets, status
│   ├── figures_index.csv          the same index as a spreadsheet
│   ├── task_01_inputs_reference/      reference.json  + 5 PNGs
│   ├── task_02_static_imu/            body_vectors.json + 4 PNGs
│   ├── task_03_attitude_init/         initial_attitude.json + 5 PNGs
│   ├── task_04_inertial_propagation/  summary.json, inertial_solution.npz + 7 PNGs
│   ├── task_05_gnss_imu_fusion/       summary.json, fused_solution.npz + 5 PNGs
│   ├── task_06_truth_overlay/         summary.json, truth_overlay.npz + 6 PNGs
│   └── task_07_evaluation/            metrics.json, residuals.npz + 6 PNGs
├── davenport/                     same structure
├── svd/                           same structure
└── comparison/
    ├── method_comparison.json
    ├── method_comparison.csv
    ├── figures_index.json / .csv
    └── 5 comparison PNGs
```

Each method folder is self-contained. `manifest.json` records the schema version, resolved input paths, every numeric configuration value, requested tasks, inserted dependencies, status, timings, figure counts, the embedded task catalog, and the Task 7 metrics.

---

## Figure naming

```text
<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png
```

For example:

```text
X001_full_TRIAD_task06_sub6.3_truth_overlay_height_above_origin_frame-NED_data-IMU_X001+GNSS_X001+STATE_X001.png
└─run─┘ └meth┘ └task┘ └sub─┘ └──task name──┘ └───figure name───┘ └frame┘ └────sensor data files────┘
```

Every figure also carries the same information *inside* the image: a two-line title with the task, subtask and figure name, and a footer giving the coordinate frame with its full meaning, the source datasets, the run id and the method. A figure dropped into a report or slide stays unambiguous on its own.

### Coordinate frame tags

| Tag | Meaning |
|---|---|
| `ECEF` | Earth-Centred Earth-Fixed (WGS-84), metres |
| `GEODETIC` | Geodetic WGS-84 latitude / longitude / altitude |
| `BODY` | IMU body frame (x-forward, y-right, z-down) |
| `NED` | Local-level North-East-Down at the Task 1 origin |
| `BODY2NED` | Body-to-NED rotation (quaternion / Euler / DCM) |
| `BODYvsNED` | Measured body vectors against their NED references |
| `NONE` | Frame-independent quantity |

### Finding a figure

`figures_index.csv` in each run folder lists every figure with its task, task name, subtask, subtask name, figure title, coordinate frame, sensor datasets and status. Sort or filter it in any spreadsheet, or:

```bash
grep ',6.3,' results/X001_full/triad/figures_index.csv
```

Figures that could not be produced — for example the truth comparisons when no reference file was supplied — appear in the index with `status = skipped` and a reason, rather than being silently absent.

---

## Using your own data

**You do not need to rewrite or re-export your files.** Every column index, delimiter and unit is configurable. Copy the annotated template and change only what differs:

```bash
cp config/pipeline_custom_data_template.yaml config/my_data.yaml
# edit config/my_data.yaml
.venv/bin/python PYTHON/main.py --config config/my_data.yaml --validate-only
.venv/bin/python PYTHON/main.py --config config/my_data.yaml
```

Always run `--validate-only` first. It reports the parsed row counts, sample rate, duration, repaired clock resets and the GNSS column names it resolved, without processing anything.

### What the pipeline expects by default

**All column indices are zero-based.**

| File | Default layout | Default units |
|---|---|---|
| IMU | col 0 counter, col 1 time, cols 2–4 gyro, cols 5–7 accel; whitespace-separated | time s; delta-angle rad; delta-velocity m/s |
| GNSS | CSV with header `Posix_Time, X_ECEF_m, Y_ECEF_m, Z_ECEF_m, VX_ECEF_mps, VY_ECEF_mps, VZ_ECEF_mps` | time s; position m (true ECEF); velocity m/s |
| Truth *(optional)* | col 0 counter, col 1 time, cols 2–4 ECEF position, cols 5–7 ECEF velocity, cols 8–11 quaternion | time s; position m; velocity m/s; quaternion `[qx,qy,qz,qw]` Body→ECEF |

For release plots, the output is `<stem>.png` for universal viewing,
`<stem>.pdf` for publication, and `<stem>.mat` for underlying numeric arrays.
When MATLAB is available, `<stem>.fig` is also written during the run; without
MATLAB it is created later by the bundled conversion command.

### If your data differs — change these keys

| Your situation | Set this in the `pipeline:` section |
|---|---|
| IMU columns in a different order or with no counter column | `imu_time_column`, `imu_gyro_columns`, `imu_accel_columns` |
| IMU already stores rad/s and m/s² instead of per-sample increments | `imu_measurement_type: rate` |
| IMU gyro in degrees | `imu_gyro_unit: deg` |
| IMU accelerometer in g or milli-g | `imu_accel_unit: g` or `mg` |
| IMU timestamps in milliseconds/microseconds | `imu_time_unit: ms` (or `us`, `ns`) |
| IMU file is comma- or tab-separated | `imu_delimiter: ","` (or `"\t"`) |
| GNSS headers are named differently | `gnss_column_overrides: {time: ..., x: ..., y: ..., z: ..., vx: ..., vy: ..., vz: ...}` |
| GNSS file is semicolon-separated | `gnss_delimiter: ";"` |
| GNSS position in km, velocity in km/h or knots | `gnss_position_unit: km`, `gnss_velocity_unit: kmph` / `kn` |
| Truth columns in a different order | `truth_time_column`, `truth_position_columns`, `truth_velocity_columns` |
| Truth has **no** attitude | `truth_quaternion_columns: null` — Tasks 6 and 7 then compare position and velocity only |
| Truth quaternion is scalar-first | `truth_quaternion_order: wxyz` |
| Truth quaternion is already Body→NED | `truth_quaternion_frame: body_to_ned` |
| Truth attitude is stamped at the interval midpoint | `truth_attitude_time_offset_s` (the bundled 10 Hz truth needs `-0.05`) |
| Different sampling rate | `static_samples` — should span a genuinely stationary stretch, roughly 1–10 s of samples |

Some GNSS header spellings are recognised without any configuration: `Posix_Time` / `POSIX_Time` / `time` / `Time` / `timestamp`; `X_ECEF_m` / `X_ECEF` / `ecef_x_m` / `x`; and the matching `y`, `z`, `vx`, `vy`, `vz` forms.

### What cannot be configured away

- **GNSS position must be true ECEF metres.** The loader rejects any position whose norm is below 6,000 km, because a local or geodetic column silently interpreted as ECEF would corrupt every downstream frame. Convert to ECEF first.
- **Time must strictly increase** in the GNSS and truth files. The one known exception — the IMU clock that resets every second — is detected, repaired, and reported as `clock_wraps_repaired`.
- **All used values must be finite.** A NaN or Inf is reported with its row and column instead of being interpolated away.

Print the whole contract, including the config key for each option:

```bash
make contract
```

Full detail: [docs/INPUT_OUTPUT_CONTRACTS.md](docs/INPUT_OUTPUT_CONTRACTS.md).

---

## Configuration reference

A configuration file has three sections. Command-line options override the matching YAML value. Unknown keys are rejected with the list of accepted keys rather than being silently ignored.

```yaml
input:                 # which files to read
  imu: DATA/IMU/IMU_X001.dat
  gnss: DATA/GNSS/GNSS_X001.csv
  truth: DATA/Truth/STATE_X001.txt     # optional

run:                   # what to run and where to put it
  method: ALL          # TRIAD | Davenport | SVD | TRIAD,SVD | ALL
  tasks: 1-7
  output: results
  run_id: X001_full

pipeline:              # input layout, algorithm tuning, output settings
  ...
```

Shipped configurations:

| File | Purpose |
|---|---|
| `config/pipeline_small.yaml` | 1,000-sample X001 extract — runs in seconds |
| `config/pipeline_x001_full.yaml` | Full X001 dataset with reference trajectory |
| `config/pipeline_x002_no_truth.yaml` | Full X002 with no reference — shows the truth-optional path |
| `config/pipeline_x003_no_truth.yaml` | Full X003 (`IMU_X003` + `GNSS_X002`), no reference, one method |
| `config/pipeline_custom_data_template.yaml` | Annotated template for your own data |

### Key algorithm settings

| Key | Default | Meaning |
|---|---|---|
| `static_samples` | 400 | Length of the static-alignment window, in samples |
| `gravity_weight` / `earth_rate_weight` | 0.9999 / 0.0001 | Relative trust in the two Wahba observations |
| `gravity_override_mps2` | `null` | Replace WGS-84 normal gravity with a calibrated local value |
| `process_accel_std_mps2` | 20.0 | Filter process noise — raise it to trust GNSS more |
| `gnss_position_std_m` / `gnss_velocity_std_mps` | 2.0 / 0.5 | GNSS measurement noise |
| `max_specific_force_mps2` / `max_angular_rate_rps` | 100.0 / 20.0 | Dropout limits; exceeding samples are interpolated and counted |
| `max_plot_points` | 50000 | Per-trace decimation cap for figures |
| `plots` | `true` | Set false (or pass `--no-plots`) for numeric artifacts only |

---

## What the pipeline guarantees

- Input files fail early with the file, the violated rule, and usually the row/column context.
- One quaternion convention throughout the outputs: scalar-first `[w,x,y,z]`, Body→NED, unit norm. The input truth order and frame are declared in configuration and converted.
- Truth and estimated position/velocity share one fixed ECEF origin and NED rotation; truth attitude uses its declared time-varying local frame.
- Height is always `−NED Down`. It is never inferred from ECEF Z or from a quaternion component.
- Truth is optional. Tasks 1–5 run unchanged, Task 6 records `status: skipped` with a reason instead of fabricating a reference, and Task 7 reports GNSS innovation statistics.
- JSON summaries are written atomically; large arrays go to compressed `.npz` (Python) and `.mat` (MATLAB).
- The task catalog is embedded in every manifest, so a result folder documents its own structure.

---

## MATLAB

```matlab
addpath('MATLAB');

% One method
r = run_pipeline( ...
    'imu',   'DATA/IMU/IMU_X001_small.dat', ...
    'gnss',  'DATA/GNSS/GNSS_X001_small.csv', ...
    'truth', 'DATA/Truth/STATE_X001_small.txt', ...
    'method','TRIAD', 'tasks','1-7');

% All three methods and the comparison
c = run_all_methods( ...
    'imu',   'DATA/IMU/IMU_X001_small.dat', ...
    'gnss',  'DATA/GNSS/GNSS_X001_small.csv', ...
    'truth', 'DATA/Truth/STATE_X001_small.txt', ...
    'tasks', '1-7');
```

The MATLAB implementation mirrors the Python task structure, directory names and figure-naming convention. See [MATLAB/README.md](MATLAB/README.md) for the per-task file map and the current parity status.

---

## Python GUI

```bash
make gui
```

Or directly: `.venv/bin/python gui.py`.

Select one or more IMU, GNSS and optional truth files; choose by-index or all-combination pairing; run TRIAD, Davenport, SVD, a subset or all methods; and select any Tasks 1–7. The GUI validates configurable input layouts, streams batch logs, previews PNG plots, and lists PNG, PDF, FIG, MAT, JSON, CSV and NPZ artifacts.

---

## Verification

```bash
make test
make smoke
```

The test suite covers the three attitude solvers, the input contracts, custom column and unit layouts, method selection, the task/figure catalog invariants, figure naming, and an end-to-end run of all seven tasks with and without truth. `docs/TASKS_AND_SUBTASKS.md` is generated from the catalog by `scripts/generate_task_docs.py`, and a test fails if it has drifted:

```bash
make docs
```

CI treats validation, lint, tests and the all-method smoke run as required checks. Generated result files stay ignored by Git.

---

## Troubleshooting: `command not found`

### `pyenv: python: command not found` or `pyenv: pip: command not found`

```text
pyenv: python: command not found
The `python' command exists in these Python versions:
  3.12.1
  3.12.1/envs/.venv_matlab
```

This means pyenv's active version is `system`, and macOS provides no bare `python` or `pip`. Nothing is broken — you just need to name an interpreter that exists.

**Fix (recommended) — let `make` handle it:**

```bash
make venv
make doctor
make run-all
```

`make` uses `.venv/bin/python` directly, so pyenv never enters the picture.

**Fix (manual) — activate the virtualenv:**

```bash
source .venv/bin/activate
python PYTHON/main.py --config config/pipeline_small.yaml
```

Inside an activated venv, plain `python` and `pip` work.

**Fix (pyenv) — give pyenv a real default:**

```bash
pyenv global 3.12.1
```

### Rules that avoid this class of error

| Instead of | Use |
|---|---|
| `python script.py` | `python3 script.py`, or `.venv/bin/python script.py` |
| `pip install X` | `python3 -m pip install X` |
| `pip install -r requirements.txt` | `make venv` |

`python3 -m pip` always works when `python3` does, because it does not depend on a separate `pip` shim existing.

### `ModuleNotFoundError: No module named 'numpy'` (or `matplotlib`, `yaml`)

You are running a Python that has no dependencies installed. Check which one:

```bash
make doctor
```

If it does not report `.venv/bin/python`, run `make venv`.

### `ModuleNotFoundError: No module named 'fusion_pipeline'`

Run the pipeline through `PYTHON/main.py`, which runs from the package directory. Importing `fusion_pipeline` elsewhere needs the editable install from `make venv`.

### `No module named 'scipy'` / `filterpy` in a release run

The configurable canonical pipeline does not need these packages, but the full
release plot implementation does. Install the release extra:

```bash
.venv/bin/python -m pip install -e '.[release]'
```

`make venv` performs this installation automatically.

### The GUI does not open

```bash
make gui
```

If that reports a missing `tkinter`, your Python was built without Tk. On macOS, `brew install python-tk@3.12` provides it. The GUI is optional — every feature is available from the CLI.

---

## Repository layout

```text
PYTHON/fusion_pipeline/catalog.py     task, subtask and figure catalog (source of truth)
PYTHON/fusion_pipeline/contracts.py   input readers, layouts and unit handling
PYTHON/fusion_pipeline/attitude.py    TRIAD, Davenport and SVD solvers
PYTHON/fusion_pipeline/pipeline.py    Tasks 1-7 and the run orchestration
PYTHON/fusion_pipeline/figures.py     figure generation, naming and indexing
PYTHON/fusion_pipeline/cli.py         command-line interface
PYTHON/main.py                        canonical Python entry point
PYTHON/fusion_pipeline/tasks/         independent Task 1–7 module entry points
PYTHON/run_pipeline.py                compatibility wrapper
PYTHON/run_release.py                 all 18 full-rate release combinations
MATLAB/+fusion/                       separate MATLAB task functions
MATLAB/run_pipeline.m                 MATLAB single-method runner
MATLAB/run_all_methods.m              MATLAB all-method comparison
DATA/                                 bundled IMU, GNSS and truth datasets
DATA/README.md                        dataset reference: runs, columns, units, noise and bias
config/                               versioned run configurations
docs/                                 task and data-contract documentation
scripts/generate_task_docs.py         regenerates the task documentation
scripts/run_gui.sh                    GUI launcher
scripts/run_pipeline.sh               shell pipeline launcher
scripts/run_task.sh                   independent task launcher
PYTHON/src/                           legacy/experimental scripts
```

The vibration-detection experiment in `run_vibration_detection.py` is a separate legacy utility and is not part of the canonical Tasks 1–7 pipeline.

## License

MIT — see [LICENSE](LICENSE).
