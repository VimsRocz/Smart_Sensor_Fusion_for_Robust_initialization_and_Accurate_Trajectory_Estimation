# Tasks, subtasks, and execution model

<!-- GENERATED FILE. Edit PYTHON/fusion_pipeline/catalog.py, then run:
     .venv/bin/python scripts/generate_task_docs.py -->

Every task, subtask, output directory and figure below is generated from
`PYTHON/fusion_pipeline/catalog.py`, which is also what the pipeline itself
reads at run time. Print the same information at any time with:

```bash
.venv/bin/python PYTHON/main.py --list-tasks
```

## Dependency rule

Tasks form a strict prefix: `1 → 2 → 3 → 4 → 5 → 6 → 7`. Requesting a
downstream task automatically runs and saves every prerequisite. `--tasks 4`
means "produce a valid Task 4 result", so Tasks 1–3 are included and recorded
as `auto_dependencies` in the manifest.

This makes each run reproducible and prevents a Task 5 result from silently
consuming stale Task 1–3 files. To change any task, edit the versioned YAML
configuration or pass CLI values and rerun the desired prefix.

Task 3 is the only algorithmic branch between TRIAD, Davenport and SVD. Every
downstream result is therefore method-specific and lives in its own folder.

## Figure naming

```text
<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png
```

Each run also writes `figures_index.csv` and `figures_index.json` listing every
figure with its task, subtask, coordinate frame and source datasets.

### Coordinate frame tags

| Tag | Meaning |
|---|---|
| `ECEF` | Earth-Centred Earth-Fixed (WGS-84), metres |
| `GEODETIC` | Geodetic WGS-84 latitude/longitude/altitude |
| `BODY` | IMU body frame (x-forward, y-right, z-down) |
| `NED` | Local-level North-East-Down at the Task 1 origin |
| `BODY2NED` | Body-to-NED rotation (quaternion / Euler / DCM) |
| `BODYvsNED` | Measured body vectors against their NED references |
| `NONE` | Frame-independent quantity |

## Task 1 — Inputs and navigation reference

Validate every input file against the documented contract and build the WGS-84 reference used by all later tasks.

Output directory: `task_01_inputs_reference/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **1.1** | Validate IMU, GNSS and truth structure, units and timing | `input_time_coverage` — Input time coverage and sample rates (NONE · imu+gnss+truth)<br>`gnss_raw_position_velocity` — Raw GNSS position and velocity as supplied (ECEF · gnss)<br>`imu_raw_rates` — IMU angular rate and specific force after unit handling (BODY · imu) |
| **1.2** | Derive the WGS-84 origin and ECEF to NED rotation from the first GNSS fix | `reference_origin_map` — Reference origin on the WGS-84 graticule (GEODETIC · gnss) |
| **1.3** | Compute local normal gravity and the Earth-rotation vector in NED | `gravity_earth_rate_vectors` — Reference gravity and Earth-rate vectors (NED · gnss) |

## Task 2 — Static interval and measured body vectors

Find the quietest stretch of IMU data and average it into the two body vectors that Wahba's problem needs.

Output directory: `task_02_static_imu/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **2.1** | Convert delta-angle/delta-velocity increments into SI rates | `converted_rates` — IMU angular rate and specific force used by Task 2 (BODY · imu) |
| **2.2** | Select the minimum-variance static window | `static_window_variance_scan` — Rolling-variance detector across the whole log (BODY · imu)<br>`static_window_selection` — Selected static window on the IMU record (BODY · imu) |
| **2.3** | Average and normalise the specific-force and Earth-rate body vectors | `mean_body_vs_reference_vectors` — Averaged body vectors against their NED references (BODYvsNED · imu+gnss) |

## Task 3 — Initial attitude and IMU biases

Solve Wahba's problem with the selected method and turn the resulting attitude into accelerometer and gyroscope bias estimates.

Output directory: `task_03_attitude_init/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **3.1** | Solve the Body-to-NED alignment with TRIAD, Davenport or SVD | `rotation_matrix` — Solved Body-to-NED direction cosine matrix (BODY2NED · imu+gnss)<br>`vector_alignment_error` — Residual alignment error per observed vector (BODY2NED · imu+gnss) |
| **3.2** | Project onto SO(3) and normalise the scalar-first quaternion | `initial_quaternion` — Initial quaternion components [w,x,y,z] (BODY2NED · imu+gnss)<br>`initial_euler_angles` — Initial yaw, pitch and roll (BODY2NED · imu+gnss) |
| **3.3** | Estimate accelerometer and gyroscope bias in the solved attitude | `imu_bias_estimates` — Estimated accelerometer and gyroscope bias (BODY · imu) |

## Task 4 — IMU-only strapdown propagation

Integrate the bias-corrected IMU alone so the inertial solution can be judged before GNSS is allowed to correct it.

Output directory: `task_04_inertial_propagation/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **4.1** | Screen physical-range outliers and remove the Task 3 biases | `range_screening` — Specific-force and angular-rate magnitudes against their limits (BODY · imu)<br>`bias_corrected_imu` — IMU signals before and after bias removal (BODY · imu) |
| **4.2** | Propagate the Body-to-NED quaternion with Earth and transport rates | `propagated_quaternion` — Propagated Body-to-NED quaternion history (BODY2NED · imu)<br>`propagated_euler_angles` — Propagated yaw, pitch and roll history (BODY2NED · imu) |
| **4.3** | Apply Coriolis compensation and integrate the NED state | `imu_only_position_velocity` — IMU-only position and velocity (NED · imu)<br>`imu_only_acceleration` — IMU-only resolved acceleration (NED · imu)<br>`imu_only_ground_track` — IMU-only horizontal ground track (NED · imu) |
| **4.6** | Compare GNSS-derived and IMU-derived position, velocity, and acceleration in NED, ECEF, and Body frames | `gnss_vs_imu_ned` — GNSS-derived versus IMU-derived kinematics (NED · imu+gnss)<br>`gnss_vs_imu_ecef` — GNSS-derived versus IMU-derived kinematics (ECEF · imu+gnss)<br>`gnss_vs_imu_body` — GNSS-derived versus IMU-derived kinematics (BODY · imu+gnss) |

## Task 5 — GNSS/IMU Kalman fusion

Run the six-state position/velocity filter that corrects the inertial solution at every GNSS epoch.

Output directory: `task_05_gnss_imu_fusion/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **5.1** | Predict the six-state NED position/velocity from IMU acceleration | `prediction_vs_gnss` — Filter prediction against the GNSS measurements (NED · imu+gnss) |
| **5.2** | Apply Joseph-form updates at each asynchronous GNSS epoch | `kalman_innovations` — Position and velocity innovations at each update (NED · gnss) |
| **5.3** | Export the fused state and the innovation history | `fused_position_velocity` — Fused position and velocity (NED · imu+gnss)<br>`fused_ground_track` — Fused horizontal ground track with GNSS fixes (NED · imu+gnss)<br>`fused_vs_imu_only` — Fused solution against the IMU-only solution (NED · imu+gnss) |
| **5.10** | Present the final fused position, velocity, and acceleration in NED, ECEF, and Body frames | `fused_state_ned` — Final fused position, velocity, and acceleration (NED · imu+gnss)<br>`fused_state_ecef` — Final fused position, velocity, and acceleration (ECEF · imu+gnss)<br>`fused_state_body` — Final fused position, velocity, and acceleration (BODY · imu+gnss) |

## Task 6 — Truth overlay in a common frame

Put truth and estimate in one frame, one time base, and one quaternion convention before anything is compared.

Output directory: `task_06_truth_overlay/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **6.1** | Restrict to the common time range and apply the attitude-only time offset | `truth_time_alignment` — Truth and estimate time coverage after alignment (NONE · imu+truth) *[needs truth]* |
| **6.2** | Convert truth ECEF position/velocity with the Task 1 origin and rotation | `fused_vs_truth_position_velocity` — Fused and truth position and velocity (NED · imu+gnss+truth) *[needs truth]*<br>`fused_vs_truth_ground_track` — Fused and truth horizontal ground track (NED · imu+gnss+truth) *[needs truth]* |
| **6.3** | Compare relative height defined as negative NED Down | `height_above_origin` — Relative height, defined as negative NED Down (NED · imu+gnss+truth) *[needs truth]* |
| **6.4** | Normalise and hemisphere-align the Body-to-NED quaternions | `quaternion_comparison` — Fused and truth quaternion components (BODY2NED · imu+truth) *[needs truth]*<br>`euler_angle_comparison` — Fused and truth yaw, pitch and roll (BODY2NED · imu+truth) *[needs truth]* |

## Task 7 — Residual evaluation and quality metrics

Reduce the Task 6 overlay to residual time histories and the scalar metrics used to rank the three methods.

Output directory: `task_07_evaluation/`

| Subtask | What it does | Figures (frame · data) |
|---|---|---|
| **7.1** | Compute NED position residuals | `position_residuals` — Fused minus truth position residuals (NED · imu+gnss+truth) *[needs truth]*<br>`position_error_distribution` — Position error norm distribution (NED · imu+gnss+truth) *[needs truth]* |
| **7.2** | Compute NED velocity residuals | `velocity_residuals` — Fused minus truth velocity residuals (NED · imu+gnss+truth) *[needs truth]* |
| **7.3** | Compute the sign-invariant quaternion geodesic error | `attitude_error` — Quaternion geodesic attitude error (BODY2NED · imu+truth) *[needs truth]* |
| **7.4** | Export the scalar comparison metrics | `metric_summary` — Scalar accuracy metrics for this method (NONE · imu+gnss+truth)<br>`innovation_summary` — GNSS innovation statistics used when truth is absent (NED · gnss) |
| **7.6** | Compare fused and truth states in NED, ECEF, and Body frames and report detailed attitude errors | `fused_vs_truth_ned` — Fused and truth position and velocity (NED · imu+gnss+truth) *[needs truth]*<br>`fused_vs_truth_ecef` — Fused and truth position and velocity (ECEF · imu+gnss+truth) *[needs truth]*<br>`fused_vs_truth_body` — Fused and truth position and velocity (BODY · imu+gnss+truth) *[needs truth]*<br>`quaternion_truth_vs_estimate` — Body-to-NED quaternion truth versus estimate (BODY2NED · imu+truth) *[needs truth]*<br>`quaternion_error_components` — Body-to-NED quaternion component errors (BODY2NED · imu+truth) *[needs truth]*<br>`euler_error_over_time` — Body-to-NED Euler angle errors (BODY2NED · imu+truth) *[needs truth]*<br>`attitude_error_angle` — Sign-invariant total attitude error angle (BODY2NED · imu+truth) *[needs truth]* |

## Cross-method comparison

Written once per multi-method run into `comparison/`, alongside `method_comparison.json` and `method_comparison.csv`.

| ID | Figure | Frame · data |
|---|---|---|
| C.1 | `initial_attitude_by_method` — Initial attitude solved by each method | BODY2NED · imu+gnss |
| C.2 | `fused_position_by_method` — Fused position from each method | NED · imu+gnss |
| C.3 | `position_error_by_method` — Position error norm from each method *[needs truth]* | NED · imu+gnss+truth |
| C.4 | `attitude_error_by_method` — Attitude error from each method *[needs truth]* | BODY2NED · imu+truth |
| C.5 | `metric_bars_by_method` — Task 7 metrics side by side | NONE · imu+gnss+truth |

## Behaviour without a truth file

Truth is optional. Tasks 1–5 run unchanged, Task 6 writes `status: skipped`
with a reason instead of fabricating a reference, and Task 7 reports GNSS
position/velocity innovation RMS so the run still carries a quality signal.
Figures that need truth are recorded in `figures_index.csv` with
`status = skipped` rather than being silently omitted.
