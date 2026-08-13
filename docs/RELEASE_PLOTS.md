# Release Task 1–7 plots

This is **the** plot set. It is what the released version produced and what the
project expects. Everything lands flat in `results/`, named:

```text
<METHOD>_<IMU-stem>_<GNSS-stem>_task<N>_<name>.png    (and .pdf)
```

for example `TRIAD_IMU_X001_GNSS_X001_task5_8_4_fused_state_NED.png`.

## How to produce them

```bash
make release                                      # X001 + X001 + TRIAD
make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=SVD
make release-18                                   # all 18 combinations
make release-list                                 # list without running
```

Any replacement files in the documented fixed format:

```bash
make release-custom \
  IMU_FILE=/path/to/IMU_NEW.dat \
  GNSS_FILE=/path/to/GNSS_NEW.csv \
  TRUTH_FILE=/path/to/STATE_NEW.txt \
  METHOD=TRIAD
```

Directly:

```bash
.venv/bin/python PYTHON/run_release.py --imu x001 --gnss x001 --method TRIAD
```

The runner validates file structure and synchronized time coverage before it
invokes the full-rate Tasks 1–7 implementation. IMU and GNSS row counts are
expected to differ because their sample rates differ.

It also invokes MATLAB automatically and verifies that every PNG has a genuine,
directly openable same-stem `.fig`. Set `MATLAB_BIN=/path/to/matlab` on the Make
command if MATLAB is installed but not on `PATH`. Existing PNG results can be
converted without rerunning fusion using `make release-figs MATLAB_BIN=...`.

Requires the legacy dependencies:

```bash
.venv/bin/python -m pip install -e '.[release]'
```

## The plots, task by task

Every filename carries its **subtask number**, so `task5_8_3_mixed_frames` is
Task 5, subtask 8.3. The subtask numbers are the ones the source code itself
logs (`Subtask 4.13 Validate and Plot Data`, `Subtask 5.8 Plotting Results`, …).

| Subtask | File (after the `<IMU>_<GNSS>_<METHOD>_` prefix) | Content |
|---|---|---|
| **1.2** | `task1_2_location_map` | Initial position from the first GNSS fix, on an Earth map |
| **2.2** | `task2_2_static_interval` | Detected static/ZUPT interval on the IMU record |
| **2.2** | `IMU_X001_task2_2_zupt_variance` | ZUPT detection and accelerometer variance over time |
| **2.3** | `task2_3_vectors` | Measured body-frame gravity and Earth-rate vectors |
| **3.7.1** | `task3_7_1_errors` | Gravity and Earth-rate alignment error, TRIAD vs Davenport vs SVD |
| **3.7.2** | `task3_7_2_quaternions` | `qw, qx, qy, qz` across the three methods |
| **3.7.3** | `X001_<METHOD>_task3_7_3_attitude_angles_over_time` | Roll/pitch/yaw from the initial attitude |
| **4.13.1** | `task4_13_1_comparison_ned` | Derived GNSS vs derived IMU, NED |
| **4.13.2** | `task4_13_2_mixed_frames` | GNSS/IMU in mixed frames |
| **4.13.3** | `task4_13_3_all_ned` | Integrated IMU-only data, NED |
| **4.13.4** | `task4_13_4_all_ecef` | Integrated IMU-only data, ECEF |
| **4.13.5** | `task4_13_5_all_body` | Integrated IMU-only data, body |
| **5.8.1** | `task5_8_1_attitude_<METHOD>` | Attitude from the filter |
| **5.8.2** | `task5_8_2_results_<METHOD>` | **3×3: GNSS vs IMU vs Fused — position, velocity, acceleration in N/E/D** |
| **5.8.3** | `task5_8_3_mixed_frames` | **Position NED, Velocity ECEF, Acceleration Body — all three frames** |
| **5.8.4** | `task5_8_4_all_ned` | Kalman filter output, NED |
| **5.8.5** | `task5_8_5_all_ecef` | Kalman filter output, ECEF |
| **5.8.6** | `task5_8_6_all_body` | Kalman filter output, body |
| **5.8.7** | `task5_8_7_<method>_innovations` | Kalman innovations |
| **5.9.1** | `task5_9_1_position_residuals` | Position residuals vs GNSS |
| **5.9.2** | `task5_9_2_velocity_residuals` | Velocity residuals vs GNSS |
| **5.9.3** | `task5_9_3_residuals_<METHOD>` | Filter residual summary |
| **5.9.4** | `task5_9_4_velocity_profile` | Filter vs GNSS speed |
| **6.1** | `task6_1_attitude_angles` | Attitude angles over time |
| **6.2** | `X001_<METHOD>_task6_2_attitude_angles_over_time` | Attitude angles, full trajectory |
| **6.3** | `task6_3_truth_vs_fused` | Fused vs truth overlay (needs `--truth-file`) |
| **7.3** | `task7_3_ned_residuals` | 3×3 position/velocity/acceleration residuals in NED |
| **7.4** | `task7_4_ned_residual_norms` | Residual norms over time |
| **7.6** | `Task7_6_BodyToNED_attitude_*` | Attitude truth vs estimate (needs truth) |
| — | `tasks_overview` | Single-page overview of the run |

Figure **titles** carry the same numbers, e.g. `Task 5.8.3 – TRIAD – Mixed
Frames (Position NED, Velocity ECEF, Acceleration Body)`.

Each plot is written as **`.png` and native MATLAB `.fig`**, plus a `.mat`
companion holding the plotted arrays; most plotting paths also write PDF. The FIG stores the exact rendered
plot inside a native MATLAB figure, so it opens after upload without running a
redraw script. The release command fails before computation if MATLAB cannot be
found, unless the user explicitly supplies `ALLOW_MISSING_FIG=1`.

## Two fixes that were needed to produce these

Both were pre-existing defects that stopped the plots being written on a machine
without MATLAB:

1. **`PYTHON/src/utils/matlab_fig_export.py`** — `save_matlab_fig()` returned
   early when the MATLAB engine was unavailable, and the PNG/PDF export sat
   *inside* that MATLAB-only branch. So on any machine without MATLAB, all 30
   call sites silently produced no figures. The PNG/PDF export is pure
   Matplotlib and now always runs; only the `.fig` mirror needs MATLAB.

2. **`PYTHON/src/task7_ned_residuals_plot.py`** — two breakages: `float()` on a
   1-element array (removed in NumPy 2), and it looked for `pos_ned_m`/`vel_ned_ms`
   while `GNSS_IMU_Fusion.py` writes `pos_ned`/`vel_ned`. Both now accept either
   spelling.

## Fixes to the Task 5 mixed-frames figure

`task5_8_3_mixed_frames` had two defects, both now fixed in
`GNSS_IMU_Fusion.py`:

1. **The acceleration row had only one trace.** Rows 1 and 2 compared GNSS,
   Fused and Truth; the acceleration row plotted the fused series alone. It now
   plots the GNSS-derived acceleration (rotated into body) and the truth
   acceleration alongside the fused one, so all three rows compare the same
   three sources.
2. **The acceleration was invisible.** The IMU power-up transient reaches
   ~600 m/s², which pinned the y-axis and flattened the real signal onto zero.
   The acceleration panels now clip the y-limits to the 99.5th percentile of the
   fused trace, so the actual dynamics are readable. No data is altered — only
   the visible range.
3. **Only two frames were shown.** The figure was position ECEF, velocity ECEF,
   acceleration body. It is now **position NED, velocity ECEF, acceleration
   body**, so all three reference frames appear, one per row.

## Known issue in Task 7

`task7_ned_residuals` currently shows position residuals of order **10⁶ m**. That
is not a real error — truth and estimate are not being reduced to a common NED
origin inside that legacy script before differencing. The Task 5 residual plots
(`task5_residuals_position_residuals`, `task5_residuals_velocity_residuals`),
which compare against GNSS, are the trustworthy residual figures until this is
fixed.
