# Python Pipeline – Task 7 Filter Evaluation

Task 7 analyses the filter residuals and attitude history. The Python scripts load the residual arrays produced in Task 5 and create diagnostic figures directly in ``results/`` using the dataset tag ``<TAG>`` as part of each filename.

## Overview

Residual position and velocity are compared with the GNSS data. When a truth trajectory is available the difference ``truth - fused`` is also plotted.

## Subtasks

### 7.1 Load Data
- Read ``residual_pos`` and ``residual_vel`` along with ``attitude_q`` from ``*_kf_output.npz``.
- Check the array lengths and truncate mismatched samples.

### 7.2 Compute Residuals
- Interpolate GNSS position and velocity to the estimator time vector.
- Print mean and standard deviation of the residuals.

### 7.3 Plot Residuals
- Save `<tag>_task7_3_residuals_position_velocity.pdf` and an error norm plot in the same folder.
- `<tag>` now includes the IMU dataset, GNSS log and method, e.g. `IMU_X002_GNSS_X002_Davenport`.

### 7.4 Plot Attitude Angles
- Convert the quaternion history to Euler angles.
- Write `<tag>_task7_4_attitude_angles_euler.pdf` with roll, pitch and yaw over time.

### 7.5 Truth – Fused Difference
- If the reference trajectory is provided, plot component-wise differences.
- Figures are saved as `<tag>_task7_5_diff_truth_fused_over_time_<frame>.pdf` for
  all frames (NED, ECEF and Body).

### 7.6 Attitude Truth vs Estimate
- Generate quaternion plots comparing truth and Kalman filter estimates.
- Save `<tag>_task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png`.
- Plot quaternion component errors and Euler angle errors over time.
- Additional files include `<tag>_task7_6_BodyToNED_attitude_quaternion_error_components.png`,
  `<tag>_task7_6_BodyToNED_attitude_euler_error_over_time.png` and
  `<tag>_task7_6_attitude_error_angle_over_time.png`.

## Running the Script

Run the evaluation helper after Task 5 to generate residual and attitude plots:

```bash
.venv/bin/python PYTHON/src/run_all_methods.py --task 7
```

The script searches `PYTHON/results/` for `*_kf_output.npz` files and prints mean
residual statistics for each dataset.

## Output Files

All figures are stored in `PYTHON/results/` using the dataset and method tag:

- `<tag>_task7_3_residuals_position_velocity.pdf`
- `<tag>_task7_4_attitude_angles_euler.pdf`
- `<tag>_task7_5_diff_truth_fused_over_time_<frame>.pdf`
- `<tag>_task7_6_BodyToNED_attitude_truth_vs_estimate_quaternion.png`
- `<tag>_task7_6_BodyToNED_attitude_quaternion_error_components.png`
- `<tag>_task7_6_BodyToNED_attitude_euler_error_over_time.png`
- `<tag>_task7_6_attitude_error_angle_over_time.png`

## Result

Task 7 produces residual and attitude plots that summarise the filter performance for each data set.
