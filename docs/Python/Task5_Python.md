# Python Pipeline – Task 5 Kalman Filter Fusion

Task 5 fuses the IMU-only trajectory from Task 4 with the GNSS measurements using a simple Kalman filter.

## Overview

Corrected IMU data and GNSS updates are combined in a predict/update loop.  The filter outputs fused position, velocity and attitude estimates along with innovation diagnostics.

```text
IMU integration (Task 4)
       ↓
Kalman filter predict
       + GNSS updates → update
       ↓
Fused trajectory + innovations
```

## Subtasks

### 5.1 Set Up Logging
- Initialise the Python `logging` module and create log files in the results directory.

### 5.2–5.4 Prepare Inputs
- Convert GNSS data to NED and compute acceleration just as in Task 4.
- Correct the IMU specific force using the biases and rotation matrix from previous tasks.
- Integrate the IMU data to provide position and velocity priors.

### 5.6 Run the Kalman Filter
- Predict state and covariance with the integrated IMU data.
- Update with interpolated GNSS measurements at each epoch.
- Apply zero‑velocity updates when the IMU is detected to be static.

### 5.8 Plot and Summarise
- Save plots of fused position, velocity and acceleration for each method.
- Record innovations, residuals and attitude angles in the results folder.
- Write a small markdown summary listing the generated PDFs.
- The [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) keep plot legends consistent across tasks.

## Running the Script

Task 5 is reached once the IMU-only integration from Task 4 completes.  Launch
the fusion script for a dataset:

```bash
.venv/bin/python PYTHON/src/GNSS_IMU_Fusion.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv
```

The filter iterates over the measurement epochs and prints innovation
statistics to the terminal.

## Output Files

`PYTHON/results/` contains the fused trajectory and diagnostics:

- `<tag>_kf_output.mat` and `<tag>_kf_output.npz` with the state history
- `<tag>_task5_fused_state_<frame>.pdf` plots for NED, ECEF and body frames
- `plot_summary.md` with a table of generated figures

## Result

The Kalman filter produces the final fused trajectory and diagnostic statistics, completing the processing chain started in Task 1.
