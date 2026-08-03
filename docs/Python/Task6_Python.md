# Python Pipeline – Task 6 State Overlay

Task 6 visualises the Kalman filter output from Task 5 together with the raw
`STATE_X` trajectory. Only the ``*_overlay_state.pdf`` figures are produced.

## Overview

The script loads the stored filter results (`*_kf_output.mat` or `.npz`),
retrieves the corresponding GNSS and IMU logs and aligns them with the ground
truth file. The helper :func:`plot_overlay` then creates 3×3 figures in the NED,
ECEF and body frames.  Since NED uses a positive *Down* axis, the script flips
that component before plotting so altitude increases upward in the figures. The
layout mirrors the Task 5 plots but replaces the GNSS series with the truth
trajectory.

```text
Task 5 output
       + Ground truth → overlay plots
```

## Subtasks

### 6.1 Load Estimator Result
Parse the Task 5 output file and obtain fused position, velocity and acceleration.

### 6.2 Assemble Frames
Use :func:`assemble_frames` to align IMU, GNSS and truth data in NED, ECEF and body frames.

### 6.3 Generate Plots
Call :func:`plot_overlay` for each frame to produce overlay figures. The ``--show-measurements`` flag adds the raw IMU and GNSS curves.

### 6.4 Save Overlay Figures
Overlay PDFs are stored directly in ``results/`` as ``<dataset>_<method>_task6_overlay_state_<frame>.pdf``.

## Running the Script

Invoke the Task 6 helper once Task 5 has produced a filter output file:

```bash
.venv/bin/python PYTHON/src/task6_plot_truth.py --est-file PYTHON/results/IMU_X001_GNSS_X001_TRIAD_kf_output.mat \
    --output PYTHON/results
```

Add `--show-measurements` to include raw IMU and GNSS curves in the overlay.

## Output Files

The command writes overlay figures directly to `PYTHON/results/` following the
pattern `<dataset>_<method>_task6_overlay_state_<frame>.pdf` for NED, ECEF and
body frames.

## Result

Task 6 yields comparison plots that clearly show how well the fused trajectory
matches the reference solution. When required arrays are absent from the
estimator output (for example the state history ``x_log``), the Python script
now issues a warning and skips the overlay step instead of failing.
