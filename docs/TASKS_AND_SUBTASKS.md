# Tasks, subtasks, and execution model

## Dependency rule

Tasks form a strict prefix: `1 → 2 → 3 → 4 → 5 → 6 → 7`. A downstream request automatically runs and saves every prerequisite. `--tasks 4`, for example, means “produce a valid Task 4 result,” so Tasks 1–3 are included and recorded as `auto_dependencies` in the manifest.

This makes each run reproducible and prevents a Task 5 result from silently consuming stale Task 1–3 files. To change any task, edit the versioned YAML configuration or pass CLI values and rerun the desired prefix.

## Task 1 — Inputs and navigation reference

- 1.1 validates IMU/GNSS/truth structures and units.
- 1.2 derives WGS-84 latitude, longitude, altitude, and the ECEF→NED rotation from the first GNSS fix.
- 1.3 computes latitude-dependent normal gravity and the Earth-rate vector in NED.

## Task 2 — Static IMU body vectors

- 2.1 converts delta angle/velocity to SI rates when configured.
- 2.2 selects the lowest-variance window of configurable length.
- 2.3 averages the specific-force and angular-rate vectors and normalizes the pairs used by Task 3.

## Task 3 — Initial attitude and biases

- 3.1 runs TRIAD, Davenport, or SVD/Wahba for a Body→NED rotation.
- 3.2 projects the matrix onto SO(3) and writes a normalized scalar-first quaternion.
- 3.3 derives accelerometer and gyro biases by comparing static measurements with the chosen attitude's expected gravity/Earth-rate observations.

Task 3 is the only algorithmic branch between the three methods. Every downstream result remains method-specific.

## Task 4 — IMU-only propagation

- 4.1 screens configurable physical-range outliers, interpolates rejected samples, removes Task 3 biases, and compensates the local Earth rate. The rejected counts are saved in Task 4's summary.
- 4.2 propagates and continuously normalizes the Body→NED quaternion while compensating both Earth rate and velocity-dependent NED transport rate.
- 4.3 rotates specific force into NED, adds gravity and Coriolis acceleration, and integrates velocity/position.

## Task 5 — GNSS/IMU fusion

- 5.1 predicts a six-state NED position/velocity filter with IMU acceleration.
- 5.2 performs Joseph-form position/velocity updates whenever an asynchronous GNSS epoch becomes available.
- 5.3 exports fused state and innovation histories.

## Task 6 — Truth comparison

- 6.1 restricts truth and estimate to the common time range and applies the declared attitude-only time offset (the bundled 10 Hz truth uses −0.05 s midpoint alignment).
- 6.2 converts truth ECEF position/velocity with Task 1's exact origin and rotation.
- 6.3 computes relative height as negative NED Down.
- 6.4 converts Body→ECEF truth attitude into the time-varying local NED frame at every truth position, then normalizes and hemisphere-aligns truth/estimate quaternions before plotting.

## Task 7 — Evaluation and method comparison

- 7.1 computes NED position residuals.
- 7.2 computes NED velocity residuals.
- 7.3 computes sign-invariant quaternion geodesic error when truth attitude exists.
- 7.4 exports RMSE/final/max metrics to JSON; an all-method run also writes comparison JSON/CSV/PNG.

Without truth, Task 7 reports GNSS position/velocity innovation RMS so the run still has a useful quality signal.
