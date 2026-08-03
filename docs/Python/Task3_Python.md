# Python Pipeline – Task 3 Attitude Determination

Task 3 solves Wahba's problem using a small SVD-based routine that refines the traditional TRIAD approach.

## Overview

The normalised gravity and Earth‑rate vectors from Tasks 1 and 2 are paired to compute a body‑to‑NED rotation matrix and a quaternion.  Diagnostic plots compare TRIAD with the alternative methods.

```text
Body vectors (Task 2) + Reference vectors (Task 1)
    ↓
TRIAD → rotation matrix → quaternion
    ↓
Optional comparison plots
```

## Subtasks

### 3.1 Prepare Vector Pairs
- Normalise the gravity and Earth‑rate vectors in both frames.
- Optionally recompute the Earth‑rate vector in closed form to verify the sign convention.

### 3.2 Compute TRIAD Matrix
The vectors are fed to `triad_svd()`, which forms the Wahba matrix and
solves for the optimal rotation with a singular value decomposition.
This is numerically more robust than the classic cross-product formula.
The file `init_vectors.py` also defines a placeholder `triad_basis()`
function for potential cross-language alignment in the future.

### 3.3 Quaternion Output
- Convert `R_tri` to a quaternion with `rot_to_quaternion()` and enforce a positive scalar component.
- Log the quaternion and append it to `triad_init_log.txt`.

### 3.4 Check Alignment
- Rotate the body vectors with `R_tri` and compute gravity and Earth‑rate errors in degrees.
- Save the comparison plot as `results/<tag>_task3_errors_comparison.pdf`.
- Use the [standardized legend terms](../PlottingChecklist.md#standardized-legend-terms) so the curves match the rest of the documentation.

### 3.5 Plot Quaternion Components
- Plot `qw`, `qx`, `qy`, `qz` alongside Davenport and SVD for context.
- The figure is saved as `results/<tag>_task3_quaternions_comparison.pdf`.

### 3.6 Validate Attitude Determination and Compare Methods
- Evaluate the TRIAD, Davenport and SVD solutions by rotating the body-frame gravity
  and Earth‑rate vectors and measuring the angle to the corresponding navigation-frame
  references.
- Print the per‑method errors and warn if all Earth‑rate errors differ by less than
  `1\u00d710\u22125\u00b0`.
- An example output is shown below:

```text
Attitude errors using reference vectors (Case 1):
TRIAD      -> Gravity error (deg): 0.000000
TRIAD      -> Earth rate error (deg):  0.000001
Davenport  -> Gravity error (deg): 0.000000
Davenport  -> Earth rate error (deg):  0.000001
SVD        -> Gravity error (deg): 0.000000
SVD        -> Earth rate error (deg):  0.000000

Detailed Earth-Rate Errors:
  TRIAD     : 0.000001°
  Davenport : 0.000001°
  SVD       : 0.000000°

Earth-rate errors by method:
  TRIAD     : 0.000000854°
  Davenport : 0.000000854°
  SVD       : 0.000000000°
  Δ = 8.54e-07° (tolerance = 1.0e-05)
Warning: All Earth-rate errors are very close; differences are within 1.0e-05°
```

## Running the Script

Task 3 runs automatically after Tasks 1 and 2.  Execute the pipeline for a
dataset as shown below:

```bash
.venv/bin/python PYTHON/src/GNSS_IMU_Fusion.py --imu-file IMU_X001.dat --gnss-file GNSS_X001.csv
```

The console reports per‑method attitude errors, while the figures compare the
TRIAD, Davenport and SVD solutions.

## Output Files

Two comparison plots are saved to `PYTHON/results/` using the dataset tag:

- `<tag>_task3_errors_comparison.pdf`
- `<tag>_task3_quaternions_comparison.pdf`

## Result

Task 3 produces the initial attitude as a rotation matrix and quaternion.  The values are required for IMU integration in **Task 4** and the Kalman filter in **Task 5**.
