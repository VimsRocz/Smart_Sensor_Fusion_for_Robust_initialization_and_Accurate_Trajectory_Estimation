# Input and output contracts

The pipeline validates the complete input set before Task 1 writes anything. Validation errors name the file, the violated rule, and usually the row or column context.

The large release runner (`PYTHON/run_release.py`) performs an additional
fixed-format preflight for its 18 bundled combinations. It requires compatible
time coverage, not equal row counts: a 400 Hz IMU naturally has 400 rows for
each 1 Hz GNSS epoch. It rejects an IMU/GNSS pair whose inferred coverage
differs by more than one GNSS epoch, and it requires supplied truth to extend
through the common sensor window. Its exact replacement-file format is listed
in the root [README](../README.md#custom-fixed-format-release-files).

**Every layout below is a default, not a requirement.** If your files are formatted differently, declare the difference in the `pipeline:` section of your configuration instead of rewriting the data. Start from `config/pipeline_custom_data_template.yaml`, and print the live contract at any time with:

```bash
.venv/bin/python PYTHON/main.py --print-contract
```

**All column indices are zero-based.** Column 0 is the first column in the file.

---

## IMU contract

Numeric text with at least three rows.

| Default index | Meaning | Default units |
|---:|---|---|
| 0 | sample/count | ignored |
| 1 | sensor time | seconds |
| 2–4 | gyro x/y/z | delta angle, radians per sample |
| 5–7 | accelerometer x/y/z | delta velocity, m/s per sample |
| 8+ | temperature/status | ignored |

### Configuration keys

| Key | Default | Accepted values |
|---|---|---|
| `imu_time_column` | `1` | any zero-based column index |
| `imu_gyro_columns` | `[2, 3, 4]` | three zero-based column indices |
| `imu_accel_columns` | `[5, 6, 7]` | three zero-based column indices |
| `imu_measurement_type` | `delta` | `delta` (per-sample increments, divided by dt) or `rate` (already rad/s and m/s²) |
| `imu_time_unit` | `s` | `s`, `ms`, `us`, `ns` |
| `imu_gyro_unit` | `rad` | `rad`, `deg` |
| `imu_accel_unit` | `mps2` | `mps2`, `g`, `mg` (`g` = 9.80665 m/s² exactly, the unit definition — not the local gravity Task 1 computes) |
| `imu_delimiter` | `null` | `null` for any whitespace, or a single character such as `","` |

### Rules

- The median sample interval must lie between 1 µs and 10 s. A value outside that range usually means `imu_time_column` or `imu_time_unit` is wrong, and the error says so.
- Time must strictly increase after the known one-second clock reset is repaired. The number of repaired wraps is reported as `clock_wraps_repaired`.
- Every value in the used columns must be finite.
- Unit scaling is applied before the delta-to-rate division, so `imu_gyro_unit` and `imu_measurement_type` are independent.

---

## GNSS contract

Delimited text with a header row and at least two data rows. Columns are located by header name, not by position.

Canonical headers:

```text
Posix_Time,X_ECEF_m,Y_ECEF_m,Z_ECEF_m,VX_ECEF_mps,VY_ECEF_mps,VZ_ECEF_mps
```

Recognised without any configuration:

| Canonical value | Accepted header names |
|---|---|
| time | `Posix_Time`, `POSIX_Time`, `time`, `Time`, `timestamp` |
| ECEF X/Y/Z | `X_ECEF_m` / `Y_ECEF_m` / `Z_ECEF_m`, the same without `_m`, `ecef_x_m` etc., or lowercase `x` / `y` / `z` |
| ECEF VX/VY/VZ | `VX_ECEF_mps` etc., the same without `_mps`, `ecef_vx_mps` etc., or lowercase `vx` / `vy` / `vz` |

Extra columns — latitude, longitude, height, DOP, calendar fields — may be present and are ignored.

### Configuration keys

| Key | Default | Purpose |
|---|---|---|
| `gnss_column_overrides` | `{}` | Map any of `time, x, y, z, vx, vy, vz` to the exact header in your file. Naming a header that does not exist is an error that lists the headers found. |
| `gnss_delimiter` | `","` | Single character, e.g. `";"` or `"\t"` |
| `gnss_time_unit` | `s` | `s`, `ms`, `us`, `ns` |
| `gnss_position_unit` | `m` | `m`, `km`, `cm` |
| `gnss_velocity_unit` | `mps` | `mps`, `kmph`, `kn` |

### Rules

- Time must strictly increase.
- **Position must be true ECEF.** After unit scaling, every position norm must be at least 6,000 km. This rejects local or geodetic columns silently interpreted as ECEF, which would corrupt every downstream frame. Convert to ECEF before running.

---

## Truth contract (optional)

Numeric text with at least two rows.

```text
count time_s X_ECEF_m Y_ECEF_m Z_ECEF_m VX_ECEF_mps VY_ECEF_mps VZ_ECEF_mps [q0 q1 q2 q3]
```

### Configuration keys

| Key | Default | Purpose |
|---|---|---|
| `truth_time_column` | `1` | zero-based column index |
| `truth_position_columns` | `[2, 3, 4]` | ECEF position columns |
| `truth_velocity_columns` | `[5, 6, 7]` | ECEF velocity columns |
| `truth_quaternion_columns` | `[8, 9, 10, 11]` | Four columns, or `null` when the reference has no attitude |
| `truth_quaternion_order` | `xyzw` | `xyzw` or `wxyz` — the order **in your file** |
| `truth_quaternion_frame` | `body_to_ecef` | `body_to_ecef` or `body_to_ned` |
| `truth_attitude_time_offset_s` | `-0.05` | Attitude-only time correction |
| `truth_time_unit` / `truth_position_unit` / `truth_velocity_unit` | `s` / `m` / `mps` | as for GNSS |
| `truth_delimiter` | `null` | `null` for any whitespace, or a single character |

### Rules

- Defaults match the bundled `STATE_X001` files: `[qx,qy,qz,qw]`, Body→ECEF, with a `-0.05` s offset because its 10 Hz attitude represents the interval midpoint while the timestamp marks the interval start. **Set `truth_attitude_time_offset_s: 0.0` unless you know your reference shares that quirk.**
- The pipeline reorders each quaternion into `[w,x,y,z]` and, for a `body_to_ecef` source, uses that row's ECEF position to rotate it into the time-varying local NED frame before any interpolation or comparison. Position and velocity overlays still use Task 1's single fixed origin.
- A zero quaternion is rejected. Truth time must strictly increase.
- With `truth_quaternion_columns: null`, Tasks 6 and 7 compare position and velocity only; the attitude figures are recorded in `figures_index.csv` with `status = skipped`.

---

## Output guarantees

### Structure

- Each executed task has exactly one named directory, taken from the catalog in `PYTHON/fusion_pipeline/catalog.py`.
- Every subtask produces at least one figure.
- Every written Python figure has same-stem PNG, PDF and plotted-data MAT artifacts. With `--fig auto` MATLAB writes an editable native FIG when detected; `--fig on` makes a missing FIG an error and `--fig off` disables conversion. Python records FIG as deferred only when MATLAB is unavailable.
- A MAT companion is numeric interchange data, not a renamed figure. Native FIGs contain real MATLAB axes and plot objects and support zoom, pan, data tips and property editing.
- JSON summaries are written atomically (temp file plus rename).
- Large numeric arrays use compressed NPZ in Python and MAT in MATLAB.
- Each run writes `figures_index.json` and `figures_index.csv` listing every figure with its task, task name, subtask, subtask name, figure title, coordinate frame, source datasets and status.
- The run-level `manifest.json` records the schema version, resolved inputs, all configuration, requested tasks, inserted dependencies, method, status, timestamps, figure counts, metrics, and the embedded task catalog.

### Figure naming

```text
<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.<ext>
```

`<ext>` is `png`, `pdf`, `mat`, or `fig` as supported by the current runtime.

`<DATASETS>` lists the input file stems the figure actually draws from, joined with `+`. Frame tags are `ECEF`, `GEODETIC`, `BODY`, `NED`, `BODY2NED`, `BODYvsNED` and `NONE`; their meanings are in [TASKS_AND_SUBTASKS.md](TASKS_AND_SUBTASKS.md). The same metadata is stamped inside each image as a title and footer.

### Numerical conventions

- Every quaternion output is normalised `[w,x,y,z]`, Body→NED.
- Truth quaternion signs are aligned to the estimate before component plots; the geodesic attitude error is sign invariant.
- GNSS and truth position/velocity use Task 1's single fixed ECEF origin and rotation; Body→ECEF truth attitudes use a time-varying local NED rotation derived from each truth position.
- Relative height is `-position_ned_down_m` in Tasks 6 and 7.
- Task 1 uses latitude- and altitude-dependent WGS-84 normal gravity by default. Set a positive `gravity_override_mps2` only to substitute a calibrated local value; the output records both the normal value and whether it was overridden.
- Task 4 interpolates samples beyond `max_specific_force_mps2` and `max_angular_rate_rps` and records both counts in its summary.
- A missing truth file is never fabricated: Task 6 writes `status: skipped` with a reason, and Task 7 reports GNSS innovation metrics instead.
