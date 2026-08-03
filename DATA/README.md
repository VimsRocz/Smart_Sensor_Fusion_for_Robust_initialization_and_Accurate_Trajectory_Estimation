# Dataset reference

Two sources, and they agree:

1. **`Tasks_Master_Project_Vimal.docx`** — the project specification, which defines the file formats and what each run number means. Quoted below as *(spec)*.
2. **The files themselves** — every format claim here was independently measured before the specification was consulted. Marked *(measured)*.

Anything inferred from neither is marked *(inferred)*.

Reproduce any number here with:

```bash
.venv/bin/python PYTHON/run_pipeline.py --config config/pipeline_x001_full.yaml --validate-only
```

---

## 1. The scenario

All three runs are the same flight. Derived from `Truth/STATE_X001.txt`:

| Event | Time | Altitude | Speed |
|---|---:|---:|---:|
| Power-up, stationary on the pad | 0 s | 160 m | 0 m/s |
| Liftoff | 1200.1 s | 160 m | 3.4 m/s |
| Max speed | 1245.6 s | 27,909 m | 1626.7 m/s |
| **Sensor data ends** | **1249.0 s** | **32,463 m** | **1604.5 m/s** |
| Apogee | 1388.5 s | 123,541 m | 886.5 m/s |
| Truth ends | 2008.6 s | 173 m | 107.2 m/s |

Launch site: latitude **−31.871189°**, longitude **133.455801°**, altitude 159.5 m — consistent with the Woomera range in South Australia *(inferred from coordinates; the files do not name a site)*. The profile is a sounding-rocket flight to 123 km *(inferred from the altitude and speed history)*.

### The single most important fact about this data

**The IMU and GNSS files stop at t = 1249 s.** They cover 20 minutes of standing still on the pad plus only **49 seconds of powered flight**. Apogee and the entire descent — 760 s of the trajectory — exist in the truth file but have **no sensor data at all**.

So every result this pipeline produces describes a long static alignment followed by a brief boost. It says nothing about apogee, re-entry or descent performance. This is also why the Task 4–7 plots are flat until t ≈ 1200 s and then move violently in the last few percent of the record.

---

## 2. IMU files — `IMU/IMU_X001.dat`, `IMU_X002.dat`, `IMU_X003.dat`

Whitespace-separated text, no header, **500,000 rows × 10 columns**, 400 Hz, 1250 s.

| Col | Spec name | Measured value / range |
|---:|---|---|
| 0 | Message Counter, range 0…255 | **8-bit, wraps 255 → 0.** Wraps 1953 times. Not a row index. |
| 1 | Sample Time in seconds, **reset at each PPS** | 0.0025 … 0.9975, resets to 0 every 1 s (1250 resets) |
| 2–4 | Angular increment for Gyro X / Y / Z | **delta angle [rad]** per 2.5 ms sample |
| 5–7 | Velocity increment for Accelerometer X / Y / Z | **delta velocity [m/s]** per 2.5 ms sample |
| 8 | Temperature | constant `22.5` °C — carries no information |
| 9 | Status | constant `1` — carries no information |

The column names are the spec's; the values are measured. Only columns 1–7 are used; 0, 8 and 9 are read and ignored.

### Proof that columns 2–7 are increments, not rates

Over the stationary first 1000 s, after dividing by the 2.5 ms sample interval:

| Quantity | Measured | Expected |
|---|---|---|
| mean \|angular rate\| | **7.292115 × 10⁻⁵ rad/s** | Earth's rotation rate = 7.292115 × 10⁻⁵ rad/s |
| mean \|specific force\| | **9.795240 m/s²** | local WGS-84 normal gravity = 9.794245 m/s² |

The gyroscope recovers Earth rate to seven significant figures. This is why the shipped configs use `imu_measurement_type: delta`. Treating these columns as rates would scale everything by 400.

### The two clock quirks, and one open requirement

1. **The counter wraps at 255**, so it cannot be used to order samples.
2. **The time column resets every second.** This is not a defect — *(spec)*: "The Pulse per Second (PPS) of the GNSS resets the internal clock of the IMU. Thus, the measurements can be synchronized." The reset is the synchronisation mechanism. The loader unwraps it and reports the count as `clock_wraps_repaired`.

Both are handled automatically; nothing needs editing.

> **Not yet implemented.** *(spec)*: "However, an ambiguity for the integer second remains and needs to be resolved."
>
> PPS pins the IMU clock to GNSS *within* a second, but which integer GNSS second the first IMU sample belongs to is not recorded anywhere in the file. The loader currently assumes IMU sample 0 coincides with GNSS epoch 0 and zero-bases both time axes. For this bundled data that assumption happens to hold, but it is an assumption, not a resolution. See [docs/REQUIREMENTS_COMPLIANCE.md](../docs/REQUIREMENTS_COMPLIANCE.md).

### The start-up transient

`IMU_X001.dat` contains **80 samples between t = 0.100 s and t = 0.297 s** where specific force spikes to **594 m/s²** — roughly 60 g while the vehicle is provably stationary. This is a power-up artefact, not motion.

Task 4 detects these against `max_specific_force_mps2: 100.0`, replaces them by interpolation, and reports the count. That is the `80 accel and 0 gyro samples interpolated` line in the run output. The same 80 samples are present in X002 and X003.

---

## 3. GNSS files — `GNSS/GNSS_X001.csv`, `GNSS_X002.csv`

CSV with a header row, **1250 data rows × 20 columns**, 1 Hz, 1249 s.

| Col | Header | Status |
|---:|---|---|
| 0–5 | `UTC_yyyy`, `UTC_MM`, `UTC_dd`, `UTC_HH`, `UTC_mm`, `UTC_ss` | present, **not read** |
| 6 | `Posix_Time` | **used** — the time base |
| 7–9 | `Latitude_deg`, `Longitude_deg`, `Height_deg` | **all exactly zero in both files — never use these** |
| 10–12 | `X_ECEF_m`, `Y_ECEF_m`, `Z_ECEF_m` | **used** — position, metres |
| 13–15 | `VX_ECEF_mps`, `VY_ECEF_mps`, `VZ_ECEF_mps` | **used** — velocity, m/s |
| 16–19 | `HDOP`, `VDOP`, `PDOP`, `TDOP` | **all exactly zero in both files** |

The latitude/longitude/height columns being zero is the trap in this format: they look usable and are not. The pipeline derives geodetic coordinates from the ECEF columns instead. `Height_deg` is also misnamed — a height in degrees is meaningless.

There is **no `GNSS_X003.csv`**. X003 reuses `GNSS_X002.csv`.

### The delivered format differs from the specification

*(spec)*: "The format of the GNSS is not yet known. It will contain the following data: GPS Week number, GPS Second of week, Position in ECEF, Velocity in ECEF."

The delivered files carry **`Posix_Time` plus six UTC calendar columns instead of GPS week and second-of-week**. Position and velocity in ECEF are present as specified. The loader reads `Posix_Time` and ignores the calendar columns, so no conversion is needed — but if a later drop switches to GPS week/second-of-week, point `gnss_column_overrides` at the second-of-week column and set the week number aside; the pipeline only needs a strictly increasing seconds axis.

---

## 4. Truth file — `Truth/STATE_X001.txt`

Whitespace-separated, `#` comment header, **20,087 rows × 12 columns**, 10 Hz, 2008.6 s.

| Col | Contents |
|---:|---|
| 0 | State counter |
| 1 | Time [s] |
| 2–4 | ECEF position x/y/z [m] |
| 5–7 | ECEF velocity x/y/z [m/s] |
| 8–11 | Attitude quaternion in **`[qx, qy, qz, qw]`** order, **Body→ECEF** |

Only X001 has a truth file.

### Two format quirks, both handled by configuration

- The quaternion is **vector-first and Body→ECEF**, so the shipped configs set `truth_quaternion_order: xyzw` and `truth_quaternion_frame: body_to_ecef`. The pipeline reorders it to `[qw,qx,qy,qz]` and rotates it into local NED at each truth position.
- The 10 Hz attitude represents the **midpoint** of each interval while the timestamp marks the interval **start**, hence `truth_attitude_time_offset_s: -0.05`. **Set this to `0.0` for any other reference trajectory** unless you have confirmed the same behaviour.

### This truth is not an independent reference

Measured against `GNSS_X001.csv`:

| Check | Result |
|---|---|
| Do the 1250 GNSS epochs fall on truth timestamps? | **Exactly** — max offset 0.00 s |
| Max position difference at those epochs | **5.00 × 10⁻⁶ m** |
| Max velocity difference at those epochs | **4.97 × 10⁻⁶ m/s** |

The truth file is written to 5 decimal places, so its rounding quantisation is ±5 × 10⁻⁶. The differences sit exactly at that bound. **`GNSS_X001.csv` is a 1 Hz decimation of the same state log that `STATE_X001.txt` stores at 10 Hz.** They are one trajectory printed twice at different precision.

Consequences for Task 7:

- **Position and velocity RMSE are circular.** Task 5 feeds GNSS into the filter, and truth ≡ GNSS, so `position_rmse_m` measures how tightly the filter tracks its own measurements — smoothing lag and tuning. It is a valid regression check. **It is not an independent accuracy figure and must not be reported as one.**
- **Attitude RMSE is not circular.** GNSS carries no attitude, and nothing in Tasks 1–5 reads the truth quaternion — Task 3 derives attitude from gravity and Earth rate, Task 4 propagates it. `attitude_rmse_deg` is therefore a genuine comparison against a reference the estimator never saw.

Also note the truth attitude is **constant for the first 1200 s** (59.8% of rows identical; first change at t = 1200.7 s). Attitude accuracy before liftoff is a static-alignment result only.

---

## 5. What each run number means

*(spec)*: "IMU_X001.dat is without additional errors. IMU_X002.dat is with noise. IMU_X003.dat adds bias to the IMU. GNSS_X001.csv is without error and GNSS_X002.csv is with error."

Every one of those statements is confirmed by differencing the files directly:

| Run | IMU | GNSS | Truth | Measured difference |
|---|---|---|---|---|
| **X001** | `IMU_X001.dat` | `GNSS_X001.csv` | `STATE_X001.txt` | **Error-free.** Consecutive IMU rows are bit-identical while stationary; GNSS matches truth to 5 × 10⁻⁶ m. |
| **X002** | `IMU_X002.dat` | `GNSS_X002.csv` | — | **X001 + zero-mean noise** on both IMU and GNSS. |
| **X003** | `IMU_X003.dat` | `GNSS_X002.csv` | — | **X002 + a constant IMU bias.** GNSS byte-identical to X002, as the spec implies — it defines only two GNSS files. |

### X002 = X001 + noise (measured)

| Signal | Added noise (1σ) | As a rate |
|---|---|---|
| Gyroscope | 4.00 × 10⁻⁷ rad per sample | **1.60 × 10⁻⁴ rad/s** |
| Accelerometer | 2.00 × 10⁻⁵ m/s per sample | **8.00 × 10⁻³ m/s²** |
| GNSS position | — | **≈ 2.0 m per axis** (max excursion 7.2 m) |
| GNSS velocity | — | **≈ 0.10 m/s per axis** |

All means are zero to within 10⁻⁸, so this is pure noise with no bias.

The GNSS position noise matches the shipped `gnss_position_std_m: 2.0` exactly. The shipped `gnss_velocity_std_mps: 0.5` is **five times more pessimistic** than the 0.10 m/s actually present — lowering it to `0.1` for X002/X003 would let the filter trust GNSS velocity appropriately.

### X003 = X002 + constant bias (measured)

Constant to the files' 6-significant-digit text precision:

| Axis | Gyroscope bias | Accelerometer bias |
|---|---|---|
| x | −1.52 × 10⁻⁶ rad/s (−0.314 °/hr) | +9.0 × 10⁻³ m/s² (+918 µg) |
| y | +3.80 × 10⁻⁶ rad/s (+0.784 °/hr) | −5.4 × 10⁻³ m/s² (−551 µg) |
| z | −2.28 × 10⁻⁶ rad/s (−0.470 °/hr) | +3.6 × 10⁻³ m/s² (+367 µg) |

### Task 3 cannot recover this bias per-axis — and that is correct behaviour

It is tempting to treat the table above as a target for Task 3's `accel_bias_body_mps2`. It is not. Running Tasks 1–3 on X002 and X003 and differencing the estimates gives:

| | x | y | z |
|---|---:|---:|---:|
| Injected | +9.0 × 10⁻³ | −5.4 × 10⁻³ | +3.6 × 10⁻³ m/s² |
| Task 3 recovered | +7.69 × 10⁻³ | −4.08 × 10⁻⁶ | −1.85 × 10⁻³ m/s² |

The y axis is missed almost entirely. This is **not** a defect. Measured on this data:

- The recovered bias vector is parallel to the measured specific-force direction to within **0.0005°**.
- Its magnitude equals the projection of the injected bias onto that direction to within **0.04%** (7.9113 × 10⁻³ vs 7.9144 × 10⁻³ m/s²).
- The perpendicular component — **7.78 × 10⁻³ m/s², nearly half the total** — is lost entirely.
- Task 3 reports `gravity_error_deg = 0.000e+00` for both runs.

That last line is the explanation. TRIAD, Davenport and SVD all drive the gravity residual to exactly zero, so any accelerometer bias perpendicular to the specific-force vector is indistinguishable from a small tilt and gets absorbed into the attitude instead. **A static two-vector alignment can only observe the bias component along gravity.** This is the standard tilt/bias ambiguity, and it is a property of the geometry, not of this implementation.

The gyroscope behaves similarly but less cleanly: x and z came within 3% of the injected values while y was missed by 90%, because the Earth-rate observation is only partially matched by TRIAD.

**What this means in practice:** observing the full three-axis bias requires motion, or physically rotating the unit to obtain a second independent gravity direction. With 20 minutes of pad time and 49 s of flight, this dataset can only pin down the along-gravity component. Any claim that the pipeline "estimates IMU biases" should be qualified accordingly.

### Intended pairing

X003 has no GNSS of its own. The pairing is confirmed by the legacy `config_full.yml`:

```yaml
- imu: IMU_X001.dat   gnss: GNSS_X001.csv
- imu: IMU_X002.dat   gnss: GNSS_X002.csv
- imu: IMU_X003.dat   gnss: GNSS_X002.csv
```

Pairing `IMU_X003.dat` with `GNSS_X001.csv` would mix a biased IMU against noise-free GNSS — a combination the data was not built for.

---

## 6. Small variants

Every run also ships a `*_small` variant: the **first 1000 IMU rows** (2.5 s), 9 GNSS epochs, and 99 truth states. Verified: `IMU_X001_small.dat` is bit-identical to the first 1000 rows of `IMU_X001.dat`.

These are for fast smoke tests only. 2.5 s is **entirely within the stationary pad phase and includes the 80-sample start-up spike**, so accuracy numbers from a small run are not meaningful — use them to check that the code runs, not how well it performs.

---

## 7. Which configuration to use

The quickest route is `--dataset`, which resolves the pairings in the table above so X003 can never be paired with the wrong GNSS:

```bash
make list-datasets                   # every dataset and its pairing
make run-x003                        # IMU_X003 + GNSS_X002, no truth, one method
make run-x003 METHOD=SVD             # pick the method
.venv/bin/python PYTHON/run_pipeline.py --dataset x003 --method TRIAD
```

Shipped configuration files, for runs that need tuning changes as well:

| File | Data | Truth |
|---|---|---|
| `config/pipeline_small.yaml` | X001 small | yes |
| `config/pipeline_x001_full.yaml` | X001 full | yes |
| `config/pipeline_x002_no_truth.yaml` | X002 full | no — exercises the truth-optional path |
| `config/pipeline_x003_no_truth.yaml` | X003 full (`IMU_X003` + `GNSS_X002`) | no |
| `config/pipeline_custom_data_template.yaml` | your own files | your choice |

`--no-truth` drops the reference even for a dataset that has one, which is the way to run X001 under the same conditions as X002 and X003.

Full column and unit options: [docs/INPUT_OUTPUT_CONTRACTS.md](../docs/INPUT_OUTPUT_CONTRACTS.md).
