# Input and output contracts

The pipeline validates the complete input set before Task 1 writes anything. Validation errors include the file, violated rule, and usually the row/column context.

## IMU contract

Whitespace-separated numeric text with at least three rows and eight columns:

| Index | Meaning | Default units |
|---:|---|---|
| 0 | sample/count | integer-like |
| 1 | sensor time | seconds |
| 2–4 | gyro x/y/z | delta angle, radians |
| 5–7 | accelerometer x/y/z | delta velocity, m/s |
| 8+ | optional temperature/status | ignored by canonical computation |

Set `pipeline.imu_measurement_type: rate` when columns 2–4 already contain rad/s and columns 5–7 contain m/s². All used values must be finite. The median interval must be between 1 µs and 10 s. Strict time is required after the known one-second clock reset is repaired.

## GNSS contract

CSV with a header and at least two rows. Canonical required headers are:

```text
Posix_Time,X_ECEF_m,Y_ECEF_m,Z_ECEF_m,VX_ECEF_mps,VY_ECEF_mps,VZ_ECEF_mps
```

Accepted aliases:

| Canonical value | Accepted names |
|---|---|
| time | `Posix_Time`, `POSIX_Time`, `time`, `Time`, `timestamp` |
| ECEF X/Y/Z | `X_ECEF_m`/`Y_ECEF_m`/`Z_ECEF_m`, versions without `_m`, `ecef_x_m` etc., or lowercase `x/y/z` |
| ECEF VX/VY/VZ | `VX_ECEF_mps` etc., versions without `_mps`, `ecef_vx_mps` etc., or lowercase `vx/vy/vz` |

Time must strictly increase. Position is metres and must have an Earth-scale norm of at least 6,000 km. Velocity is m/s. Latitude, longitude, height, DOP, and calendar fields may be present but are not required.

## Truth contract

Optional whitespace-separated numeric text with at least two rows and eight columns:

```text
count time_s X_ECEF_m Y_ECEF_m Z_ECEF_m VX_ECEF_mps VY_ECEF_mps VZ_ECEF_mps [q0 q1 q2 q3]
```

The four quaternion columns are interpreted using three configuration declarations:

- `truth_quaternion_order`: `xyzw` or `wxyz`;
- `truth_quaternion_frame`: `body_to_ecef` or `body_to_ned`.
- `truth_attitude_time_offset_s`: attitude-only time correction; bundled `STATE_X001` uses `-0.05` s because its 10 Hz attitude represents the interval midpoint.

Defaults (`xyzw`, `body_to_ecef`) match the bundled `STATE_X001` files. The pipeline reorders each quaternion and uses that row's ECEF position to rotate it into the time-varying local NED frame before interpolation or comparison. (Position/velocity overlays still use Task 1's fixed NED origin.) A zero quaternion is rejected. Truth time must strictly increase.

## Output guarantees

- Each executed task has exactly one named directory.
- JSON summaries are written atomically.
- Large numeric arrays use compressed NPZ in Python and MAT in MATLAB.
- Every quaternion output is normalized `[w,x,y,z]`, Body→NED.
- Truth quaternion signs are aligned to the estimated quaternion before component plots; geodesic attitude error is sign invariant.
- ECEF truth and GNSS position/velocity use Task 1's single fixed origin and rotation; Body→ECEF truth attitudes use a time-varying local NED rotation derived from each truth position.
- Relative height is `-position_ned_down_m` in Task 6 and Task 7.
- A missing truth file is not fabricated. Task 6 writes `status: skipped`; Task 7 reports innovation metrics.
- Task 4 rejects and interpolates samples beyond configurable `max_specific_force_mps2` and `max_angular_rate_rps` limits, and records both counts.

The run-level `manifest.json` records schema version, resolved inputs, all numeric configuration, requested tasks, inserted dependencies, method, status, timestamps, and metrics.

Task 1 uses latitude/altitude-dependent WGS-84 normal gravity by default. Set a positive `gravity_override_mps2` only when a calibrated local value should replace it; the output records both the normal value and whether it was overridden.
