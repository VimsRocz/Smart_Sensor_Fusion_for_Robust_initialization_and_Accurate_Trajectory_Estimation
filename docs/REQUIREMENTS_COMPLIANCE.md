# Requirements compliance

Validation of this repository against **`Tasks_Master_Project_Vimal.docx`** ("Master Project — Postprocessing of IMU and GNSS Data") and the **`coding_rules.pdf`** modelling and coding guidelines.

Legend: **✅ met** · **⚠️ partial** · **❌ not met**

Reproduce the mechanical checks with:

```bash
make test          # unit tests
make docs          # regenerate and diff the generated task documentation
make list-datasets # dataset pairings
```

---

## 1. Data format — fully verified

Every format statement in the specification was independently measured from the files. All agree. Details and the measurement method are in [DATA/README.md](../DATA/README.md).

| Requirement | Status | Evidence |
|---|:--:|---|
| IMU col 0: Message Counter, range 0…255 | ✅ | 8-bit, wraps 1953 times over 500,000 rows |
| IMU col 1: Sample Time [s], reset at each PPS | ✅ | 1250 resets; unwrapped and reported as `clock_wraps_repaired` |
| IMU cols 2–4: angular increment, gyro X/Y/Z | ✅ | ÷dt recovers Earth rate 7.292115 × 10⁻⁵ rad/s to 7 figures |
| IMU cols 5–7: velocity increment, accel X/Y/Z | ✅ | ÷dt recovers 9.795240 m/s² vs local gravity 9.794245 |
| IMU col 8: Temperature | ✅ | constant 22.5 °C |
| IMU col 9: Status | ✅ | constant 1 |
| GNSS: Position in ECEF | ✅ | read and validated (norm ≥ 6000 km enforced) |
| GNSS: Velocity in ECEF | ✅ | read and validated |
| GNSS: GPS Week number + Second of week | ⚠️ | **Delivered files carry `Posix_Time` + UTC calendar columns instead.** The spec anticipated this ("format not yet known"). The loader reads `Posix_Time`; `gnss_column_overrides` handles a future switch. |
| IMU_X001 without additional errors | ✅ | static rows bit-identical |
| IMU_X002 with noise | ✅ | gyro 1.60 × 10⁻⁴ rad/s, accel 8.00 × 10⁻³ m/s² (1σ), zero mean |
| IMU_X003 adds bias | ✅ | 0.31–0.78 °/hr gyro, 367–918 µg accel, constant |
| GNSS_X001 without error | ✅ | matches truth to 5 × 10⁻⁶ m (its rounding limit) |
| GNSS_X002 with error | ✅ | 2.0 m position, 0.10 m/s velocity (1σ) |

---

## 2. Goal and tasks

| Requirement | Status | Notes |
|---|:--:|---|
| Read IMU and GNSS data files | ✅ | Task 1. Column indices, delimiters and units are all configurable, so a reformatted drop needs no code change. |
| Estimate 6DOF navigation (position, velocity, attitude) | ✅ | Position/velocity from Tasks 4–5, attitude from Tasks 3–4. |
| Gyro compassing for initial attitude | ✅ | Task 3 solves Wahba's problem on the averaged gravity and Earth-rate vectors — three ways (TRIAD, Davenport, SVD). This is gyro compassing. |
| Alternative: zero-velocity updates + Earth-rate measurements | ⚠️ | Task 2 selects a minimum-variance static window, which is a ZUPT *detector*. The filter applies no explicit zero-velocity **updates** during that window. |
| First guess: IMU Z axis points up | ⚠️ | Not used. The attitude is solved directly from the measured vectors, which is stronger — but the assumption is not available as a fallback if the static window is poor. |
| **Detect time of lift-off from the data** | ❌ | **Not implemented.** Lift-off is at t = 1200.1 s (established here from the truth file, not from the sensor data). |
| **Filter processes from lift-off, or a few seconds before** | ❌ | **Not implemented.** Task 5 runs the filter over the entire record from t = 0, including 20 minutes of standing still. |
| **Resolve the integer-second ambiguity** | ❌ | **Not implemented.** PPS pins the IMU clock within a second; which integer GNSS second sample 0 belongs to is not resolved. The loader assumes IMU t=0 ≡ GNSS t=0. |
| Kalman filter for processing during flight | ✅ | Task 5: six-state NED position/velocity filter with Joseph-form updates at each GNSS epoch. |

### Why the three gaps matter

They compound. Because lift-off is not detected, the filter spends 96% of its samples on a stationary vehicle, and the only 49 s of flight the sensors actually cover is processed with a filter tuned across the whole record. And because the integer-second ambiguity is unresolved, a data drop where the IMU does **not** start on the same second as GNSS would be silently misaligned by a whole second — at 1600 m/s that is a 1.6 km position error with no warning.

Both are tractable:

- **Lift-off detection** — threshold the specific-force magnitude against the static mean established in Task 2. The static window gives the noise floor for free, so a fixed multiple of its standard deviation is a defensible trigger.
- **Integer-second resolution** — cross-correlate IMU-derived speed against GNSS speed over a range of integer-second offsets and take the maximum. The current assumption becomes the zero-offset special case, and the chosen offset gets recorded in the manifest.

---

## 3. Software requirements

| Requirement | Status | Evidence |
|---|:--:|---|
| **Written in MATLAB R2022b** | ❌ | **Python is the reference implementation** — it is what CI runs, what the 60 unit tests cover, and what produced every documented number. A MATLAB implementation exists under `MATLAB/` with matching task structure, directory names and figure naming, but **it has never been executed** (no MATLAB on the development machine) and is therefore unverified. |
| All functions have unit tests in the MATLAB test framework | ❌ | 8 test cases in 1 class covering 18 MATLAB functions. The Python side has 60 tests. |
| Follow the Modelling and Coding Guidelines | ⚠️ | See §4. |

**This is the single largest gap in the project.** The specification names MATLAB R2022b as the implementation language; the working, tested, validated implementation is Python. That is a deliberate and reasonable engineering position — Python is verifiable on this machine and MATLAB is not — but it should be stated openly rather than left implicit, and it needs a decision:

1. Treat Python as a prototype and port to MATLAB, with MATLAB unit tests as the acceptance gate; or
2. Get the requirement changed; or
3. Run the existing MATLAB code on a machine with R2022b and close the verification gap directly. `MATLAB/tests/TestPipeline.m` already contains tests that check the catalog, figure naming and that every indexed figure file exists — running `runtests('MATLAB/tests/TestPipeline.m')` is the fastest way to find out where it stands.

---

## 4. Modelling and Coding Guidelines (`coding_rules.pdf`)

| Rule | Status | Evidence |
|---|:--:|---|
| No global variables | ✅ | 0 occurrences in `MATLAB/` |
| File names: no blank spaces, single dot, meaningful | ✅ | 0 offending files |
| Variables start lower case, underscore-separated | ✅ | — |
| Variables in SI units | ✅ | units carried in names (`_m`, `_mps`, `_rps`, `_deg`) |
| Rotation matrices named `DCM_<FRAME>_from_<FRAME>` | ❌ | uses `c_body_to_ned` (5 occurrences) |
| Quaternions named `quat_<FRAME>_from_<FRAME>` | ❌ | uses `quaternion_wxyz_body_to_ned` (8) |
| Positions named `pos_<object>_wrt_<FRAME>_in_<FRAME>` | ❌ | uses `position_ned_m` (14) |
| Velocities named `vel_<object>_wrt_<FRAME>_in_<FRAME>` | ❌ | uses `velocity_ned_mps` (14) |
| Vectors always column vectors | ⚠️ | mixed; several MATLAB functions return row vectors |
| Embedded MATLAB functions carry `%#codegen` | ❌ | 0 files |
| `varargin` shall not be used | ❌ | 4 files: `run_pipeline.m`, `run_all_methods.m`, `+fusion/math3d.m`, `+fusion/figures.m` |
| Comment generously | ✅ | — |

The naming rules are the substantive ones: they are unambiguous, machine-checkable, and renaming touches the JSON/`.mat` field names that the Python side also writes — so it is a coordinated change across both implementations, not a MATLAB-local edit.

`varargin` is used by the two entry points for name/value arguments. The guideline targets on-board Embedded MATLAB functions; entry-point scripts are arguably out of scope, but the rule is written without that exception.

---

## 5. What is verified right now

| Check | Result |
|---|---|
| Python unit tests | **60 passed** |
| Lint (`ruff`) on all canonical Python | **clean** |
| Generated task documentation in sync with the catalog | **unchanged** |
| All 6 bundled dataset pairings parse | **6/6 OK** |
| Full X001, all three methods, Tasks 1–7 | **complete**, 119 figures |
| Full X002 without truth, all three methods | **complete**, 28 figures/method + 10 recorded as skipped |
| MATLAB implementation | **never executed** |

---

## 6. Priority

1. **Resolve the integer-second ambiguity** — explicitly required, currently an unstated assumption, and silently wrong on any drop that violates it.
2. **Detect lift-off and start the filter there** — explicitly required, and it changes every Task 5–7 number.
3. **Decide the MATLAB question** — the requirement says MATLAB; the verified implementation is Python.
4. **Apply the guideline naming** — mechanical, but coordinate it across both implementations at once.
5. **Extend MATLAB unit test coverage** to every function.

Items 1 and 2 are ordinary engineering work against a clear specification. Item 3 is a decision, not a task.
