# Project document traceability

The canonical Python and MATLAB pipelines implement the three numbered outputs
called out in the supplied project presentation and report. The task catalogs
are the executable source of truth; this page records where each document
requirement is implemented and what files it produces.

## Required project figures

| Requirement | Project source | Canonical output | Python | MATLAB |
|---|---|---|---|---|
| **Task 4.6** | `Project_PPT.pdf`, PDF pages 35–37 | GNSS-derived versus IMU-derived position, velocity and acceleration as 3×3 grids in NED, ECEF and Body | `fusion_pipeline/figures.py` → `draw_task4` | `+fusion/task4.m` |
| **Task 5.10** | `Project_PPT.pdf`, PDF pages 49–51 | Final fused position, velocity and acceleration as 3×3 grids in NED, ECEF and Body | `fusion_pipeline/figures.py` → `draw_task5` | `+fusion/task5.m` |
| **Task 7.6** | `Project_PPT.pdf`, PDF pages 54–57 | Fused-versus-truth position/velocity in Body, ECEF and NED plus quaternion, quaternion-component error, Euler error and total attitude-error plots | `fusion_pipeline/figures.py` → `draw_task7` | `+fusion/task7.m` |
| **Task 7 differences** | `Project_PPT.pdf`, PDF pages 58–60 | Fused-minus-truth residual histories in Body, ECEF and NED | Task 7 residual figures | Task 7 residual figures |
| **User-facing execution and output** | `Project_Report.pdf`, PDF pages 38–39, section 7.6 | Python main/GUI, MATLAB runners, configurable inputs and PNG/PDF/MAT/FIG outputs | `PYTHON/main.py`, `gui.py` | `MATLAB/run_pipeline.m`, `MATLAB/run_all_methods.m` |

## Figure contract

Every canonical figure uses the same export and presentation path:

- a task/subtask title, labelled axes, units, legends and grid styling;
- a filename containing run, method, task, subtask, frame and source datasets;
- same-stem **PNG** for preview, **PDF** for the report and **MAT** for numeric data;
- a same-stem native **FIG** when MATLAB is available;
- one row in `figures_index.csv` and `figures_index.json` recording all formats.

Python writes PNG/PDF/MAT directly. MATLAB writes PNG/PDF/FIG while the live
figure handle exists. On a system without MATLAB, native FIG creation is
reported as deferred rather than creating an invalid renamed MAT file.

## Coordinate and quaternion corrections

- ECEF comparisons are anchored at the first GNSS ECEF position; an IMU local
  displacement is never plotted as though it were an absolute ECEF position.
- Relative height is `-Down` from the common NED origin in both implementations.
- Quaternions are scalar-first `[w,x,y,z]`, normalized and hemisphere-aligned
  before interpolation or component subtraction.
- Quaternion axes show actual component values rather than a misleading
  Matplotlib scientific-offset display.

## Maintenance rule

When a task or subtask changes, update both
`PYTHON/fusion_pipeline/catalog.py` and `MATLAB/+fusion/catalog.m`, regenerate
`docs/TASKS_AND_SUBTASKS.md` with `make docs`, and run the Python and MATLAB
test suites. Independent Python entry points live in
`PYTHON/fusion_pipeline/tasks/`; MATLAB tasks remain in `MATLAB/+fusion/`.
