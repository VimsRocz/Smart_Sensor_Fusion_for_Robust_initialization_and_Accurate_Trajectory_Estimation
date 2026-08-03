# MATLAB implementation

`run_pipeline.m` runs one initialization method. `run_all_methods.m` runs TRIAD, Davenport and SVD and writes a comparison. The computational tasks are deliberately separated into `+fusion/task1.m` through `+fusion/task7.m`.

```matlab
addpath('MATLAB');

% One method
r = run_pipeline('method', 'Davenport', 'tasks', '1-7');

% All three methods and the cross-method comparison
c = run_all_methods('tasks', '1-7');
```

Explicit file paths and a configuration struct are supported:

```matlab
cfg = struct('static_samples', 200, 'plots', true);
r = run_pipeline( ...
  'imu',   'DATA/IMU/IMU_X001_small.dat', ...
  'gnss',  'DATA/GNSS/GNSS_X001_small.csv', ...
  'truth', 'DATA/Truth/STATE_X001_small.txt', ...
  'method','SVD', 'tasks','1-5', 'config', cfg);
```

Requires R2021a or newer (`readtable`, `readmatrix`, `tiledlayout`, `exportgraphics`, `jsonencode`). No third-party toolbox is used, and nothing from the Statistics or Aerospace toolboxes is called.

## Shared structure with Python

`+fusion/catalog.m` mirrors `PYTHON/fusion_pipeline/catalog.py`. Both define the same seven tasks, the same subtask numbering, the same output directory names, and the same figure slugs, frames and dataset tags. **A change to one catalog must be made in the other**, otherwise the two implementations will disagree about where files go and what they are called.

| Concern | Python | MATLAB |
|---|---|---|
| Task/subtask/figure catalog | `fusion_pipeline/catalog.py` | `+fusion/catalog.m` |
| Figure naming, stamping, index | `fusion_pipeline/figures.py` | `+fusion/figures.m` |
| Task directories | `task_01_inputs_reference/` … `task_07_evaluation/` | identical |
| Figure filenames | `<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png` | identical |
| Run index | `figures_index.json` / `.csv` | identical |

`+fusion/figures.m` exposes four actions:

```matlab
figCtx = fusion.figures('context', runId, method, datasetStems);
name   = fusion.figures('filename', figCtx, taskNumber, figureSlug);
figCtx = fusion.figures('save', figCtx, taskNumber, figureSlug, outDir, f);
figCtx = fusion.figures('skip', figCtx, taskNumber, figureSlug, reason);
         fusion.figures('write_index', figCtx, runDir);
```

`'save'` stamps the identifying title and footer onto the figure, writes the PNG under the catalog filename, closes the handle, and records it. Task functions therefore never call `exportgraphics`, `sgtitle` or `close` themselves.

## Figure index and coverage

Every run writes `figures_index.csv` and `figures_index.json` describing the **complete** catalog, not just what was produced. Each row carries one of three statuses:

| Status | Meaning |
|---|---|
| `written` | The PNG exists in the task directory. |
| `skipped` | Deliberately not produced for this run — for example a truth comparison when no reference file was supplied. The reason is recorded. |
| `not_implemented` | Present in the Python pipeline but not yet emitted by MATLAB. |

Sorting `figures_index.csv` by `status` therefore gives the current MATLAB-vs-Python parity gap for any run, without needing to consult this document.

## Output location

MATLAB task outputs are independent of Python and default to `results/matlab/`. The JSON schemas, quaternion convention (scalar-first `[w,x,y,z]`, Body→NED, unit norm) and height definition (`height = −NED Down`) match the Python implementation.

## Verification status

`MATLAB/tests/TestPipeline.m` covers the runner. Run it with:

```matlab
results = runtests('MATLAB/tests/TestPipeline.m')
```

The Python implementation is the reference: it is what CI executes and what the documented accuracy figures come from. When the two disagree, treat Python as correct and file the MATLAB difference as a bug.
