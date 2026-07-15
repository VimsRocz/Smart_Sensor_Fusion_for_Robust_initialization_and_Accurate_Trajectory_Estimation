# MATLAB implementation

`run_pipeline.m` runs one initialization method. `run_all_methods.m` runs TRIAD, Davenport, and SVD and writes a comparison. Computational tasks are intentionally separated under `+fusion/task1.m` through `+fusion/task7.m`.

```matlab
addpath('MATLAB');
r = run_pipeline('method','Davenport','tasks','1-7');
c = run_all_methods('tasks','1-7');
```

Explicit file paths and a configuration struct are supported:

```matlab
cfg = struct('static_samples',200,'plots',true);
r = run_pipeline( ...
  'imu','DATA/IMU/IMU_X001_small.dat', ...
  'gnss','DATA/GNSS/GNSS_X001_small.csv', ...
  'truth','DATA/Truth/STATE_X001_small.txt', ...
  'method','SVD','tasks','1-5','config',cfg);
```

Task outputs are independent of Python and default to `results/matlab/`. The schemas and quaternion/height conventions match the Python implementation.
