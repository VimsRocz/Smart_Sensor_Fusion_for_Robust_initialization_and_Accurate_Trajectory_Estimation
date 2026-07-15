# Python implementation

Use `run_pipeline.py` as the stable entry point. The `fusion_pipeline/` package contains input contracts, attitude methods, Tasks 1–7, comparison logic, and the CLI.

From the repository root:

```bash
python PYTHON/run_pipeline.py --config config/pipeline_small.yaml
python PYTHON/run_pipeline.py --method SVD --tasks 1-5 \
  --imu DATA/IMU/IMU_X001_small.dat \
  --gnss DATA/GNSS/GNSS_X001_small.csv
```

Install the canonical package with `python -m pip install -e .`. Legacy scripts under `src/` may require the optional dependencies installed by `python -m pip install -e '.[legacy]'`.

See the repository [README](../README.md) and [input/output contracts](../docs/INPUT_OUTPUT_CONTRACTS.md).
