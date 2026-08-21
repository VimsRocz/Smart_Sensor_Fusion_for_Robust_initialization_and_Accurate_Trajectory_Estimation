# Python sensor-fusion scripts

From this directory, install the runtime dependencies with the same Python
interpreter that will run the pipeline:

```bash
python3 -m pip install -r requirements.txt
```

The safer one-command form checks the dependencies first and then launches the
selected script:

```bash
python3 run.py GNSS_IMU_Fusion.py \
  --imu-file IMU_X001.dat \
  --gnss-file GNSS_X001.csv \
  --method TRIAD
```

Replace `GNSS_IMU_Fusion.py` with another local script and put its arguments
after the script name. The main fusion, batch, and method-specific entry
points also run this preflight automatically when invoked directly.

The bootstrap uses `python3 -m pip`, so packages are installed for the exact
interpreter running the script. In a virtual environment, installation stays
inside that environment; with the system interpreter, it defaults to the user
site. Set `PIP_USER_INSTALL=0` to disable the user-site flag.

For an offline check, set `IMU_FUSION_SKIP_DEPENDENCY_INSTALL=1`; missing
packages are then reported without attempting a network install.
