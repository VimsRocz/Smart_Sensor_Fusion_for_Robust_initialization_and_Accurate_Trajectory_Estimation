# Getting Started

This page explains how to download the repository, keep it up to date and run the examples in both Python and MATLAB. Every step is kept short so that even a beginner can follow along.

## 1. Download the repository

1. Go to the [project page](https://github.com/.../IMU) in your web browser.
2. Click **Code** → **Download ZIP** to save a copy.
   Unzip it anywhere on your computer.
   Alternatively, install [Git](https://git-scm.com/) and run:
   ```bash
git clone <repo-url>
```

## 2. Update to the latest version

If you used Git to clone the project, you can update it later by running:
```bash
cd IMU
git pull
```
This downloads any new files from GitHub.

## 3. Install Python dependencies

1. Open a terminal or command prompt.
2. Change into the project folder:
   ```bash
   cd IMU
   ```
3. Install the required packages:
   ```bash
   pip install -r requirements.txt
   ```

## 4. Run the examples

### Python

From the same terminal run one of the helper scripts:
```bash
.venv/bin/python src/run_triad_only.py        # quick demo
# or
.venv/bin/python src/run_all_datasets.py      # full pipeline
```
The results and validation plots appear inside the newly created `results/run_triad_only/` or `results/run_all_datasets/` folder depending on the script. MATLAB writes to the same `results/` directory.

### MATLAB

1. Open MATLAB.
2. Use `cd` to change into the project directory.
3. Start the demo with:
   ```matlab
   run_triad_only
   % or
   run_all_datasets_matlab('TRIAD')
   ```
   MATLAB saves the figures to the `results/` folder as well. The batch runner lives in `MATLAB/run_all_datasets_matlab.m`.

## 5. Validate the output

Each script automatically checks its results against the reference data when available. Look for a summary message in the terminal or view the figures inside the appropriate results folder to confirm everything worked.

That's it! Once you see the plots you know the pipeline ran correctly.
