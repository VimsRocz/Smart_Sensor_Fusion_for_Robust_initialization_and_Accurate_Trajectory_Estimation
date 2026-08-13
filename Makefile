# Use the project virtualenv when it exists, otherwise python3.
# Never bare "python" or "pip": with pyenv set to "system" those shims resolve
# to nothing and every target would fail with "command not found".
VENV := $(CURDIR)/.venv
PY := $(shell if [ -x "$(VENV)/bin/python" ]; then echo "$(VENV)/bin/python"; else command -v python3 || echo python3; fi)
RUN = $(PY) PYTHON/run_pipeline.py
RELEASE_RUN = $(PY) PYTHON/run_release.py
SMALL = config/pipeline_small.yaml
FULL = config/pipeline_x001_full.yaml

# One method by default. Override on the command line:
#   make run-x003 METHOD=SVD          make run-x001 METHOD=ALL
METHOD ?= TRIAD
# Which bundled dataset a `make run-dataset` call uses.
DATASET ?= x003
# Release cross-product selectors: 3 IMUs x 2 GNSS files x 3 methods.
IMU_ID ?= x001
GNSS_ID ?= x001
TRUTH_FILE ?= DATA/Truth/STATE_X001.txt
OUTPUT_DIR ?= results
RELEASE_TRUTH = $(if $(filter 1 yes true,$(NO_TRUTH)),--no-truth,--truth "$(TRUTH_FILE)")
RELEASE_MATLAB = $(if $(strip $(MATLAB_BIN)),--matlab-bin "$(MATLAB_BIN)") \
                 $(if $(filter 1 yes true,$(ALLOW_MISSING_FIG)),--allow-missing-fig)

.PHONY: help venv deps test smoke docs doctor gui \
        list-tasks contract validate validate-full \
        run-triad run-davenport run-svd run-all \
        run-triad-full run-davenport-full run-svd-full run-all-full \
        run-x002-no-truth run-everything clean-results \
        list-datasets run-dataset run-x001 run-x002 run-x003 \
        run-x001-small run-x002-small run-x003-small \
        release release-combo release-18 release-list release-check release-figs \
        release-custom clean-heavy

# ---------------------------------------------------------------------------
help:
	@echo "Using Python: $(PY)"
	@echo ""
	@echo "First time"
	@echo "  make venv                 create .venv and install everything"
	@echo "  make doctor               show which interpreter and packages are found"
	@echo ""
	@echo "Release Task 1-7 plots  ->  results/"
	@echo "  make release                            X001 IMU + X001 GNSS + TRIAD"
	@echo "  make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=SVD"
	@echo "  make release-18                         all 18 IMU x GNSS x method runs"
	@echo "  make release-list                       list the exact 18 combinations"
	@echo "  make release-check IMU_ID=x002 GNSS_ID=x001 METHOD=Davenport"
	@echo "  make release-figs MATLAB_BIN=/path/to/matlab   FIGs for existing PNGs"
	@echo "  make release-custom IMU_FILE=... GNSS_FILE=... METHOD=TRIAD"
	@echo "  Add NO_TRUTH=1 to omit Tasks 6-7 truth comparisons."
	@echo "  MATLAB is required for native .fig output; set MATLAB_BIN if not on PATH."
	@echo ""
	@echo "Discovery"
	@echo "  make list-tasks           print every task, subtask and figure"
	@echo "  make contract             print the accepted input file layouts"
	@echo "  make validate             validate the small bundled inputs"
	@echo "  make validate-full        validate the full X001 inputs"
	@echo ""
	@echo "Small dataset (1,000 IMU samples - seconds to run)"
	@echo "  make run-triad            Tasks 1-7 with TRIAD"
	@echo "  make run-davenport        Tasks 1-7 with Davenport's Q-method"
	@echo "  make run-svd              Tasks 1-7 with SVD/Wahba"
	@echo "  make run-all              all three methods + cross-method comparison"
	@echo ""
	@echo "Full X001 dataset (~500,000 IMU samples)"
	@echo "  make run-triad-full       make run-davenport-full   make run-svd-full"
	@echo "  make run-all-full         all three methods + comparison"
	@echo ""
	@echo "One method on one dataset (METHOD defaults to $(METHOD))"
	@echo "  make run-x003             IMU_X003 + GNSS_X002, no truth"
	@echo "  make run-x002             IMU_X002 + GNSS_X002, no truth"
	@echo "  make run-x001             IMU_X001 + GNSS_X001 + STATE_X001"
	@echo "  make run-x003-small       ...and the 2.5 s smoke variants"
	@echo "  make run-x003 METHOD=SVD  pick the method; METHOD=ALL runs all three"
	@echo "  make run-dataset DATASET=x002 METHOD=Davenport"
	@echo "  make list-datasets        every bundled dataset and its IMU/GNSS pairing"
	@echo ""
	@echo "Other"
	@echo "  make run-x002-no-truth    full X002 with no reference trajectory"
	@echo "  make run-everything       every option above, in order"
	@echo "  make test                 install dependencies and run pytest"
	@echo "  make smoke                fast all-method run without figures"
	@echo "  make clean-results        delete the results/ tree"

# ---------------------------------------------------------------------------
# Create the project virtualenv and install the package into it.
# Safe to re-run: python3 -m venv is idempotent.
venv:
	python3 -m venv $(VENV)
	$(VENV)/bin/python -m pip install --upgrade pip setuptools wheel
	$(VENV)/bin/python -m pip install -e '.[tests,release]'
	@echo ""
	@echo "Done. Run targets now use $(VENV)/bin/python automatically:"
	@echo "    make run-all"
	@echo "To use the interpreter directly instead:"
	@echo "    source .venv/bin/activate"

# Report what the Makefile resolved, so a broken setup is obvious immediately.
doctor:
	@echo "Makefile interpreter : $(PY)"
	@$(PY) -c "import sys; print('version              :', sys.version.split()[0]); print('executable           :', sys.executable)"
	@$(PY) -c "import numpy, matplotlib, yaml; print('numpy                :', numpy.__version__); print('matplotlib           :', matplotlib.__version__); print('pyyaml               : ok')" \
	  || echo "MISSING PACKAGES     : run 'make venv'"
	@$(PY) -c "import scipy, pandas, filterpy, rich, plotly; print('release dependencies : ok')" \
	  || echo "RELEASE PACKAGES     : missing (run 'make venv')"
	@$(PY) -c "import sys; sys.path.insert(0, 'PYTHON'); from run_release import find_matlab; p=find_matlab(); print('MATLAB executable    :', p or 'NOT FOUND (required for native .fig)')"
	@$(PY) -c "import fusion_pipeline; print('fusion_pipeline      : importable')" 2>/dev/null \
	  || echo "fusion_pipeline      : not installed (run 'make venv'); run_pipeline.py still works"

deps: venv

test:
	$(PY) -m pytest -q

gui:
	$(PY) gui.py

smoke:
	$(RUN) --config $(SMALL) --no-plots

# Regenerate docs/TASKS_AND_SUBTASKS.md after editing the task catalog.
docs:
	$(PY) scripts/generate_task_docs.py

# ---------------------------------------------------------------------------
# Release Task 1-7 plots — pick any of 3 IMUs x 2 GNSS files x 3 methods.
#
#   make release
#   make release-combo IMU_ID=x003 GNSS_ID=x001 METHOD=SVD
#   make release-18
# The Python runner validates fixed-format columns and synchronized time
# coverage before invoking the full-rate Tasks 1-7 pipeline.

release release-combo:
	$(RELEASE_RUN) --imu "$(IMU_ID)" --gnss "$(GNSS_ID)" --method "$(METHOD)" \
	  $(RELEASE_TRUTH) $(RELEASE_MATLAB) --output "$(OUTPUT_DIR)"

release-18:
	$(RELEASE_RUN) --all $(RELEASE_TRUTH) $(RELEASE_MATLAB) --output "$(OUTPUT_DIR)"

release-list:
	$(RELEASE_RUN) --list

release-check:
	$(RELEASE_RUN) --imu "$(IMU_ID)" --gnss "$(GNSS_ID)" --method "$(METHOD)" \
	  $(RELEASE_TRUTH) --check-only

release-figs:
	$(RELEASE_RUN) --export-figs-only $(RELEASE_MATLAB) --output "$(OUTPUT_DIR)"

release-custom:
	$(RELEASE_RUN) --imu "$(IMU_FILE)" --gnss "$(GNSS_FILE)" --method "$(METHOD)" \
	  $(RELEASE_TRUTH) $(RELEASE_MATLAB) --output "$(OUTPUT_DIR)"

# ---------------------------------------------------------------------------
list-tasks:
	$(RUN) --list-tasks

contract:
	$(RUN) --print-contract

validate:
	$(RUN) --config $(SMALL) --validate-only

validate-full:
	$(RUN) --config $(FULL) --validate-only

# --- one method at a time, small dataset -----------------------------------
run-triad:
	$(RUN) --config $(SMALL) --method TRIAD

run-davenport:
	$(RUN) --config $(SMALL) --method Davenport

run-svd:
	$(RUN) --config $(SMALL) --method SVD

run-all:
	$(RUN) --config $(SMALL) --method ALL

# --- one method at a time, full dataset ------------------------------------
run-triad-full:
	$(RUN) --config $(FULL) --method TRIAD

run-davenport-full:
	$(RUN) --config $(FULL) --method Davenport

run-svd-full:
	$(RUN) --config $(FULL) --method SVD

run-all-full:
	$(RUN) --config $(FULL) --method ALL

# --- one method on one bundled dataset -------------------------------------
# The dataset selector resolves the verified IMU/GNSS/truth pairing, so X003
# always gets GNSS_X002 and never the noise-free GNSS_X001.
list-datasets:
	$(RUN) --list-datasets

run-dataset:
	$(RUN) --dataset $(DATASET) --method $(METHOD)

run-x001:
	$(RUN) --dataset x001 --method $(METHOD)

run-x002:
	$(RUN) --dataset x002 --method $(METHOD)

run-x003:
	$(RUN) --dataset x003 --method $(METHOD)

run-x001-small:
	$(RUN) --dataset x001_small --method $(METHOD)

run-x002-small:
	$(RUN) --dataset x002_small --method $(METHOD)

run-x003-small:
	$(RUN) --dataset x003_small --method $(METHOD)

# --- truth-optional demonstration ------------------------------------------
run-x002-no-truth:
	$(RUN) --config config/pipeline_x002_no_truth.yaml

run-everything: run-all run-all-full run-x002-no-truth
	@echo "All run options completed. Results are under results/."

clean-results:
	rm -rf results

# Drop the multi-hundred-MB intermediates but keep every plot.
clean-heavy:
	find results -name "*_kf_output.npz" -delete 2>/dev/null || true
	find results -name "*_kf_output.mat" -delete 2>/dev/null || true
	find results -name "*.pkl.gz" -delete 2>/dev/null || true
	find results -name "*.mat" -size +5M -delete 2>/dev/null || true
	rm -f triad_init_log.txt task5_results_all_methods.mat
	@echo "Plots kept. Re-run 'make release' to regenerate the intermediates."
