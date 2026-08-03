# Use the project virtualenv when it exists, otherwise python3.
# Never bare "python" or "pip": with pyenv set to "system" those shims resolve
# to nothing and every target would fail with "command not found".
VENV := $(CURDIR)/.venv
PY := $(shell if [ -x "$(VENV)/bin/python" ]; then echo "$(VENV)/bin/python"; else command -v python3 || echo python3; fi)
RUN = $(PY) PYTHON/run_pipeline.py
# The release pipeline: produces the Task 1-7 plots in results/ with the
# <IMU>_<GNSS>_<METHOD>_task<N>_<name> naming used by the released version.
RELEASE = $(PY) PYTHON/src/GNSS_IMU_Fusion.py
TASK7 = $(PY) PYTHON/src/task7_ned_residuals_plot.py
IMU ?= DATA/IMU/IMU_X001.dat
GNSS ?= DATA/GNSS/GNSS_X001.csv
TRUTH ?= DATA/Truth/STATE_X001.txt
SMALL = config/pipeline_small.yaml
FULL = config/pipeline_x001_full.yaml

# One method by default. Override on the command line:
#   make run-x003 METHOD=SVD          make run-x001 METHOD=ALL
METHOD ?= TRIAD
# Which bundled dataset a `make run-dataset` call uses.
DATASET ?= x003

.PHONY: help venv deps test smoke docs doctor gui \
        list-tasks contract validate validate-full \
        run-triad run-davenport run-svd run-all \
        run-triad-full run-davenport-full run-svd-full run-all-full \
        run-x002-no-truth run-everything clean-results \
        list-datasets run-dataset run-x001 run-x002 run-x003 \
        run-x001-small run-x002-small run-x003-small \
        release release-all release-triad release-davenport release-svd \
        release-methods release-datasets release-everything release-mix clean-heavy

# ---------------------------------------------------------------------------
help:
	@echo "Using Python: $(PY)"
	@echo ""
	@echo "First time"
	@echo "  make venv                 create .venv and install everything"
	@echo "  make doctor               show which interpreter and packages are found"
	@echo ""
	@echo "Release Task 1-7 plots  ->  results/"
	@echo "  make release                            x001 + TRIAD (defaults)"
	@echo "  make release DATASET=x002 METHOD=SVD    any dataset x any method"
	@echo "  make release-methods DATASET=x002       all 3 methods, one dataset"
	@echo "  make release-datasets METHOD=TRIAD      one method, all 3 datasets"
	@echo "  make release-everything                 all 3 methods x all 3 datasets"
	@echo "     datasets: x001 (has truth) | x002 | x003     methods: TRIAD|Davenport|SVD"
	@echo "  make release-mix IMU_FILE=... GNSS_FILE=...   pair any IMU with any GNSS"
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
	$(VENV)/bin/python -m pip install -e '.[tests]'
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
# Release Task 1-7 plots — pick any dataset and any method.
#
#   make release                              x001 + TRIAD (defaults)
#   make release DATASET=x002 METHOD=SVD      any single combination
#   make release-methods DATASET=x002         all 3 methods, one dataset
#   make release-datasets METHOD=TRIAD        one method, all 3 datasets
#   make release-everything                   all 3 methods x all 3 datasets
DATASET ?= x001
RELEASE_SH = bash scripts/run_release.sh

release:
	$(RELEASE_SH) $(DATASET) $(METHOD)

release-triad:
	$(RELEASE_SH) $(DATASET) TRIAD

release-davenport:
	$(RELEASE_SH) $(DATASET) Davenport

release-svd:
	$(RELEASE_SH) $(DATASET) SVD

# All three methods on one dataset.
release-methods:
	$(RELEASE_SH) $(DATASET) TRIAD
	$(RELEASE_SH) $(DATASET) Davenport
	$(RELEASE_SH) $(DATASET) SVD

# One method on all three datasets.
release-datasets:
	$(RELEASE_SH) x001 $(METHOD)
	$(RELEASE_SH) x002 $(METHOD)
	$(RELEASE_SH) x003 $(METHOD)

# Everything: 3 methods x 3 datasets.
release-everything:
	$(MAKE) release-methods DATASET=x001
	$(MAKE) release-methods DATASET=x002
	$(MAKE) release-methods DATASET=x003

# Cross-pair any IMU with any GNSS (all files share one time base).
#   make release-mix IMU_FILE=DATA/IMU/IMU_X002.dat GNSS_FILE=DATA/GNSS/GNSS_X001.csv
#   make release-mix IMU_FILE=... GNSS_FILE=... TRUTH_FILE=DATA/Truth/STATE_X001.txt
release-mix:
	IMU_FILE="$(IMU_FILE)" GNSS_FILE="$(GNSS_FILE)" TRUTH_FILE="$(TRUTH_FILE)" \
	  $(RELEASE_SH) mix $(METHOD)

# Backwards-compatible alias.
release-all: release-methods

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
