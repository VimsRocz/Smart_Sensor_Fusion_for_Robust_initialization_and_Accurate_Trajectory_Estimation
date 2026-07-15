
.PHONY: deps test smoke

# Install all Python dependencies needed for running the code and tests.
deps:
	pip install --upgrade pip setuptools wheel build
	pip install -e .[tests]

# Install dependencies and run the full test suite.
test: deps
	pytest -q

smoke:
	python PYTHON/run_pipeline.py --config config/pipeline_small.yaml --no-plots

