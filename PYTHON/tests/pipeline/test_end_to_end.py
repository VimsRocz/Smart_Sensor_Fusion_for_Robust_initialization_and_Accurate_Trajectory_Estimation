import json
from pathlib import Path

import numpy as np

from fusion_pipeline import METHODS, PipelineConfig, run_methods, run_pipeline

ROOT = Path(__file__).resolve().parents[3]
IMU = ROOT / "DATA/IMU/IMU_X001_small.dat"
GNSS = ROOT / "DATA/GNSS/GNSS_X001_small.csv"
TRUTH = ROOT / "DATA/Truth/STATE_X001_small.txt"


def test_individual_method_writes_every_task_contract(tmp_path):
    result = run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="TRIAD",
        tasks="1-7",
        output_root=tmp_path,
        run_id="single",
        config=PipelineConfig(static_samples=100, plots=False),
    )
    assert result["manifest"]["status"] == "complete"
    assert set(result["tasks"]) == set(range(1, 8))
    assert np.isclose(result["tasks"][3]["quaternion_norm"], 1.0)
    assert result["tasks"][6]["height_definition"].startswith("height_m = -")
    assert result["tasks"][7]["metrics"]["attitude_rmse_deg"] < 0.1
    assert (result["run_dir"] / "task_07_evaluation/metrics.json").is_file()


def test_all_methods_create_machine_readable_comparison(tmp_path):
    result = run_methods(
        IMU,
        GNSS,
        TRUTH,
        tasks="1-7",
        output_root=tmp_path,
        run_id="all",
        config=PipelineConfig(static_samples=100, plots=False),
    )
    assert list(result["results"]) == list(METHODS)
    comparison_path = result["comparison_dir"] / "method_comparison.json"
    comparison = json.loads(comparison_path.read_text(encoding="utf-8"))
    assert comparison["methods"] == list(METHODS)
    assert set(comparison["metrics"]) == set(METHODS)


def test_truth_is_optional_and_task6_is_not_fabricated(tmp_path):
    result = run_pipeline(
        IMU,
        GNSS,
        method="svd",
        tasks="7",
        output_root=tmp_path,
        run_id="no_truth",
        config=PipelineConfig(static_samples=100, plots=False),
    )
    assert result["tasks"][6]["status"] == "skipped"
    assert result["tasks"][7]["status"] == "complete_without_truth"
    assert result["tasks"][7]["metrics"]["gnss_updates"] > 0
