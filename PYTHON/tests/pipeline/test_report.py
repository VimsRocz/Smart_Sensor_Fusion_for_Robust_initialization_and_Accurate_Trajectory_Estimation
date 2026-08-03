"""The terminal report must state datasets, task/subtask names and save locations."""

import io
from pathlib import Path

import pytest

from fusion_pipeline import PipelineConfig, run_methods, run_pipeline
from fusion_pipeline.catalog import TASKS
from fusion_pipeline.report import Progress, comparison_report, render_table, run_report

ROOT = Path(__file__).resolve().parents[3]
IMU = ROOT / "DATA/IMU/IMU_X001_small.dat"
GNSS = ROOT / "DATA/GNSS/GNSS_X001_small.csv"
TRUTH = ROOT / "DATA/Truth/STATE_X001_small.txt"


@pytest.fixture(scope="module")
def single_run(tmp_path_factory):
    """Fast run: structure only, no figures rendered."""
    return run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="TRIAD",
        tasks="1-7",
        output_root=tmp_path_factory.mktemp("report"),
        run_id="X001_small",
        config=PipelineConfig(static_samples=100, plots=False),
    )


@pytest.fixture(scope="module")
def plotted_run(tmp_path_factory):
    """Slower run that actually renders figures, so the tables list them."""
    return run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="TRIAD",
        tasks="1-7",
        output_root=tmp_path_factory.mktemp("report_plots"),
        run_id="X001_small",
        config=PipelineConfig(static_samples=100, plots=True),
    )


def test_table_truncates_instead_of_breaking_alignment():
    table = render_table(["A", "B"], [["short", "a-very-long-cell-value"]], widths=[6, 8])
    lines = table.splitlines()
    assert len({len(line) for line in lines}) == 1, "every row must be the same width"
    assert "…" in table


def test_report_names_every_dataset_with_its_file(single_run):
    text = run_report(single_run, "full")
    for role in ("IMU", "GNSS", "Truth"):
        assert role in text
    assert "IMU_X001_small" in text
    assert "GNSS_X001_small" in text
    assert "STATE_X001_small" in text
    assert "DATA/IMU/IMU_X001_small.dat" in text or "IMU_X001_small.dat" in text


def test_report_lists_every_task_subtask_and_output_folder(single_run):
    text = run_report(single_run, "full")
    for task in TASKS:
        assert task.name in text, f"task {task.number} name missing"
        assert task.directory_name in text, f"task {task.number} folder missing"
        for subtask in task.subtasks:
            assert subtask.number in text, f"subtask {subtask.number} missing"


def test_report_names_every_figure_and_its_frame(plotted_run):
    text = run_report(plotted_run, "full")
    for task in TASKS:
        for figure in task.figures:
            assert figure.slug in text, f"figure {figure.slug} missing"
            assert figure.frame in text


def test_report_says_figures_are_off_rather_than_listing_them(single_run):
    text = run_report(single_run, "full")
    assert "figures disabled" in text
    assert "input_time_coverage" not in text


def test_summary_mode_keeps_tasks_but_drops_the_per_figure_tables(single_run):
    text = run_report(single_run, "summary")
    assert "Tasks executed" in text
    assert TASKS[0].directory_name in text
    assert "Subtask name" not in text
    assert "Figures by task and subtask" not in text


def test_none_mode_is_empty(single_run):
    assert run_report(single_run, "none") == ""


def test_report_shows_the_run_folder_and_index_location(single_run):
    text = run_report(single_run, "full")
    assert "Run folder:" in text
    assert "figures_index.csv" in text
    assert "Output structure" in text


def test_progress_streams_every_task_subtask_and_result(tmp_path):
    stream = io.StringIO()
    run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="TRIAD",
        tasks="1-7",
        output_root=tmp_path,
        run_id="live",
        config=PipelineConfig(static_samples=100, plots=False),
        progress=Progress(enabled=True, stream=stream),
    )
    text = stream.getvalue()
    for task in TASKS:
        assert f"Task {task.number}/7 — {task.name}" in text
        assert f"Task {task.number} complete" in text
        for subtask in task.subtasks:
            assert f"    {subtask.number}  {subtask.name}" in text
    # A result line, not just the subtask name, for representative subtasks.
    assert "origin lat" in text
    assert "gravity" in text
    assert "GNSS updates applied" in text
    assert "position RMSE" in text


def test_progress_header_names_the_method_and_every_input_file(tmp_path):
    stream = io.StringIO()
    run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="Davenport",
        tasks="1-3",
        output_root=tmp_path,
        run_id="named",
        config=PipelineConfig(static_samples=100, plots=False),
        progress=Progress(enabled=True, stream=stream),
    )
    text = stream.getvalue()
    assert "RUNNING  method Davenport" in text
    # Role, dataset name and the actual file for each input.
    for role, name, filename in (
        ("IMU", "IMU_X001_small", "IMU_X001_small.dat"),
        ("GNSS", "GNSS_X001_small", "GNSS_X001_small.csv"),
        ("Truth", "STATE_X001_small", "STATE_X001_small.txt"),
    ):
        assert role in text
        assert name in text
        assert filename in text
    assert "1000 rows @ 400.0 Hz" in text
    assert "9 epochs" in text
    assert "99 states" in text
    assert "I = IMU_X001_small" in text
    assert "G = GNSS_X001_small" in text
    assert "T = STATE_X001_small" in text
    assert "Davenport on IMU_X001_small + GNSS_X001_small finished" in text


def test_every_task_line_states_the_method_and_the_datasets_it_uses(tmp_path):
    stream = io.StringIO()
    run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="SVD",
        tasks="1-7",
        output_root=tmp_path,
        run_id="perTask",
        config=PipelineConfig(static_samples=100, plots=False),
        progress=Progress(enabled=True, stream=stream),
    )
    text = stream.getvalue()
    for task in TASKS:
        assert f"▶ SVD · Task {task.number}/7 — {task.name}" in text
        assert f"✔ SVD · Task {task.number} complete" in text
    # Task 4 is IMU-only; Task 6 additionally consumes truth.
    assert "    data: IMU_X001_small    →  task_04_inertial_propagation/" in text
    assert (
        "    data: IMU_X001_small + GNSS_X001_small + STATE_X001_small"
        "    →  task_06_truth_overlay/"
    ) in text


def test_truth_free_run_omits_truth_from_the_header_and_task_lines(tmp_path):
    stream = io.StringIO()
    run_pipeline(
        IMU,
        GNSS,
        method="TRIAD",
        tasks="1-6",
        output_root=tmp_path,
        run_id="noTruth",
        config=PipelineConfig(static_samples=100, plots=False),
        progress=Progress(enabled=True, stream=stream),
    )
    text = stream.getvalue()
    assert "not supplied" in text
    assert "STATE_X001_small" not in text
    assert "T = " not in text
    assert "    data: IMU_X001_small + GNSS_X001_small    →  task_06_truth_overlay/" in text


def test_progress_reports_each_figure_as_it_is_written(tmp_path):
    stream = io.StringIO()
    run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="SVD",
        tasks="1-3",
        output_root=tmp_path,
        run_id="live_figs",
        config=PipelineConfig(static_samples=100, plots=True),
        progress=Progress(enabled=True, stream=stream),
    )
    text = stream.getvalue()
    for task in TASKS[:3]:
        for figure in task.figures:
            assert f"✎ {figure.subtask}  {figure.slug}" in text


def test_disabled_progress_writes_nothing(tmp_path):
    stream = io.StringIO()
    run_pipeline(
        IMU,
        GNSS,
        TRUTH,
        method="TRIAD",
        tasks="1-3",
        output_root=tmp_path,
        run_id="quiet",
        config=PipelineConfig(static_samples=100, plots=False),
        progress=Progress(enabled=False, stream=stream),
    )
    assert stream.getvalue() == ""


def test_comparison_report_tabulates_every_method(tmp_path):
    result = run_methods(
        IMU,
        GNSS,
        TRUTH,
        methods="ALL",
        tasks="1-7",
        output_root=tmp_path,
        run_id="cmp",
        config=PipelineConfig(static_samples=100, plots=False),
    )
    text = comparison_report(result, "full")
    for method in ("TRIAD", "Davenport", "SVD"):
        assert method in text
    assert "position_rmse_m" in text
    assert "method_comparison.csv" in text
