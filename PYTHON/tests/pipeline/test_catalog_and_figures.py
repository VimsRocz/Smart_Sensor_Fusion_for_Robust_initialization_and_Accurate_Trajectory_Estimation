"""The catalog is the single source of truth, so guard its invariants."""

from pathlib import Path

import pytest

from fusion_pipeline.catalog import (
    COMPARISON_FIGURES,
    FRAMES,
    TASKS,
    catalog_as_dict,
    describe_catalog,
)
from fusion_pipeline.figures import FigureWriter

ROOT = Path(__file__).resolve().parents[3]


def test_every_task_declares_seven_numbered_tasks_with_subtasks():
    assert [task.number for task in TASKS] == list(range(1, 8))
    for task in TASKS:
        assert task.subtasks, f"Task {task.number} has no subtasks"
        assert task.directory_name == f"task_{task.number:02d}_{task.slug}"


def test_every_subtask_has_at_least_one_figure():
    for task in TASKS:
        covered = {figure.subtask for figure in task.figures}
        declared = {subtask.number for subtask in task.subtasks}
        assert declared == covered, (
            f"Task {task.number} subtasks without a figure: {sorted(declared - covered)}"
        )


def test_figure_slugs_are_unique_within_a_task():
    for task in TASKS:
        slugs = [figure.slug for figure in task.figures]
        assert len(slugs) == len(set(slugs))


def test_filenames_carry_task_subtask_frame_and_dataset():
    writer = FigureWriter(
        run_id="X001_small",
        method="TRIAD",
        dataset_tags={"imu": "IMU_X001_small", "gnss": "GNSS_X001_small", "truth": "STATE_X001_small"},
        enabled=False,
    )
    for task in TASKS:
        for figure in task.figures:
            name = writer.filename(task, figure)
            assert name.startswith("X001_small_TRIAD_")
            assert f"_task{task.number:02d}_" in name
            assert f"_sub{figure.subtask}_" in name
            assert task.slug in name
            assert figure.slug in name
            assert f"_frame-{figure.frame}_" in name
            assert name.endswith(".png")
            # Nothing that would break a filesystem or a shell glob.
            assert not any(ch in name for ch in ' ()[]{}*?"\'')


def test_truth_free_runs_drop_truth_from_the_dataset_tag():
    writer = FigureWriter(
        run_id="run",
        method="SVD",
        dataset_tags={"imu": "IMU_X002", "gnss": "GNSS_X002", "truth": None},
        enabled=False,
    )
    assert writer.data_tag(("imu", "gnss", "truth")) == "IMU_X002+GNSS_X002"


def test_comparison_figures_are_named_independently_of_a_single_method():
    writer = FigureWriter(
        run_id="run",
        method="TRIAD+Davenport+SVD",
        dataset_tags={"imu": "IMU", "gnss": "GNSS", "truth": "TRUTH"},
        enabled=False,
    )
    for figure in COMPARISON_FIGURES:
        name = writer.comparison_filename(figure)
        assert "ALLMETHODS" in name
        assert f"_sub{figure.subtask}_" in name


def test_disabled_writer_records_every_figure_without_drawing(tmp_path):
    writer = FigureWriter(
        run_id="run",
        method="TRIAD",
        dataset_tags={"imu": "IMU", "gnss": "GNSS", "truth": None},
        enabled=False,
    )
    writer.emit(1, "reference_origin_map", tmp_path, lambda plt: pytest.fail("must not draw"))
    paths = writer.write_index(tmp_path)
    assert paths["json"].is_file() and paths["csv"].is_file()
    assert writer.records[0]["status"] == "disabled"


def test_catalog_text_and_dict_stay_in_sync():
    text = describe_catalog()
    payload = catalog_as_dict()
    assert len(payload["tasks"]) == len(TASKS)
    for task in TASKS:
        assert task.name in text
        for subtask in task.subtasks:
            assert subtask.number in text
    for tag in FRAMES:
        assert tag in text


def test_documented_task_table_matches_the_catalog():
    """docs/TASKS_AND_SUBTASKS.md must not drift from the code."""
    document = (ROOT / "docs/TASKS_AND_SUBTASKS.md").read_text(encoding="utf-8")
    for task in TASKS:
        assert task.directory_name in document, f"{task.directory_name} missing from docs"
        for subtask in task.subtasks:
            assert subtask.number in document
