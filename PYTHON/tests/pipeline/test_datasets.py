"""Dataset selection must resolve the verified full and short pairings."""

import json
from pathlib import Path

import pytest

import fusion_pipeline.cli as cli
from fusion_pipeline.cli import main
from fusion_pipeline.datasets import BUNDLED, describe_datasets, resolve_dataset

ROOT = Path(__file__).resolve().parents[3]
SMALL_IMU = ROOT / "DATA/IMU/IMU_X001_small.dat"
SMALL_GNSS = ROOT / "DATA/GNSS/GNSS_X001_small.csv"
SMALL_TRUTH = ROOT / "DATA/Truth/STATE_X001_small.txt"


def test_registry_holds_full_and_short_versions_of_each_dataset():
    assert set(BUNDLED) == {
        "x001",
        "x002",
        "x003",
        "x001_small",
        "x002_small",
        "x003_small",
    }


def test_every_bundled_dataset_points_at_files_that_exist():
    for name, dataset in BUNDLED.items():
        assert dataset.missing() == [], f"{name} references missing files"


def test_x003_is_paired_with_the_x002_gnss():
    """X003 has no GNSS of its own; pairing it with X001 would be wrong."""
    dataset = resolve_dataset("x003")
    assert dataset.imu.endswith("IMU_X003.dat")
    assert dataset.gnss.endswith("GNSS_X002.csv")
    assert dataset.truth is None


def test_x003_small_is_paired_with_the_x002_small_gnss():
    dataset = resolve_dataset("x003_small")
    assert dataset.imu.endswith("IMU_X003_small.dat")
    assert dataset.gnss.endswith("GNSS_X002_small.csv")
    assert dataset.truth is None


def test_x002_pairs_with_its_own_gnss_and_has_no_truth():
    dataset = resolve_dataset("x002")
    assert dataset.imu.endswith("IMU_X002.dat")
    assert dataset.gnss.endswith("GNSS_X002.csv")
    assert dataset.truth is None


def test_only_x001_variants_carry_a_reference_trajectory():
    assert {name for name, d in BUNDLED.items() if d.truth} == {"x001", "x001_small"}


def test_unknown_dataset_lists_the_available_names():
    with pytest.raises(ValueError, match="x003"):
        resolve_dataset("x004")


def test_dataset_listing_names_each_pairing():
    text = describe_datasets()
    for name in BUNDLED:
        assert name in text
    assert "IMU_X003.dat" in text and "GNSS_X002.csv" in text


def test_cli_runs_one_method_only(tmp_path, capsys):
    code = main(
        [
            "--imu", str(SMALL_IMU),
            "--gnss", str(SMALL_GNSS),
            "--method", "TRIAD",
            "--tasks", "1-7",
            "--output", str(tmp_path),
            "--run-id", "one",
            "--no-plots", "--progress", "off", "--report", "none",
        ]
    )
    assert code == 0
    out = capsys.readouterr().out
    assert "TRIAD complete" in out
    assert "Davenport" not in out and "SVD complete" not in out
    # A single-method run must not produce a cross-method comparison folder.
    assert not (tmp_path / "one" / "comparison").exists()


def test_cli_runs_all_methods_with_a_comparison(tmp_path):
    code = main(
        [
            "--imu", str(SMALL_IMU),
            "--gnss", str(SMALL_GNSS),
            "--method", "ALL",
            "--tasks", "1-3",
            "--output", str(tmp_path),
            "--run-id", "every",
            "--no-plots", "--progress", "off", "--report", "none",
        ]
    )
    assert code == 0
    for method in ("triad", "davenport", "svd"):
        assert (tmp_path / "every" / method / "manifest.json").is_file()
    assert (tmp_path / "every" / "comparison" / "method_comparison.json").is_file()


def test_no_truth_flag_drops_a_reference_that_was_supplied(tmp_path):
    code = main(
        [
            "--imu", str(SMALL_IMU),
            "--gnss", str(SMALL_GNSS),
            "--truth", str(SMALL_TRUTH),
            "--no-truth",
            "--method", "SVD",
            "--tasks", "1-7",
            "--output", str(tmp_path),
            "--run-id", "notruth",
            "--no-plots", "--progress", "off", "--report", "none",
        ]
    )
    assert code == 0
    manifest = json.loads((tmp_path / "notruth" / "svd" / "manifest.json").read_text())
    assert manifest["inputs"]["truth"] is None
    task6 = json.loads(
        (tmp_path / "notruth" / "svd" / "task_06_truth_overlay" / "summary.json").read_text()
    )
    assert task6["status"] == "skipped"


def test_explicit_paths_override_the_dataset(tmp_path):
    """--imu/--gnss win over --dataset so a one-off pairing is still possible."""
    code = main(
        [
            "--dataset", "x003",
            "--imu", str(SMALL_IMU),
            "--gnss", str(SMALL_GNSS),
            "--method", "TRIAD",
            "--tasks", "2",
            "--output", str(tmp_path),
            "--run-id", "override",
            "--no-plots", "--progress", "off", "--report", "none",
        ]
    )
    assert code == 0
    manifest = json.loads((tmp_path / "override" / "triad" / "manifest.json").read_text())
    assert manifest["inputs"]["imu"].endswith("IMU_X001_small.dat")
    assert manifest["inputs"]["gnss"].endswith("GNSS_X001_small.csv")


def test_running_with_no_inputs_at_all_explains_the_options(capsys):
    with pytest.raises(SystemExit):
        main(["--method", "TRIAD"])
    assert "--dataset" in capsys.readouterr().err


def test_fig_on_fails_before_pipeline_when_matlab_is_missing(monkeypatch, capsys):
    pipeline_called = False

    def unexpected_pipeline(*_args, **_kwargs):
        nonlocal pipeline_called
        pipeline_called = True

    monkeypatch.setattr(cli, "find_matlab", lambda _explicit=None: None)
    monkeypatch.setattr(cli, "run_pipeline", unexpected_pipeline)
    code = main(
        [
            "--imu", str(SMALL_IMU),
            "--gnss", str(SMALL_GNSS),
            "--method", "TRIAD",
            "--tasks", "1",
            "--fig", "on",
        ]
    )
    assert code == 2
    assert not pipeline_called
    assert "MATLAB was not found" in capsys.readouterr().err


def test_fig_export_runs_for_completed_pipeline(monkeypatch, tmp_path):
    matlab = tmp_path / "matlab"
    exported: list[tuple[Path, Path]] = []
    monkeypatch.setattr(
        cli,
        "export_native_figures",
        lambda root, binary: exported.append((root, binary)),
    )
    cli._finish_fig_export(tmp_path / "run", matlab, "on")
    assert exported == [(tmp_path / "run", matlab)]
