from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
import run_release

from run_release import (
    Combination,
    InputValidationError,
    apply_matlab_policy,
    build_command,
    bundled_combinations,
    export_native_figures,
    find_matlab,
    inspect_gnss,
    inspect_imu,
    inspect_truth,
    prepare_deferred_fig_bundle,
    validate_alignment,
)


def _write_imu(path: Path, rows: int = 12) -> None:
    lines = []
    for index in range(rows):
        time_in_second = ((index + 1) % 4) * 0.25
        lines.append(
            f"{index % 256} {time_in_second:.2f} 0 0 0 0.1 0.0 0.0 22.5 1"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_gnss(path: Path) -> None:
    path.write_text(
        "Posix_Time,X_ECEF_m,Y_ECEF_m,Z_ECEF_m,"
        "VX_ECEF_mps,VY_ECEF_mps,VZ_ECEF_mps\n"
        "100,6378137,0,0,0,0,0\n"
        "101,6378137,0,0,0,0,0\n"
        "102,6378137,0,0,0,0,0\n",
        encoding="utf-8",
    )


def _write_truth(path: Path) -> None:
    lines = [
        f"{index} {index / 10:.1f} 6378137 0 0 0 0 0 0 0 0 1"
        for index in range(21)
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_bundled_cross_product_contains_exactly_18_unique_combinations() -> None:
    combinations = bundled_combinations()
    identities = {(item.imu_id, item.gnss_id, item.method) for item in combinations}
    assert len(combinations) == 18
    assert len(identities) == 18


def test_preflight_accepts_different_rates_with_matching_coverage(tmp_path: Path) -> None:
    imu_path = tmp_path / "IMU_CUSTOM.dat"
    gnss_path = tmp_path / "GNSS_CUSTOM.csv"
    truth_path = tmp_path / "STATE_CUSTOM.txt"
    _write_imu(imu_path)
    _write_gnss(gnss_path)
    _write_truth(truth_path)

    imu = inspect_imu(imu_path)
    gnss = inspect_gnss(gnss_path)
    truth = inspect_truth(truth_path)

    assert imu.rows == 12
    assert imu.sample_rate_hz == pytest.approx(4.0)
    assert gnss.rows == 3
    assert gnss.sample_rate_hz == pytest.approx(1.0)
    assert "about 4.000 IMU samples/GNSS epoch" in validate_alignment(
        imu, gnss, truth
    )


def test_preflight_rejects_incompatible_time_coverage(tmp_path: Path) -> None:
    imu_path = tmp_path / "short.dat"
    gnss_path = tmp_path / "GNSS_CUSTOM.csv"
    _write_imu(imu_path, rows=4)
    _write_gnss(gnss_path)

    with pytest.raises(InputValidationError, match="coverage mismatch"):
        validate_alignment(inspect_imu(imu_path), inspect_gnss(gnss_path))


def test_cross_pair_command_explicitly_allows_truth_tag_mismatch() -> None:
    combination = Combination(
        "x003",
        "x001",
        "SVD",
        Path("IMU_X003.dat"),
        Path("GNSS_X001.csv"),
    )
    command = build_command(combination, Path("STATE_X001.txt"))
    assert "--allow-truth-mismatch" in command
    assert command[command.index("--method") + 1] == "SVD"


def test_explicit_matlab_executable_is_detected(tmp_path: Path) -> None:
    executable = tmp_path / "matlab"
    executable.write_text("#!/bin/sh\n", encoding="utf-8")
    executable.chmod(0o755)
    assert find_matlab(str(executable)) == executable.resolve()


def test_missing_matlab_requires_explicit_deferred_mode() -> None:
    with pytest.raises(InputValidationError, match="MATLAB was not found"):
        apply_matlab_policy(None, plots_enabled=True, defer_fig=False)


def test_deferred_mode_warns_and_can_continue(capsys) -> None:
    apply_matlab_policy(None, plots_enabled=True, defer_fig=True)
    assert "does not rerun fusion" in capsys.readouterr().err


@pytest.mark.parametrize(
    "target", ["release", "release-combo", "release-18", "release-custom"]
)
def test_release_make_targets_allow_missing_local_matlab(target: str) -> None:
    completed = subprocess.run(
        ["make", "-n", target],
        cwd=run_release.ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    assert "--defer-fig" in completed.stdout


def test_deferred_bundle_is_self_contained(tmp_path: Path) -> None:
    prepare_deferred_fig_bundle(tmp_path)
    assert (tmp_path / "export_release_figures.m").is_file()
    instructions = (tmp_path / "CREATE_NATIVE_FIGS.txt").read_text(encoding="utf-8")
    assert "export_release_figures(pwd)" in instructions


def test_native_fig_export_audits_every_png(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    (tmp_path / "one.png").write_bytes(b"png")
    nested = tmp_path / "nested"
    nested.mkdir()
    (nested / "two.png").write_bytes(b"png")

    def fake_matlab_run(*_args, **_kwargs):
        for png in tmp_path.rglob("*.png"):
            png.with_suffix(".fig").write_bytes(b"native-fig")

    monkeypatch.setattr(run_release.subprocess, "run", fake_matlab_run)
    assert export_native_figures(tmp_path, Path("/fake/matlab")) == 2
