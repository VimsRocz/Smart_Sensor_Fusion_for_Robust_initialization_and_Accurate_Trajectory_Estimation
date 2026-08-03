"""A differently formatted log must be readable through configuration alone."""

from pathlib import Path

import numpy as np
import pytest

from fusion_pipeline.contracts import (
    GnssLayout,
    ImuLayout,
    InputContractError,
    TruthLayout,
    load_gnss,
    load_imu,
    load_truth,
)
from fusion_pipeline.pipeline import PipelineConfig, parse_methods

ROOT = Path(__file__).resolve().parents[3]
IMU = ROOT / "DATA/IMU/IMU_X001_small.dat"
GNSS = ROOT / "DATA/GNSS/GNSS_X001_small.csv"
TRUTH = ROOT / "DATA/Truth/STATE_X001_small.txt"


def test_reordered_imu_columns_produce_identical_rates(tmp_path):
    reference = load_imu(IMU)
    raw = np.loadtxt(IMU)
    # Same data, columns shuffled to: time, accel, gyro (no counter column).
    reordered = np.column_stack([raw[:, 1], raw[:, 5:8], raw[:, 2:5]])
    path = tmp_path / "reordered.dat"
    np.savetxt(path, reordered)

    loaded = load_imu(
        path,
        ImuLayout(time_column=0, accel_columns=(1, 2, 3), gyro_columns=(4, 5, 6)),
    )
    assert np.allclose(loaded.gyro_rps, reference.gyro_rps)
    assert np.allclose(loaded.accel_mps2, reference.accel_mps2)


def test_degree_and_g_units_are_converted_to_si(tmp_path):
    reference = load_imu(IMU)
    raw = np.loadtxt(IMU)
    converted = raw.copy()
    converted[:, 2:5] = np.degrees(raw[:, 2:5])
    converted[:, 5:8] = raw[:, 5:8] / 9.80665
    path = tmp_path / "deg_g.dat"
    np.savetxt(path, converted)

    loaded = load_imu(path, ImuLayout(gyro_unit="deg", accel_unit="g"))
    assert np.allclose(loaded.gyro_rps, reference.gyro_rps, rtol=1e-10, atol=1e-14)
    assert np.allclose(loaded.accel_mps2, reference.accel_mps2, rtol=1e-10, atol=1e-12)


def test_millisecond_imu_clock_is_scaled_before_rate_conversion(tmp_path):
    reference = load_imu(IMU)
    raw = np.loadtxt(IMU)
    milliseconds = raw.copy()
    milliseconds[:, 1] = raw[:, 1] * 1000.0
    path = tmp_path / "ms.dat"
    np.savetxt(path, milliseconds)

    loaded = load_imu(path, ImuLayout(time_unit="ms"))
    assert np.isclose(loaded.dt_s, reference.dt_s)
    assert np.allclose(loaded.time_s, reference.time_s)


def test_gnss_header_overrides_and_semicolon_delimiter(tmp_path):
    reference = load_gnss(GNSS)
    path = tmp_path / "custom.csv"
    lines = ["t;px;py;pz;vx_;vy_;vz_"]
    for index in range(reference.rows):
        values = [
            float(reference.time_s[index]),
            *(float(v) for v in reference.position_ecef_m[index]),
            *(float(v) for v in reference.velocity_ecef_mps[index]),
        ]
        lines.append(";".join(repr(value) for value in values))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    loaded = load_gnss(
        path,
        GnssLayout(
            delimiter=";",
            column_overrides={
                "time": "t", "x": "px", "y": "py", "z": "pz",
                "vx": "vx_", "vy": "vy_", "vz": "vz_",
            },
        ),
    )
    assert loaded.rows == reference.rows
    assert np.allclose(loaded.position_ecef_m, reference.position_ecef_m)
    assert np.allclose(loaded.velocity_ecef_mps, reference.velocity_ecef_mps)


def test_gnss_override_naming_a_missing_header_is_reported(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("time,x,y,z,vx,vy,vz\n0,7e6,0,0,0,0,0\n1,7e6,1,0,0,0,0\n", encoding="utf-8")
    with pytest.raises(InputContractError, match="not a header in the file"):
        load_gnss(path, GnssLayout(column_overrides={"time": "no_such_column"}))


def test_truth_without_quaternion_columns_loads_position_only():
    truth = load_truth(TRUTH, TruthLayout(quaternion_columns=None))
    assert truth.quaternion_wxyz is None
    assert truth.position_ecef_m.shape[1] == 3


def test_wxyz_declaration_changes_the_stored_quaternion_order():
    as_xyzw = load_truth(TRUTH, TruthLayout(quaternion_order="xyzw"))
    as_wxyz = load_truth(TRUTH, TruthLayout(quaternion_order="wxyz"))
    assert not np.allclose(as_xyzw.quaternion_wxyz, as_wxyz.quaternion_wxyz)


def test_too_few_columns_names_the_key_to_change(tmp_path):
    path = tmp_path / "short.dat"
    np.savetxt(path, np.tile(np.arange(4.0), (5, 1)))
    with pytest.raises(InputContractError, match="imu_gyro_columns"):
        load_imu(path)


def test_unknown_configuration_key_lists_the_accepted_keys():
    with pytest.raises(ValueError, match="imu_gyro_columns"):
        PipelineConfig.from_mapping({"imu_gyro_colums": [2, 3, 4]})


def test_config_builds_layouts_that_match_the_declared_values():
    cfg = PipelineConfig.from_mapping(
        {
            "imu_time_column": 0,
            "imu_gyro_columns": [4, 5, 6],
            "imu_accel_columns": [1, 2, 3],
            "imu_accel_unit": "g",
            "gnss_delimiter": ";",
            "truth_quaternion_columns": None,
        }
    )
    imu_layout = cfg.imu_layout()
    assert imu_layout.time_column == 0
    assert imu_layout.gyro_columns == (4, 5, 6)
    assert imu_layout.accel_unit == "g"
    assert cfg.gnss_layout().delimiter == ";"
    assert cfg.truth_layout().quaternion_columns is None


@pytest.mark.parametrize(
    "selection,expected",
    [
        ("TRIAD", ["TRIAD"]),
        ("svd", ["SVD"]),
        ("ALL", ["TRIAD", "Davenport", "SVD"]),
        ("all", ["TRIAD", "Davenport", "SVD"]),
        ("SVD,TRIAD", ["TRIAD", "SVD"]),
        (["davenport", "SVD"], ["Davenport", "SVD"]),
    ],
)
def test_method_selection_accepts_one_a_subset_or_all(selection, expected):
    assert parse_methods(selection) == expected


@pytest.mark.parametrize("selection", ["", "TRIAD,TRIAD", "ALL,TRIAD", "QUEST"])
def test_invalid_method_selections_are_rejected(selection):
    with pytest.raises(ValueError):
        parse_methods(selection)
