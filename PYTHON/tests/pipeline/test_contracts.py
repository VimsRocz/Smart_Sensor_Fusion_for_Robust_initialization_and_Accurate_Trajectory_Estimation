from pathlib import Path

import numpy as np
import pytest

from fusion_pipeline.contracts import InputContractError, load_gnss, load_imu, load_truth
from fusion_pipeline.pipeline import parse_tasks

ROOT = Path(__file__).resolve().parents[3]


def test_bundled_small_inputs_satisfy_contracts():
    imu = load_imu(ROOT / "DATA/IMU/IMU_X001_small.dat")
    gnss = load_gnss(ROOT / "DATA/GNSS/GNSS_X001_small.csv")
    truth = load_truth(ROOT / "DATA/Truth/STATE_X001_small.txt")
    assert imu.rows == 1000
    assert np.isclose(imu.dt_s, 0.0025)
    assert imu.clock_wraps > 0
    assert gnss.rows == 9
    assert truth.quaternion_wxyz.shape == (99, 4)


def test_task_selection_expands_upstream_dependencies():
    requested, expanded = parse_tasks("3,5")
    assert requested == [3, 5]
    assert expanded == [1, 2, 3, 4, 5]


def test_bad_gnss_header_reports_contract_error(tmp_path):
    path = tmp_path / "bad.csv"
    path.write_text("time,x,y\n0,1,2\n1,2,3\n", encoding="utf-8")
    with pytest.raises(InputContractError, match="missing"):
        load_gnss(path)
