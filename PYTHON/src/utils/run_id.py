"""Python counterpart to ``MATLAB/src/utils/run_id.m``."""

from pathlib import Path


def run_id(imu_path: str, gnss_path: str, method: str) -> str:
    """Return a consistent run label like ``TRIAD_IMU_X002_GNSS_X002``.

    Method first, then IMU, then GNSS, matching naming.make_tag.
    """

    imu_tag = Path(imu_path).name.upper().replace(".DAT", "")
    gnss_tag = Path(gnss_path).name.upper().replace(".CSV", "")
    return f"{method.upper()}_{imu_tag}_{gnss_tag}"


__all__ = ["run_id"]

