"""The bundled datasets and their verified IMU/GNSS pairings.

Selecting a dataset by name avoids the main way of getting this wrong: X003 has
no GNSS file of its own and must be paired with ``GNSS_X002.csv``. Pairing it
with the noise-free ``GNSS_X001.csv`` would combine a biased IMU with perfect
GNSS, which is not a combination the data was built for.

See ``DATA/README.md`` for what each run number means.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

#: Repository root: PYTHON/fusion_pipeline/datasets.py -> up three levels.
REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class Dataset:
    name: str
    imu: str
    gnss: str
    truth: str | None
    description: str

    def resolve(self, root: Path | None = None) -> dict[str, str | None]:
        base = Path(root) if root is not None else REPO_ROOT
        return {
            "imu": str(base / self.imu),
            "gnss": str(base / self.gnss),
            "truth": str(base / self.truth) if self.truth else None,
        }

    def missing(self, root: Path | None = None) -> list[str]:
        base = Path(root) if root is not None else REPO_ROOT
        wanted = [self.imu, self.gnss] + ([self.truth] if self.truth else [])
        return [name for name in wanted if not (base / name).is_file()]


BUNDLED: dict[str, Dataset] = {
    "x001": Dataset(
        "x001",
        "DATA/IMU/IMU_X001.dat",
        "DATA/GNSS/GNSS_X001.csv",
        "DATA/Truth/STATE_X001.txt",
        "Noise-free baseline. The only run with a reference trajectory.",
    ),
    "x002": Dataset(
        "x002",
        "DATA/IMU/IMU_X002.dat",
        "DATA/GNSS/GNSS_X002.csv",
        None,
        "X001 plus zero-mean IMU and GNSS noise. No truth.",
    ),
    "x003": Dataset(
        "x003",
        "DATA/IMU/IMU_X003.dat",
        "DATA/GNSS/GNSS_X002.csv",
        None,
        "X002 plus a constant IMU bias. Reuses the X002 GNSS. No truth.",
    ),
}


def resolve_dataset(name: str, root: Path | None = None) -> Dataset:
    key = str(name).strip().lower()
    if key not in BUNDLED:
        raise ValueError(
            f"Unknown dataset {name!r}. Available: {', '.join(sorted(BUNDLED))}"
        )
    dataset = BUNDLED[key]
    missing = dataset.missing(root)
    if missing:
        raise ValueError(
            f"Dataset {key!r} is missing files: {', '.join(missing)}"
        )
    return dataset


def describe_datasets(root: Path | None = None) -> str:
    """Render the dataset table for ``--list-datasets``."""
    lines = [
        "Bundled datasets  (use with --dataset NAME)",
        "",
        f"  {'Name':<12} {'IMU':<28} {'GNSS':<28} {'Truth':<24} Notes",
        f"  {'-' * 12} {'-' * 28} {'-' * 28} {'-' * 24} {'-' * 50}",
    ]
    for key in sorted(BUNDLED):
        dataset = BUNDLED[key]
        truth = Path(dataset.truth).name if dataset.truth else "— none —"
        flag = "" if not dataset.missing(root) else "   [FILES MISSING]"
        lines.append(
            f"  {dataset.name:<12} {Path(dataset.imu).name:<28} "
            f"{Path(dataset.gnss).name:<28} {truth:<24} {dataset.description}{flag}"
        )
    lines += [
        "",
        "X003 has no GNSS file of its own; it is paired with GNSS_X002 by design.",
        "Only X001 ships a reference trajectory. Runs without truth complete Tasks 1-5,",
        "record Task 6 as skipped, and report GNSS innovation statistics in Task 7.",
        "",
        "Add --no-truth to ignore the reference even when the dataset provides one.",
    ]
    return "\n".join(lines)
