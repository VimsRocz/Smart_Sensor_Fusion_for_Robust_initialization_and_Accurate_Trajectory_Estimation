"""MATLAB discovery and native editable FIG batch export."""

from __future__ import annotations

import csv
import json
import os
import shutil
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
MATLAB_ROOT = REPO_ROOT / "MATLAB"


def find_matlab(explicit: str | Path | None = None) -> Path | None:
    """Locate MATLAB on macOS, Linux or Windows."""
    candidates: list[Path] = []
    configured = explicit or os.environ.get("MATLAB_BIN")
    if configured:
        candidates.append(Path(configured).expanduser())
    discovered = shutil.which("matlab")
    if discovered:
        candidates.append(Path(discovered))

    for base in (Path("/Applications"), Path.home() / "Applications"):
        if base.is_dir():
            candidates.extend(sorted(base.glob("MATLAB*.app/bin/matlab"), reverse=True))
    candidates.extend(sorted(Path("/usr/local/MATLAB").glob("R*/bin/matlab"), reverse=True))
    for variable in ("ProgramFiles", "ProgramFiles(x86)"):
        base = os.environ.get(variable)
        if base:
            candidates.extend(
                sorted(Path(base).glob("MATLAB/R*/bin/matlab.exe"), reverse=True)
            )

    return next(
        (
            candidate.resolve()
            for candidate in candidates
            if candidate.is_file() and os.access(candidate, os.X_OK)
        ),
        None,
    )


def _matlab_string(path: Path) -> str:
    return str(path).replace("'", "''")


def _find_artifact(index_dir: Path, record: dict[str, object], suffix: str) -> Path | None:
    filename = record.get("filename")
    if not isinstance(filename, str) or not filename:
        return None
    artifact_name = Path(filename).with_suffix(suffix).name
    candidates = [
        Path(str(record.get("path", ""))).with_suffix(suffix),
        index_dir / artifact_name,
        index_dir / str(record.get("task_directory", "")) / artifact_name,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    return next(index_dir.rglob(artifact_name), None)


def _refresh_json_index(index_path: Path) -> tuple[int, int]:
    payload = json.loads(index_path.read_text(encoding="utf-8"))
    records = payload.get("figures", [])
    if not isinstance(records, list):
        return 0, 0
    written = 0
    deferred = 0
    for record in records:
        if not isinstance(record, dict) or record.get("status") != "written":
            continue
        fig_path = _find_artifact(index_path.parent, record, ".fig")
        artifacts = record.setdefault("artifacts", {})
        if not isinstance(artifacts, dict):
            artifacts = {}
            record["artifacts"] = artifacts
        if fig_path is not None:
            artifacts["fig"] = fig_path.name
            artifacts["fig_status"] = "written"
            record["fig_filename"] = fig_path.name
            record["fig_status"] = "written"
            written += 1
        else:
            artifacts["fig_status"] = "deferred"
            record["fig_status"] = "deferred"
            deferred += 1
    payload["native_figures_written"] = written
    payload["native_figures_deferred"] = deferred
    index_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    return written, deferred


def _refresh_csv_index(index_path: Path) -> None:
    with index_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = list(reader)
        fieldnames = list(reader.fieldnames or [])
    for field in ("fig_filename", "fig_status"):
        if field not in fieldnames:
            fieldnames.append(field)
    for row in rows:
        if row.get("status") != "written":
            continue
        fig_path = _find_artifact(index_path.parent, row, ".fig")
        row["fig_filename"] = fig_path.name if fig_path else ""
        row["fig_status"] = "written" if fig_path else "deferred"
    with index_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _refresh_summary(path: Path, written: int, deferred: int) -> None:
    if not path.is_file():
        return
    payload = json.loads(path.read_text(encoding="utf-8"))
    figures = payload.get("figures")
    if isinstance(figures, dict):
        figures["native_figures_written"] = written
        figures["native_figures_deferred"] = deferred
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def refresh_figure_indexes(results_dir: Path) -> None:
    """Update artifact indexes after deferred FIG files are created."""
    for index_path in results_dir.rglob("figures_index.json"):
        written, deferred = _refresh_json_index(index_path)
        csv_path = index_path.with_suffix(".csv")
        if csv_path.is_file():
            _refresh_csv_index(csv_path)
        _refresh_summary(index_path.parent / "manifest.json", written, deferred)
        _refresh_summary(index_path.parent / "method_comparison.json", written, deferred)


def export_native_figures(results_dir: Path, matlab_binary: Path) -> int:
    """Create data-editable FIG companions and audit every pipeline PNG."""
    results_dir = results_dir.expanduser().resolve()
    results_dir.mkdir(parents=True, exist_ok=True)
    expression = (
        f"addpath('{_matlab_string(MATLAB_ROOT)}'); "
        f"export_release_figures('{_matlab_string(results_dir)}');"
    )
    try:
        subprocess.run(
            [str(matlab_binary), "-batch", expression],
            cwd=REPO_ROOT,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"MATLAB editable FIG export failed: {exc}") from exc
    pngs = sorted(results_dir.rglob("*.png"))
    missing = [png for png in pngs if not png.with_suffix(".fig").is_file()]
    if missing:
        preview = "\n".join(f"  {path}" for path in missing[:10])
        raise RuntimeError(
            f"MATLAB FIG audit failed: {len(missing)} of {len(pngs)} PNG files "
            f"have no native .fig companion.\n{preview}"
        )
    refresh_figure_indexes(results_dir)
    print(
        f"MATLAB FIG audit: {len(pngs)}/{len(pngs)} PNG files have native "
        ".fig companions"
    )
    return len(pngs)
