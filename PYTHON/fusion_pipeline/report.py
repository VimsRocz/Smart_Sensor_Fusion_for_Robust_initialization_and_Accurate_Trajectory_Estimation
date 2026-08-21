"""Terminal tables summarising a run: datasets, tasks, subtasks and outputs.

Presentation only. Nothing here changes what the pipeline computes or writes;
the same facts are available machine-readably in ``manifest.json`` and
``figures_index.csv``.
"""

from __future__ import annotations

import os
import sys
import time
from pathlib import Path
from typing import Any, Iterable, Sequence

from .catalog import TASK_BY_NUMBER

#: Compact per-figure dataset codes, expanded in a legend under each run.
SOURCE_CODES = {"imu": "I", "gnss": "G", "truth": "T"}

REPORT_MODES = ("none", "summary", "full")


# ---------------------------------------------------------------------------
# Table rendering
# ---------------------------------------------------------------------------
def _truncate(text: Any, width: int) -> str:
    text = "" if text is None else str(text)
    if len(text) <= width:
        return text
    if width <= 1:
        return text[:width]
    return text[: width - 1] + "…"


def render_table(
    headers: Sequence[str],
    rows: Iterable[Sequence[Any]],
    widths: Sequence[int] | None = None,
    aligns: Sequence[str] | None = None,
    indent: str = "  ",
) -> str:
    """Render a box-drawn table, truncating cells to ``widths``."""
    rows = [[("" if cell is None else str(cell)) for cell in row] for row in rows]
    columns = len(headers)
    if widths is None:
        widths = [
            max(len(headers[index]), *(len(row[index]) for row in rows)) if rows else len(headers[index])
            for index in range(columns)
        ]
    aligns = aligns or ["<"] * columns

    def line(left: str, middle: str, right: str) -> str:
        return indent + left + middle.join("─" * (width + 2) for width in widths) + right

    def row_text(cells: Sequence[str]) -> str:
        rendered = [
            f" {_truncate(cell, width):{align}{width}} "
            for cell, width, align in zip(cells, widths, aligns)
        ]
        return indent + "│" + "│".join(rendered) + "│"

    out = [line("┌", "┬", "┐"), row_text(headers), line("├", "┼", "┤")]
    out.extend(row_text(row) for row in rows)
    out.append(line("└", "┴", "┘"))
    return "\n".join(out)


def relative(path: str | Path) -> str:
    """Path relative to the working directory when that is shorter."""
    try:
        candidate = os.path.relpath(str(path), Path.cwd())
    except ValueError:  # different drive on Windows
        return str(path)
    return candidate if len(candidate) < len(str(path)) else str(path)


def heading(text: str, width: int = 100) -> str:
    return f"\n{'═' * width}\n {text}\n{'═' * width}"


# ---------------------------------------------------------------------------
# Live progress
# ---------------------------------------------------------------------------
class Progress:
    """Streams task/subtask progress and per-subtask results while a run works.

    Every line is flushed immediately so a long full-dataset run shows where it
    is instead of going quiet for minutes. Disabled instances are no-ops, so
    call sites never need to guard.
    """

    #: Sensor roles in the order they are always listed.
    ROLES = ("imu", "gnss", "truth")

    def __init__(self, enabled: bool = True, stream: Any = None) -> None:
        self.enabled = bool(enabled)
        self.stream = stream if stream is not None else sys.stdout
        self.method = ""
        self.datasets: dict[str, dict[str, str] | None] = {}
        self._task_started: float | None = None
        self._task_number: int | None = None
        self._figures = 0

    def _write(self, text: str) -> None:
        if not self.enabled:
            return
        self.stream.write(text + "\n")
        self.stream.flush()

    # -- dataset naming -------------------------------------------------
    def dataset_name(self, role: str) -> str | None:
        entry = self.datasets.get(role)
        return entry["name"] if entry else None

    def _names_for(self, roles: Sequence[str]) -> str:
        names = [self.dataset_name(role) for role in self.ROLES if role in roles]
        present = [name for name in names if name]
        return " + ".join(present) if present else "—"

    def _codes_for(self, roles: Sequence[str]) -> str:
        codes = [
            SOURCE_CODES[role]
            for role in self.ROLES
            if role in roles and self.dataset_name(role)
        ]
        return "+".join(codes) if codes else "—"

    def _task_roles(self, number: int) -> list[str]:
        """Every sensor role the figures of this task draw on."""
        used = {role for figure in TASK_BY_NUMBER[number].figures for role in figure.sources}
        return [role for role in self.ROLES if role in used]

    # -- run ------------------------------------------------------------
    def run_begin(
        self,
        method: str,
        run_id: str,
        executed: Sequence[int],
        datasets: dict[str, dict[str, str] | None],
        run_dir: Any,
    ) -> None:
        self.method = method
        self.datasets = datasets
        width = 100
        self._write("\n" + "═" * width)
        self._write(
            f" RUNNING  method {method}   ·   run id {run_id}   ·   "
            f"tasks {min(executed)}-{max(executed)} ({len(executed)} of 7)"
        )
        self._write("═" * width)
        for role, label in (("imu", "IMU"), ("gnss", "GNSS"), ("truth", "Truth")):
            entry = datasets.get(role)
            if entry:
                self._write(
                    f"   {label:<6} {entry['name']:<24} {entry['path']:<44} {entry['size']}"
                )
            else:
                self._write(f"   {label:<6} {'—':<24} not supplied")
        self._write(f"   Output {relative(run_dir)}/")
        legend = "   Codes  " + " · ".join(
            f"{SOURCE_CODES[role]} = {self.dataset_name(role)}"
            for role in self.ROLES
            if self.dataset_name(role)
        )
        self._write(legend)

    def run_end(self, method: str, elapsed: float, figures: int) -> None:
        imu, gnss = self.dataset_name("imu"), self.dataset_name("gnss")
        self._write(
            f"─── {method} on {imu} + {gnss} finished in {elapsed:.1f} s"
            f" — {figures} figures ───"
        )

    # -- task -----------------------------------------------------------
    def task_begin(self, number: int) -> None:
        task = TASK_BY_NUMBER[number]
        self._task_started = time.perf_counter()
        self._task_number = number
        self._figures = 0
        self._write(f"\n▶ {self.method} · Task {number}/7 — {task.name}")
        self._write(
            f"    data: {self._names_for(self._task_roles(number))}"
            f"    →  {task.directory_name}/"
        )

    def subtask(self, number: str, detail: str = "") -> None:
        task = TASK_BY_NUMBER[self._task_number] if self._task_number else None
        name = task.subtask_map.get(number, "") if task else ""
        self._write(f"    {number}  {name}")
        if detail:
            self._write(f"         {detail}")

    def figure(
        self,
        subtask: str,
        slug: str,
        frame: str,
        status: str,
        sources: Sequence[str] = (),
    ) -> None:
        data = self._codes_for(sources)
        boxed = f"[{frame}]".ljust(12)
        if status == "written":
            self._figures += 1
            self._write(f"         ✎ {subtask}  {slug:<34} {boxed} {data}")
        elif status == "skipped":
            self._write(f"         · {subtask}  {slug:<34} {boxed} {data}  skipped")

    def task_end(self, number: int) -> None:
        elapsed = time.perf_counter() - (self._task_started or time.perf_counter())
        suffix = f"{self._figures} figures, " if self._figures else ""
        self._write(f"  ✔ {self.method} · Task {number} complete — {suffix}{elapsed:.2f} s")


#: Shared no-op used whenever progress reporting is switched off.
SILENT = Progress(enabled=False)


# ---------------------------------------------------------------------------
# Sections
# ---------------------------------------------------------------------------
def dataset_table(manifest: dict[str, Any], task1: dict[str, Any] | None) -> str:
    """Which files were read, under which dataset name, and how big they are."""
    inputs = manifest.get("inputs", {})
    tags = manifest.get("dataset_tags", {})
    validation = (task1 or {}).get("validation", {}) or {}
    rows: list[list[str]] = []
    for role, key in (("IMU", "imu"), ("GNSS", "gnss"), ("Truth", "truth")):
        path = inputs.get(key)
        if not path:
            rows.append([role, "—", "not supplied", "—"])
            continue
        info = validation.get(key) or {}
        if key == "imu":
            rate = info.get("sample_rate_hz")
            size = f"{info.get('rows', '?')} rows"
            if rate:
                size += f" @ {rate:.1f} Hz"
        elif key == "gnss":
            size = f"{info.get('rows', '?')} epochs"
        else:
            size = f"{info.get('rows', '?')} states"
        if info.get("duration_s") is not None:
            size += f", {info['duration_s']:.2f} s"
        rows.append([role, tags.get(key) or "—", relative(path), size])
    return render_table(
        ["Role", "Dataset name", "Input file", "Size"],
        rows,
        widths=[6, 22, 46, 26],
    )


def _source_code(sources: Sequence[str] | None) -> str:
    if not sources:
        return "—"
    return "+".join(SOURCE_CODES.get(source, "?") for source in sources)


def task_tables(records: Sequence[dict[str, Any]], run_dir: Path, executed: Sequence[int]) -> str:
    """One table per task: subtask names, figures, frames and status."""
    blocks: list[str] = []
    for number in executed:
        task = TASK_BY_NUMBER[number]
        task_records = [record for record in records if record["task"] == number]
        directory = Path(run_dir) / task.directory_name
        blocks.append(f"\n  Task {task.number} — {task.name}")
        blocks.append(f"  Saved in: {relative(directory)}/")
        if not task_records:
            blocks.append("    (no figures declared)")
            continue
        if all(record["status"] == "disabled" for record in task_records):
            blocks.append("    figures disabled (--no-plots); numeric artifacts only")
            for subtask in task.subtasks:
                blocks.append(f"    {subtask.number} — {subtask.name}")
            continue
        # Name each subtask once; repeating it on every figure row is noise.
        rows: list[list[str]] = []
        previous = None
        for record in task_records:
            same = record["subtask"] == previous
            previous = record["subtask"]
            rows.append(
                [
                    "" if same else record["subtask"],
                    "" if same else record["subtask_name"],
                    record["figure"],
                    record["coordinate_frame"],
                    _source_code(record.get("sources")),
                    record["status"],
                ]
            )
        blocks.append(
            render_table(
                ["Subtask", "Subtask name", "Figure", "Frame", "Data", "Status"],
                rows,
                widths=[7, 44, 32, 9, 7, 8],
                indent="    ",
            )
        )
    return "\n".join(blocks)


def task_summary_table(records: Sequence[dict[str, Any]], run_dir: Path, executed: Sequence[int]) -> str:
    """One row per task: name, subtask count, figure counts and folder."""
    rows: list[list[str]] = []
    for number in executed:
        task = TASK_BY_NUMBER[number]
        task_records = [record for record in records if record["task"] == number]
        written = sum(1 for record in task_records if record["status"] == "written")
        skipped = sum(1 for record in task_records if record["status"] == "skipped")
        figures = f"{written}"
        if skipped:
            figures += f" (+{skipped} skipped)"
        rows.append(
            [
                str(task.number),
                task.name,
                ", ".join(item.number for item in task.subtasks),
                figures,
                task.directory_name + "/",
            ]
        )
    return render_table(
        ["Task", "Task name", "Subtasks", "Figures", "Sub-folder of the run folder"],
        rows,
        widths=[4, 40, 22, 16, 32],
    )


def metrics_table(metrics: dict[str, Any]) -> str:
    numeric = [(key, value) for key, value in metrics.items() if isinstance(value, (int, float))]
    if not numeric:
        return "  (no scalar metrics)"
    return render_table(
        ["Metric", "Value"],
        [[key, f"{value:.6g}"] for key, value in numeric],
        widths=[32, 20],
        aligns=["<", ">"],
    )


def comparison_metrics_table(methods: Sequence[str], metrics: dict[str, dict[str, Any]]) -> str:
    keys = sorted(
        {
            key
            for values in metrics.values()
            for key, value in values.items()
            if isinstance(value, (int, float))
        }
    )
    if not keys:
        return "  (no comparable metrics)"
    rows = [
        [key] + [
            f"{metrics[method][key]:.6g}" if key in metrics.get(method, {}) else "—"
            for method in methods
        ]
        for key in keys
    ]
    return render_table(
        ["Metric", *methods],
        rows,
        widths=[28] + [18] * len(methods),
        aligns=["<"] + [">"] * len(methods),
    )


def output_tree(run_dir: Path, records: Sequence[dict[str, Any]], executed: Sequence[int]) -> str:
    """The folder structure this run created, with per-task figure counts."""
    lines = [f"  {relative(run_dir)}/"]
    lines.append("  ├── manifest.json            run configuration, timings and metrics")
    lines.append("  ├── figures_index.csv        every figure with task, subtask, frame and datasets")
    lines.append("  ├── figures_index.json       the same index, machine-readable")
    for position, number in enumerate(executed):
        task = TASK_BY_NUMBER[number]
        written = sum(
            1 for record in records if record["task"] == number and record["status"] == "written"
        )
        connector = "└──" if position == len(executed) - 1 else "├──"
        lines.append(f"  {connector} {task.directory_name}/".ljust(48) + f"{written} figures + task data")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Whole-run reports
# ---------------------------------------------------------------------------
def run_report(result: dict[str, Any], mode: str = "full") -> str:
    """Full terminal report for one method run."""
    if mode == "none":
        return ""
    manifest = result["manifest"]
    records = result.get("figures", [])
    executed = manifest.get("executed_tasks", [])
    run_dir = Path(result["run_dir"])
    tasks = result.get("tasks", {})

    out = [heading(f"RUN COMPLETE — method {manifest['method']} — run id {manifest['run_id']}")]
    out.append("\n  Input datasets")
    out.append(dataset_table(manifest, tasks.get(1)))

    out.append(f"\n  Run folder: {relative(run_dir)}/")
    out.append("\n  Tasks executed")
    out.append(task_summary_table(records, run_dir, executed))
    requested = manifest.get("requested_tasks", [])
    auto = manifest.get("auto_dependencies", [])
    if auto:
        out.append(
            f"  Requested tasks {requested}; {auto} added automatically as dependencies."
        )

    if mode == "full":
        out.append("\n  Figures by task and subtask")
        out.append(task_tables(records, run_dir, executed))
        out.append(
            "\n    Data codes: I = IMU, G = GNSS, T = Truth."
            "\n    Filenames follow"
            " <run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png"
        )

    out.append("\n  Output structure")
    out.append(output_tree(run_dir, records, executed))

    if "metrics" in manifest:
        out.append("\n  Task 7 metrics")
        out.append(metrics_table(manifest["metrics"]))

    figures = manifest.get("figures", {})
    out.append(
        f"\n  {figures.get('written', 0)} figures written"
        f", {figures.get('skipped', 0)} skipped"
        f" — index: {relative(figures.get('index_csv', ''))}"
    )
    if manifest.get("elapsed_s") is not None:
        out.append(f"  Elapsed: {manifest['elapsed_s']:.2f} s")
    return "\n".join(out)


def comparison_report(result: dict[str, Any], mode: str = "full") -> str:
    """Terminal report for the cross-method comparison folder."""
    if mode == "none":
        return ""
    comparison = result["comparison"]
    directory = Path(result["comparison_dir"])
    methods = comparison["methods"]

    out = [heading(f"CROSS-METHOD COMPARISON — {' + '.join(methods)}")]
    out.append("\n  Method run folders")
    out.append(
        render_table(
            ["Method", "Run folder"],
            [[method, relative(path) + "/"] for method, path in comparison["run_directories"].items()],
            widths=[14, 78],
        )
    )
    out.append("\n  Task 7 metrics by method")
    out.append(comparison_metrics_table(methods, comparison["metrics"]))

    records = result.get("figures", [])
    if mode == "full" and records:
        out.append(f"\n  Comparison figures — saved in: {relative(directory)}/")
        out.append(
            render_table(
                ["ID", "Figure", "Frame", "Data", "Status"],
                [
                    [
                        record["subtask"],
                        record["figure"],
                        record["coordinate_frame"],
                        _source_code(record.get("sources")),
                        record["status"],
                    ]
                    for record in records
                ],
                widths=[5, 34, 10, 7, 16],
                indent="    ",
            )
        )
    out.append(
        f"\n  Comparison folder: {relative(directory)}/"
        f"\n    method_comparison.csv / .json   metrics table for all methods"
        f"\n    figures_index.csv / .json       every comparison figure"
    )
    return "\n".join(out)
