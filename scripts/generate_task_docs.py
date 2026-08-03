#!/usr/bin/env python3
"""Regenerate docs/TASKS_AND_SUBTASKS.md from the pipeline catalog.

The catalog in ``fusion_pipeline/catalog.py`` drives directory names, JSON
summaries and figure filenames. Generating the document from the same source
keeps the documentation from drifting; ``test_documented_task_table_matches_the_catalog``
fails if this script has not been re-run after a catalog change.

    .venv/bin/python scripts/generate_task_docs.py
"""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "PYTHON"))

from fusion_pipeline.catalog import (  # noqa: E402
    COMPARISON_FIGURES,
    COMPARISON_SLUG,
    FRAMES,
    TASKS,
)

OUTPUT = ROOT / "docs/TASKS_AND_SUBTASKS.md"

PREAMBLE = """# Tasks, subtasks, and execution model

<!-- GENERATED FILE. Edit PYTHON/fusion_pipeline/catalog.py, then run:
     .venv/bin/python scripts/generate_task_docs.py -->

Every task, subtask, output directory and figure below is generated from
`PYTHON/fusion_pipeline/catalog.py`, which is also what the pipeline itself
reads at run time. Print the same information at any time with:

```bash
.venv/bin/python PYTHON/run_pipeline.py --list-tasks
```

## Dependency rule

Tasks form a strict prefix: `1 → 2 → 3 → 4 → 5 → 6 → 7`. Requesting a
downstream task automatically runs and saves every prerequisite. `--tasks 4`
means "produce a valid Task 4 result", so Tasks 1–3 are included and recorded
as `auto_dependencies` in the manifest.

This makes each run reproducible and prevents a Task 5 result from silently
consuming stale Task 1–3 files. To change any task, edit the versioned YAML
configuration or pass CLI values and rerun the desired prefix.

Task 3 is the only algorithmic branch between TRIAD, Davenport and SVD. Every
downstream result is therefore method-specific and lives in its own folder.

## Figure naming

```text
<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png
```

Each run also writes `figures_index.csv` and `figures_index.json` listing every
figure with its task, subtask, coordinate frame and source datasets.

### Coordinate frame tags

| Tag | Meaning |
|---|---|
"""

CLOSING = """
## Behaviour without a truth file

Truth is optional. Tasks 1–5 run unchanged, Task 6 writes `status: skipped`
with a reason instead of fabricating a reference, and Task 7 reports GNSS
position/velocity innovation RMS so the run still carries a quality signal.
Figures that need truth are recorded in `figures_index.csv` with
`status = skipped` rather than being silently omitted.
"""


def render() -> str:
    lines: list[str] = [PREAMBLE.rstrip("\n")]
    for tag, description in FRAMES.items():
        lines.append(f"| `{tag}` | {description} |")
    lines.append("")

    for task in TASKS:
        lines.append(f"## Task {task.number} — {task.name}")
        lines.append("")
        lines.append(f"{task.purpose}")
        lines.append("")
        lines.append(f"Output directory: `{task.directory_name}/`")
        lines.append("")
        lines.append("| Subtask | What it does | Figures (frame · data) |")
        lines.append("|---|---|---|")
        for subtask in task.subtasks:
            figures = [figure for figure in task.figures if figure.subtask == subtask.number]
            rendered = "<br>".join(
                f"`{figure.slug}` — {figure.title} "
                f"({figure.frame} · {'+'.join(figure.sources)})"
                + (" *[needs truth]*" if figure.requires_truth else "")
                for figure in figures
            ) or "—"
            lines.append(f"| **{subtask.number}** | {subtask.name} | {rendered} |")
        lines.append("")

    lines.append("## Cross-method comparison")
    lines.append("")
    lines.append(
        "Written once per multi-method run into "
        f"`{COMPARISON_SLUG}/`, alongside `method_comparison.json` and "
        "`method_comparison.csv`."
    )
    lines.append("")
    lines.append("| ID | Figure | Frame · data |")
    lines.append("|---|---|---|")
    for figure in COMPARISON_FIGURES:
        note = " *[needs truth]*" if figure.requires_truth else ""
        lines.append(
            f"| {figure.subtask} | `{figure.slug}` — {figure.title}{note} "
            f"| {figure.frame} · {'+'.join(figure.sources)} |"
        )
    lines.append(CLOSING)
    return "\n".join(lines).rstrip("\n") + "\n"


def main() -> int:
    text = render()
    previous = OUTPUT.read_text(encoding="utf-8") if OUTPUT.is_file() else None
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(text, encoding="utf-8")
    print(f"{'unchanged' if text == previous else 'updated'}: {OUTPUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
