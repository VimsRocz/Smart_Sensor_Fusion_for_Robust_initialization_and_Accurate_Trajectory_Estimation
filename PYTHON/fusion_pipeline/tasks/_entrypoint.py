"""Shared command runner for one document-numbered task module."""

from __future__ import annotations

import sys
from typing import Sequence

from ..catalog import TASK_BY_NUMBER
from ..cli import main as pipeline_main


def describe_task(task_number: int) -> str:
    task = TASK_BY_NUMBER[task_number]
    lines = [f"Task {task.number}: {task.name}", task.purpose, "", "Subtasks:"]
    lines.extend(
        f"  {subtask.number}  {subtask.name}" for subtask in task.subtasks
    )
    lines.append("")
    lines.append("Figures:")
    lines.extend(
        f"  {figure.subtask}  {figure.slug} [{figure.frame}]"
        for figure in task.figures
    )
    return "\n".join(lines)


def run_task(task_number: int, argv: Sequence[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    if arguments == ["--list-subtasks"]:
        print(describe_task(task_number))
        return 0
    if "--tasks" in arguments or any(
        argument.startswith("--tasks=") for argument in arguments
    ):
        raise SystemExit(
            "A task module fixes --tasks automatically; remove the explicit --tasks option."
        )
    return pipeline_main([*arguments, "--tasks", str(task_number)])
