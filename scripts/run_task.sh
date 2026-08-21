#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || ! "$1" =~ ^[1-7]$ ]]; then
    echo "Usage: scripts/run_task.sh TASK_NUMBER [pipeline options]" >&2
    exit 2
fi

task_number="$1"
shift
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${repo_root}/.venv/bin/python"
if [[ ! -x "${python_bin}" ]]; then
    python_bin="$(command -v python3)"
fi

cd "${repo_root}"
export PYTHONPATH="${repo_root}/PYTHON${PYTHONPATH:+:${PYTHONPATH}}"
exec "${python_bin}" -m "fusion_pipeline.tasks.task_0${task_number}" "$@"
