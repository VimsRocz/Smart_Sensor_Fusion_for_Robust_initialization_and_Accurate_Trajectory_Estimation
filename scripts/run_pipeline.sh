#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${repo_root}/.venv/bin/python"
if [[ ! -x "${python_bin}" ]]; then
    python_bin="$(command -v python3)"
fi

cd "${repo_root}"
exec "${python_bin}" PYTHON/main.py "$@"
