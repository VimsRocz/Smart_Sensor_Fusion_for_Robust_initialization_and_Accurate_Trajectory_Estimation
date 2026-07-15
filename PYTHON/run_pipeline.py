#!/usr/bin/env python3
"""Stable repository entry point for Tasks 1-7."""

from __future__ import annotations

import sys
from pathlib import Path

PYTHON_ROOT = Path(__file__).resolve().parent
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from fusion_pipeline.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
