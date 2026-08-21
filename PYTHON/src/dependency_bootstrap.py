"""Install and verify dependencies before a standalone script starts.

This module intentionally uses only the Python standard library so it can be
imported before NumPy, SciPy, or any other third-party package.  Entry points
call :func:`ensure_dependencies` before importing their runtime dependencies.
"""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import subprocess
import sys
from typing import Mapping


HERE = Path(__file__).resolve().parent
DEFAULT_REQUIREMENTS = HERE / "requirements.txt"

# Distribution names and import names are not always identical (for example,
# PyYAML is imported as ``yaml``), so keep the mapping explicit.
REQUIRED_IMPORTS: Mapping[str, str] = {
    "numpy": "numpy",
    "scipy": "scipy",
    "pandas": "pandas",
    "matplotlib": "matplotlib",
    "PyYAML": "yaml",
    "filterpy": "filterpy",
    "rich": "rich",
    "tabulate": "tabulate",
    "tqdm": "tqdm",
    "pyproj": "pyproj",
    "plotly": "plotly",
    "kaleido": "kaleido",
}


def missing_dependencies() -> list[str]:
    """Return distribution names whose import modules are not available."""

    return [
        distribution
        for distribution, module in REQUIRED_IMPORTS.items()
        if importlib.util.find_spec(module) is None
    ]


def _pip_command(requirements: Path) -> list[str]:
    """Build a pip command for the interpreter currently running the script."""

    command = [sys.executable, "-m", "pip", "install", "-r", str(requirements)]
    # Avoid permission errors for a system interpreter while keeping virtual
    # environments isolated.  PIP_USER_INSTALL=0 can opt out when desired.
    in_virtualenv = sys.prefix != getattr(sys, "base_prefix", sys.prefix)
    if not in_virtualenv and os.environ.get("PIP_USER_INSTALL", "1") != "0":
        command.append("--user")
    return command


def ensure_dependencies(requirements: str | Path | None = None) -> None:
    """Install missing dependencies and fail clearly if they remain missing.

    Installation is skipped when all required imports are already available.
    Set ``IMU_FUSION_SKIP_DEPENDENCY_INSTALL=1`` for offline/CI environments;
    in that mode a clear error lists the missing packages instead.
    """

    requirements_path = Path(requirements) if requirements else DEFAULT_REQUIREMENTS
    requirements_path = requirements_path.expanduser().resolve()
    missing = missing_dependencies()
    if not missing:
        return

    if os.environ.get("IMU_FUSION_SKIP_DEPENDENCY_INSTALL") == "1":
        missing_text = ", ".join(missing)
        raise RuntimeError(
            "Missing Python dependencies: "
            f"{missing_text}. Install them with "
            f"{sys.executable} -m pip install -r {requirements_path}"
        )

    if not requirements_path.is_file():
        raise FileNotFoundError(
            f"Dependency file not found: {requirements_path}. "
            "Run the script from a checkout containing requirements.txt."
        )

    print(
        "Installing missing Python dependencies "
        f"({', '.join(missing)}) from {requirements_path} ...",
        flush=True,
    )
    subprocess.check_call(_pip_command(requirements_path))

    still_missing = missing_dependencies()
    if still_missing:
        raise RuntimeError(
            "Dependency installation completed, but these imports are still "
            f"missing: {', '.join(still_missing)}"
        )


def main() -> int:
    """Allow ``python dependency_bootstrap.py`` to verify/install dependencies."""

    ensure_dependencies()
    print("Python dependencies are ready.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
