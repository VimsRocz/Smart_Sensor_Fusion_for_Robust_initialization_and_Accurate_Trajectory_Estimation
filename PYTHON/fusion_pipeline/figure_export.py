"""Write Matplotlib figures and their MATLAB-compatible companions.

Matplotlib does not implement MATLAB's native ``.fig`` format.  When the
MATLAB Engine is installed this module rebuilds the visible plot in MATLAB and
calls ``savefig``.  On a Python-only machine it still writes the rendered
PNG/PDF and a best-effort ``.mat`` data companion; the release runner can
convert those PNGs to native FIG files later on a MATLAB system.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any


_ENG: Any = None

# FigureWriter uses this registry for lightweight progress/reporting.
WRITTEN: list[str] = []
NATIVE_WRITTEN: list[str] = []


def _write_generic_mat_companion(fig: Any, path: Path) -> None:
    """Save line data and labels when the caller supplied no richer MAT file."""
    if path.exists():
        return
    try:
        import numpy as np
        from scipy.io import savemat  # type: ignore

        data: dict[str, object] = {}
        for axes_index, axes in enumerate(fig.get_axes(), start=1):
            prefix = f"ax{axes_index}"
            data[f"{prefix}_title"] = np.array(axes.get_title(), dtype=object)
            data[f"{prefix}_xlabel"] = np.array(axes.get_xlabel(), dtype=object)
            data[f"{prefix}_ylabel"] = np.array(axes.get_ylabel(), dtype=object)
            for line_index, line in enumerate(axes.get_lines(), start=1):
                line_prefix = f"{prefix}_line{line_index}"
                data[f"{line_prefix}_x"] = np.asarray(line.get_xdata())
                data[f"{line_prefix}_y"] = np.asarray(line.get_ydata())
                label = line.get_label()
                if label and not label.startswith("_"):
                    data[f"{line_prefix}_label"] = np.array(label, dtype=object)
        if not data:
            data["plot_note"] = np.array(
                "Rendered plot is stored in the same-stem PNG/PDF; native FIG "
                "requires MATLAB.",
                dtype=object,
            )
        savemat(path, data, do_compression=True)
        print(f"[MAT ] {path} generic plotted-data companion")
    except Exception as exc:  # pragma: no cover - optional SciPy dependency
        print(f"[WARN] could not write generic MAT companion {path}: {exc}")


def _matlab_engine() -> Any:
    """Return one cached MATLAB Engine session, or ``False`` when unavailable."""
    global _ENG
    if _ENG is None:
        try:
            import matlab.engine  # type: ignore

            _ENG = matlab.engine.start_matlab()
            _ENG.close("all", nargout=0)
        except Exception as exc:  # pragma: no cover - environment dependent
            print(
                f"[INFO] MATLAB Engine unavailable ({exc}); native .fig export "
                "will be deferred until MATLAB batch conversion."
            )
            _ENG = False
    return _ENG


def _write_native_fig(fig: Any, target: Path) -> bool:
    """Embed the exact rendered PNG in a genuine native MATLAB FIG file."""
    eng = _matlab_engine()
    if not eng:
        return False

    png_path = target.with_suffix(".png")
    if not png_path.is_file():
        return False
    try:
        eng.close("all", nargout=0)
        quoted_png = str(png_path).replace("'", "''")
        quoted_target = str(target).replace("'", "''")
        eng.eval(
            "img=imread('" + quoted_png + "');"
            "h=figure('Visible','off','Color','w');"
            "image(img);axis image off;"
            "set(gca,'Position',[0 0 1 1]);"
            "savefig(h,'" + quoted_target + "');close(h);",
            nargout=0,
        )
    except Exception as exc:  # pragma: no cover - MATLAB dependent
        print(f"[WARN] could not write native FIG {target}: {exc}")
        return False
    print(f"[FIG] {target}")
    NATIVE_WRITTEN.append(target.name)
    return True


def save_matlab_fig(fig: Any, out_stem: str) -> Path | None:
    """Save PNG/PDF/MAT now and a native MATLAB FIG when MATLAB is available.

    ``out_stem`` is a path without a required extension.  The returned path is
    the native ``.fig`` path, or ``None`` when MATLAB is unavailable.  ``None``
    is intentional: a MAT file renamed to FIG would not be accepted by
    MATLAB's ``openfig`` and must never be emitted as a misleading fallback.
    """
    stem = Path(out_stem)
    if stem.suffix.lower() in {".png", ".pdf", ".mat", ".fig"}:
        stem = stem.with_suffix("")
    stem.parent.mkdir(parents=True, exist_ok=True)

    def artifact_path(suffix: str) -> Path:
        return stem.parent / f"{stem.name}{suffix}"

    has_geo = any(type(axis).__name__.startswith("GeoAxes") for axis in fig.get_axes())
    save_kw = {} if has_geo else {"bbox_inches": "tight"}
    for suffix, dpi in ((".png", 200), (".pdf", 300)):
        target = artifact_path(suffix)
        try:
            fig.savefig(target, dpi=dpi, **save_kw)
            print(f"[{suffix[1:].upper()}] {target}")
            if suffix == ".png":
                WRITTEN.append(target.name)
        except Exception as exc:  # pragma: no cover - backend/disk dependent
            print(f"[WARN] could not write {target}: {exc}")

    mat_path = artifact_path(".mat")
    _write_generic_mat_companion(fig, mat_path)

    fig_path = artifact_path(".fig")
    if _write_native_fig(fig, fig_path):
        return fig_path
    return None


def validate_fig_openable(fig_path: str | Path) -> bool:
    """Validate a native FIG through MATLAB's ``openfig`` when available."""
    eng = _matlab_engine()
    if not eng:
        print(f"[SKIP] MATLAB Engine unavailable; cannot validate {fig_path}")
        return False
    try:
        eng.close("all", nargout=0)
        eng.openfig(str(fig_path), "invisible", nargout=0)
        eng.close("all", nargout=0)
        print(f"[OK] Validated MATLAB .fig opens: {fig_path}")
        return True
    except Exception as exc:  # pragma: no cover - MATLAB dependent
        print(f"[FAIL] Could not open .fig in MATLAB: {fig_path} ({exc})")
        return False
