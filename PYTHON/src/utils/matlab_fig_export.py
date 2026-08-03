from __future__ import annotations

from pathlib import Path
import matplotlib.pyplot as plt

_ENG = None  # lazy-initialised MATLAB engine


def _matlab_engine():
    global _ENG
    if _ENG is None:
        try:
            import matlab.engine  # type: ignore
            _ENG = matlab.engine.start_matlab()
            _ENG.close('all', nargout=0)
        except Exception as e:  # pragma: no cover - environment dependent
            print(f"[WARN] MATLAB engine unavailable: {e}")
            _ENG = False
    return _ENG


def _mpl_axes_to_matlab(ax, eng):
    import matlab  # type: ignore

    # Lines
    for line in ax.get_lines():
        x = line.get_xdata(); y = line.get_ydata()
        if len(x) and len(y):
            mx = matlab.double([float(v) for v in x])
            my = matlab.double([float(v) for v in y])
            eng.plot(mx, my, nargout=0)
            eng.hold('on', nargout=0)

    # Scatter collections (best-effort)
    for col in getattr(ax, 'collections', []):
        offsets = getattr(col, 'get_offsets', lambda: [])()
        if len(offsets):
            xs = [float(p[0]) for p in offsets]
            ys = [float(p[1]) for p in offsets]
            eng.scatter(xs, ys, nargout=0)
            eng.hold('on', nargout=0)

    # Labels/Title/Legend
    xl = ax.get_xlabel() or ""; yl = ax.get_ylabel() or ""; tl = ax.get_title() or ""
    if xl: eng.xlabel(xl, nargout=0)
    if yl: eng.ylabel(yl, nargout=0)
    if tl: eng.title(tl, nargout=0)
    labels = [l.get_label() for l in ax.get_lines() if l.get_label() and not l.get_label().startswith('_')]
    if labels:
        try:
            eng.legend(labels, nargout=0)
        except Exception:
            pass
    # Grid (best-effort)
    try:
        if getattr(ax.xaxis, '_gridOnMajor', False) or getattr(ax.yaxis, '_gridOnMajor', False):
            eng.grid('on', nargout=0)
    except Exception:
        pass


def save_matlab_fig(fig, out_stem: str) -> Path | None:
    """Save a Matplotlib figure as PNG and PDF, plus a native MATLAB .fig.

    The PNG/PDF export is pure Matplotlib and always runs. Only the native
    ``.fig`` mirror needs the MATLAB engine; without it the figures are still
    written, which is what every caller expects.

    Returns the Path to the saved ``.fig``, or None when MATLAB is absent.
    """
    stem = Path(out_stem)
    stem.parent.mkdir(parents=True, exist_ok=True)

    for suffix, dpi in ((".png", 200), (".pdf", 300)):
        target = stem.with_suffix(suffix)
        try:
            fig.savefig(target, dpi=dpi, bbox_inches="tight")
            print(f"[{suffix[1:].upper()}] {target}")
        except Exception as exc:  # pragma: no cover - backend dependent
            print(f"[WARN] could not write {target}: {exc}")

    eng = _matlab_engine()
    if not eng:
        return None

    out = stem.with_suffix('.fig')

    # Extract axes from the source figure
    axes = [ax for ax in fig.get_axes() if ax.get_visible()]
    if not axes:
        return None

    # Build MATLAB figure offscreen and paint content
    eng.close('all', nargout=0)
    eng.figure('Visible', 'off', nargout=0)
    rows, cols = (1, len(axes)) if len(axes) > 1 else (1, 1)
    for idx, ax in enumerate(axes, start=1):
        eng.subplot(float(rows), float(cols), float(idx), nargout=0)
        _mpl_axes_to_matlab(ax, eng)

    eng.savefig(str(out), nargout=0)  # native .fig
    print(f"[FIG] {out}")
    return out


def save_all_matplotlib_as_fig(out_dir: str, prefix: str = "") -> None:
    """Recreate every open Matplotlib figure in MATLAB and save as .fig only."""
    eng = _matlab_engine()
    if not eng:
        print("[SKIP] MATLAB engine not available; .fig export skipped.")
        return

    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    for num in plt.get_fignums():
        fig = plt.figure(num)
        supt = fig._suptitle.get_text() if getattr(fig, '_suptitle', None) else f"figure_{num}"
        base = supt.strip().replace(" ", "_").replace("/", "_") or f"figure_{num}"
        if prefix:
            base = f"{prefix}_{base}"
        save_matlab_fig(fig, str(out / base))


def validate_fig_openable(fig_path: str | Path) -> bool:
    """Attempt to open and close a MATLAB .fig using the MATLAB engine.

    Returns True if open/close succeeds; False if the engine is unavailable or
    openfig fails. This is a best-effort validation to check file integrity.
    """
    eng = _matlab_engine()
    if not eng:
        # Cannot validate without MATLAB engine; report False (unknown)
        print(f"[SKIP] MATLAB engine unavailable; cannot validate {fig_path}")
        return False
    try:
        eng.close('all', nargout=0)
        eng.openfig(str(fig_path), 'invisible', nargout=0)
        eng.close('all', nargout=0)
        print(f"[OK] Validated MATLAB .fig opens: {fig_path}")
        return True
    except Exception as e:  # pragma: no cover - engine dependent
        print(f"[FAIL] Could not open .fig in MATLAB: {fig_path} ({e})")
        return False
