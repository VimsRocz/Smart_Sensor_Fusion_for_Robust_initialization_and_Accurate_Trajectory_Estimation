"""Write plots plus a data-backed interchange file for native MATLAB FIGs."""

from __future__ import annotations

from pathlib import Path
from typing import Any


_ENG: Any = None
REPO_ROOT = Path(__file__).resolve().parents[2]
MATLAB_ROOT = REPO_ROOT / "MATLAB"

# FigureWriter uses this registry for lightweight progress/reporting.
WRITTEN: list[str] = []
NATIVE_WRITTEN: list[str] = []


def _write_generic_mat_companion(fig: Any, path: Path) -> None:
    """Serialize axes and plotted objects for editable MATLAB reconstruction."""
    try:
        import numpy as np
        from matplotlib.collections import PathCollection, QuadMesh
        from matplotlib.colors import to_rgba
        from scipy.io import loadmat, savemat  # type: ignore

        def text_value(value: Any) -> np.ndarray:
            return np.array(str(value or ""), dtype=object)

        def rgba(value: Any) -> np.ndarray:
            try:
                return np.asarray(to_rgba(value), dtype=float)
            except (TypeError, ValueError):
                return np.asarray((0.0, 0.0, 0.0, 1.0), dtype=float)

        suptitle = getattr(fig, "_suptitle", None)
        footer_parts = [
            item.get_text()
            for item in fig.texts
            if item is not suptitle and item.get_text().strip()
        ]
        data: dict[str, object] = {}
        if path.is_file():
            existing = loadmat(path)
            data.update(
                {key: value for key, value in existing.items() if not key.startswith("__")}
            )
        data.update({
            "figure_schema_version": text_value("sensor-fusion-figure-v1"),
            "figure_title": text_value(suptitle.get_text() if suptitle else ""),
            "figure_footer": text_value("\n".join(footer_parts)),
            "figure_size_inches": np.asarray(fig.get_size_inches(), dtype=float),
            "axes_count": np.asarray([[len(fig.get_axes())]], dtype=np.int32),
        })
        for axes_index, axes in enumerate(fig.get_axes(), start=1):
            prefix = f"ax{axes_index}"
            data[f"{prefix}_position"] = np.asarray(axes.get_position().bounds, dtype=float)
            data[f"{prefix}_title"] = text_value(axes.get_title())
            data[f"{prefix}_xlabel"] = text_value(axes.get_xlabel())
            data[f"{prefix}_ylabel"] = text_value(axes.get_ylabel())
            data[f"{prefix}_xlim"] = np.asarray(axes.get_xlim(), dtype=float)
            data[f"{prefix}_ylim"] = np.asarray(axes.get_ylim(), dtype=float)
            data[f"{prefix}_xscale"] = text_value(axes.get_xscale())
            data[f"{prefix}_yscale"] = text_value(axes.get_yscale())
            data[f"{prefix}_facecolor"] = rgba(axes.get_facecolor())
            data[f"{prefix}_axis_on"] = np.asarray([[int(axes.axison)]], dtype=np.int8)
            data[f"{prefix}_xticks"] = np.asarray(axes.get_xticks(), dtype=float)
            data[f"{prefix}_yticks"] = np.asarray(axes.get_yticks(), dtype=float)
            data[f"{prefix}_xticklabels"] = np.asarray(
                [item.get_text() for item in axes.get_xticklabels()], dtype=object
            )
            data[f"{prefix}_yticklabels"] = np.asarray(
                [item.get_text() for item in axes.get_yticklabels()], dtype=object
            )
            aspect = axes.get_aspect()
            data[f"{prefix}_aspect"] = text_value(aspect)
            data[f"{prefix}_aspect_equal"] = np.asarray(
                [[int(aspect == "equal" or aspect == 1.0)]], dtype=np.int8
            )
            grid_visible = any(line.get_visible() for line in axes.get_xgridlines()) or any(
                line.get_visible() for line in axes.get_ygridlines()
            )
            data[f"{prefix}_grid"] = np.asarray([[int(grid_visible)]], dtype=np.int8)

            lines = axes.get_lines()
            data[f"{prefix}_line_count"] = np.asarray([[len(lines)]], dtype=np.int32)
            for line_index, line in enumerate(lines, start=1):
                line_prefix = f"{prefix}_line{line_index}"
                data[f"{line_prefix}_x"] = np.asarray(line.get_xdata())
                data[f"{line_prefix}_y"] = np.asarray(line.get_ydata())
                label = line.get_label()
                data[f"{line_prefix}_label"] = text_value(
                    "" if not label or label.startswith("_") else label
                )
                data[f"{line_prefix}_color"] = rgba(line.get_color())
                data[f"{line_prefix}_linestyle"] = text_value(line.get_linestyle())
                data[f"{line_prefix}_linewidth"] = np.asarray(
                    [[float(line.get_linewidth())]], dtype=float
                )
                data[f"{line_prefix}_marker"] = text_value(line.get_marker())
                data[f"{line_prefix}_markersize"] = np.asarray(
                    [[float(line.get_markersize())]], dtype=float
                )

            collections = [
                collection
                for collection in axes.collections
                if isinstance(collection, PathCollection) and len(collection.get_offsets())
            ]
            data[f"{prefix}_scatter_count"] = np.asarray(
                [[len(collections)]], dtype=np.int32
            )
            for collection_index, collection in enumerate(collections, start=1):
                collection_prefix = f"{prefix}_scatter{collection_index}"
                data[f"{collection_prefix}_offsets"] = np.asarray(
                    collection.get_offsets(), dtype=float
                )
                sizes = getattr(collection, "get_sizes", lambda: np.asarray([20.0]))()
                data[f"{collection_prefix}_sizes"] = np.asarray(sizes, dtype=float)
                facecolors = getattr(collection, "get_facecolors", lambda: np.empty((0, 4)))()
                edgecolors = getattr(collection, "get_edgecolors", lambda: np.empty((0, 4)))()
                data[f"{collection_prefix}_facecolor"] = (
                    np.asarray(facecolors[0], dtype=float)
                    if len(facecolors)
                    else rgba("tab:blue")
                )
                data[f"{collection_prefix}_edgecolor"] = (
                    np.asarray(edgecolors[0], dtype=float)
                    if len(edgecolors)
                    else rgba("none")
                )
                label = collection.get_label()
                data[f"{collection_prefix}_label"] = text_value(
                    "" if not label or label.startswith("_") else label
                )

            rectangles = [
                patch
                for patch in axes.patches
                if all(hasattr(patch, name) for name in ("get_x", "get_y", "get_width", "get_height"))
            ]
            container_labels: dict[int, str] = {}
            for container in axes.containers:
                label = getattr(container, "get_label", lambda: "")()
                if not label or label.startswith("_"):
                    continue
                for patch_index, patch in enumerate(getattr(container, "patches", ())):
                    container_labels[id(patch)] = label if patch_index == 0 else ""
            data[f"{prefix}_rectangle_count"] = np.asarray(
                [[len(rectangles)]], dtype=np.int32
            )
            for rectangle_index, rectangle in enumerate(rectangles, start=1):
                rectangle_prefix = f"{prefix}_rectangle{rectangle_index}"
                display_vertices = rectangle.get_transform().transform(
                    rectangle.get_path().vertices
                )
                data_vertices = axes.transData.inverted().transform(display_vertices)
                minimum = np.nanmin(data_vertices, axis=0)
                maximum = np.nanmax(data_vertices, axis=0)
                data[f"{rectangle_prefix}_position"] = np.asarray(
                    [
                        minimum[0],
                        minimum[1],
                        maximum[0] - minimum[0],
                        maximum[1] - minimum[1],
                    ],
                    dtype=float,
                )
                data[f"{rectangle_prefix}_facecolor"] = rgba(rectangle.get_facecolor())
                data[f"{rectangle_prefix}_edgecolor"] = rgba(rectangle.get_edgecolor())
                label = container_labels.get(id(rectangle), rectangle.get_label())
                data[f"{rectangle_prefix}_label"] = text_value(
                    "" if not label or label.startswith("_") else label
                )

            images: list[dict[str, Any]] = []
            for image in axes.get_images():
                images.append(
                    {
                        "cdata": np.asarray(image.get_array()),
                        "extent": np.asarray(image.get_extent(), dtype=float),
                        "origin": image.origin,
                        "colormap": image.get_cmap().name,
                        "clim": image.get_clim(),
                    }
                )
            for mesh in (item for item in axes.collections if isinstance(item, QuadMesh)):
                coordinates = np.asarray(mesh.get_coordinates(), dtype=float)
                cdata = np.asarray(mesh.get_array())
                rows, columns = coordinates.shape[0] - 1, coordinates.shape[1] - 1
                if cdata.ndim == 1 and cdata.size == rows * columns:
                    cdata = cdata.reshape(rows, columns)
                images.append(
                    {
                        "cdata": cdata,
                        "extent": np.asarray(
                            [
                                np.nanmin(coordinates[..., 0]),
                                np.nanmax(coordinates[..., 0]),
                                np.nanmin(coordinates[..., 1]),
                                np.nanmax(coordinates[..., 1]),
                            ],
                            dtype=float,
                        ),
                        "origin": "lower",
                        "colormap": mesh.get_cmap().name,
                        "clim": mesh.get_clim(),
                    }
                )
            data[f"{prefix}_image_count"] = np.asarray([[len(images)]], dtype=np.int32)
            for image_index, image in enumerate(images, start=1):
                image_prefix = f"{prefix}_image{image_index}"
                data[f"{image_prefix}_cdata"] = image["cdata"]
                data[f"{image_prefix}_extent"] = image["extent"]
                data[f"{image_prefix}_origin"] = text_value(image["origin"])
                data[f"{image_prefix}_colormap"] = text_value(image["colormap"])
                data[f"{image_prefix}_clim"] = np.asarray(image["clim"], dtype=float)

            texts = [item for item in axes.texts if item.get_text().strip()]
            data[f"{prefix}_text_count"] = np.asarray([[len(texts)]], dtype=np.int32)
            for text_index, item in enumerate(texts, start=1):
                text_prefix = f"{prefix}_text{text_index}"
                data[f"{text_prefix}_position"] = np.asarray(item.get_position(), dtype=float)
                data[f"{text_prefix}_string"] = text_value(item.get_text())
                data[f"{text_prefix}_coordinates"] = text_value(
                    "axes" if item.get_transform() == axes.transAxes else "data"
                )

        savemat(path, data, do_compression=True, long_field_names=True)
        print(f"[MAT ] {path} editable plotted-data companion")
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


def _write_native_fig(_fig: Any, target: Path) -> bool:
    """Ask MATLAB to reconstruct data objects and save a genuine FIG."""
    eng = _matlab_engine()
    if not eng:
        return False

    png_path = target.with_suffix(".png")
    mat_path = target.with_suffix(".mat")
    if not png_path.is_file() or not mat_path.is_file():
        return False
    try:
        eng.close("all", nargout=0)
        eng.addpath(str(MATLAB_ROOT), nargout=0)
        mode = eng.export_python_figure(
            str(mat_path),
            str(png_path),
            str(target),
            nargout=1,
        )
    except Exception as exc:  # pragma: no cover - MATLAB dependent
        print(f"[WARN] could not write native FIG {target}: {exc}")
        return False
    print(f"[FIG] {target} ({mode})")
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
