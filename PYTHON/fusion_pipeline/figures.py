"""Figure generation, naming and indexing for Tasks 1-7.

Every PNG produced by the pipeline is named from the catalog so the filename
alone identifies the run, the method, the task, the subtask, the task name, the
figure, the coordinate frame, and the sensor files the data came from::

    <run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png

The same metadata is stamped onto the figure itself and collected into
``figures_index.json`` / ``figures_index.csv`` at the root of the run.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence

import numpy as np

from .catalog import (
    COMPARISON_FIGURES,
    COMPARISON_SLUG,
    FigureSpec,
    TASK_BY_NUMBER,
    TaskSpec,
    frame_description,
)
from .math3d import quaternion_series_to_euler_zyx_deg

NED_LABELS = ("North", "East", "Down")
AXIS_LABELS = ("x", "y", "z")
QUATERNION_LABELS = ("w", "x", "y", "z")
EULER_LABELS = ("Yaw", "Pitch", "Roll")


def _pyplot():
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    return plt


def safe_token(value: str) -> str:
    """Reduce a value to characters that are safe in a filename."""
    cleaned = "".join(ch if ch.isalnum() or ch in "-_." else "_" for ch in str(value))
    while "__" in cleaned:
        cleaned = cleaned.replace("__", "_")
    return cleaned.strip("_") or "unnamed"


TITLE_FONTSIZE = 12.0
CAVEAT_FONTSIZE = 8.0
FOOTER_FONTSIZE = 7.5
FOOTER_MIN_FONTSIZE = 5.0
#: Mean glyph width of the default sans font, in inches per point of font size.
_GLYPH_WIDTH_IN_PER_PT = 0.0083
_FOOTER_USABLE_FRACTION = 0.96


def _max_fontsize(text: str, width_inches: float) -> float:
    """Largest font size at which ``text`` still fits across the figure."""
    if not text:
        return FOOTER_FONTSIZE
    fitted = _FOOTER_USABLE_FRACTION * width_inches / (len(text) * _GLYPH_WIDTH_IN_PER_PT)
    return max(FOOTER_MIN_FONTSIZE, fitted)


def _fits(text: str, width_inches: float, fontsize: float) -> bool:
    return len(text) * _GLYPH_WIDTH_IN_PER_PT * fontsize <= _FOOTER_USABLE_FRACTION * width_inches


#: Attribute a builder sets to have _stamp draw a caveat under the figure title.
_CAVEAT_ATTRIBUTE = "_fusion_caveat"


def set_caveat(fig, text: str) -> None:
    """Attach a one-line caveat that :meth:`FigureWriter.emit` renders below the title."""
    setattr(fig, _CAVEAT_ATTRIBUTE, text)


def stride_for(length: int, max_points: int) -> int:
    """Decimation step that keeps a plot under ``max_points`` markers."""
    if length <= max_points:
        return 1
    return int(np.ceil(length / max_points))


class FigureWriter:
    """Creates, stamps, names and indexes every figure of one method run."""

    def __init__(
        self,
        *,
        run_id: str,
        method: str,
        dataset_tags: dict[str, str | None],
        enabled: bool = True,
        max_points: int = 50_000,
        dpi: int = 160,
        progress: Any = None,
    ) -> None:
        self.progress = progress
        self.run_id = safe_token(run_id)
        self.method = method
        self.dataset_tags = {
            key: (safe_token(value) if value else None)
            for key, value in dataset_tags.items()
        }
        self.enabled = bool(enabled)
        self.max_points = int(max_points)
        self.dpi = int(dpi)
        self.records: list[dict[str, Any]] = []
        self.plt = _pyplot() if self.enabled else None

    # -- naming ---------------------------------------------------------
    def data_tag(self, sources: Sequence[str]) -> str:
        parts = [self.dataset_tags.get(source) for source in sources]
        present = [part for part in parts if part]
        return "+".join(present) if present else "none"

    def filename(self, task: TaskSpec, figure: FigureSpec) -> str:
        return (
            f"{self.run_id}"
            f"_{safe_token(self.method)}"
            f"_task{task.number:02d}"
            f"_sub{figure.subtask}"
            f"_{task.slug}"
            f"_{figure.slug}"
            f"_frame-{figure.frame}"
            f"_data-{self.data_tag(figure.sources)}"
            ".png"
        )

    def comparison_filename(self, figure: FigureSpec) -> str:
        return (
            f"{self.run_id}"
            f"_ALLMETHODS"
            f"_{COMPARISON_SLUG}"
            f"_sub{figure.subtask}"
            f"_{figure.slug}"
            f"_frame-{figure.frame}"
            f"_data-{self.data_tag(figure.sources)}"
            ".png"
        )

    # -- emission -------------------------------------------------------
    def _spec(self, task_number: int, figure_slug: str) -> tuple[TaskSpec | None, FigureSpec]:
        if task_number == 0:
            match = next((f for f in COMPARISON_FIGURES if f.slug == figure_slug), None)
            if match is None:
                raise KeyError(f"Unknown comparison figure {figure_slug!r}")
            return None, match
        task = TASK_BY_NUMBER[task_number]
        match = next((f for f in task.figures if f.slug == figure_slug), None)
        if match is None:
            raise KeyError(f"Task {task_number} has no figure {figure_slug!r}")
        return task, match

    def _stamp(self, fig, task: TaskSpec | None, figure: FigureSpec) -> tuple[float, float]:
        """Add the identifying title and footer; return the (bottom, top) margins."""
        if task is None:
            heading = f"Cross-method comparison {figure.subtask} — {figure.title}"
        else:
            heading = (
                f"Task {task.number} · Subtask {figure.subtask} — {task.name}\n{figure.title}"
            )
        caveat = getattr(fig, _CAVEAT_ATTRIBUTE, None)
        top = 0.94
        if caveat:
            # Reserve a band for the caveat measured from the real figure height,
            # so it never lands on the descenders of the title.
            height_inches = float(fig.get_size_inches()[1])
            title_inches = (heading.count("\n") + 1) * TITLE_FONTSIZE * 1.25 / 72.0
            caveat_inches = CAVEAT_FONTSIZE * 1.3 / 72.0
            gap = 0.07
            fig.suptitle(heading, fontsize=TITLE_FONTSIZE, fontweight="bold", y=0.995, va="top")
            caveat_y = 0.995 - (title_inches + gap) / height_inches
            fig.text(
                0.5, caveat_y, caveat, ha="center", va="top",
                fontsize=CAVEAT_FONTSIZE, color="#8a5a00",
            )
            top = caveat_y - (caveat_inches + gap) / height_inches
        else:
            fig.suptitle(heading, fontsize=TITLE_FONTSIZE, fontweight="bold")
        frame_line = f"frame: {figure.frame} — {frame_description(figure.frame)}"
        origin_line = (
            f"data: {self.data_tag(figure.sources).replace('+', ' + ')}    |    "
            f"run: {self.run_id}    |    method: {self.method}"
        )
        # A single line only fits on wide figures; narrow ones would clip it.
        width_inches = float(fig.get_size_inches()[0])
        single = f"{frame_line}    |    {origin_line}"
        if _fits(single, width_inches, FOOTER_FONTSIZE):
            lines = [single]
        else:
            lines = [frame_line, origin_line]
        fontsize = min(
            FOOTER_FONTSIZE,
            *(_max_fontsize(line, width_inches) for line in lines),
        )
        fig.text(
            0.5,
            0.006,
            "\n".join(lines),
            ha="center",
            va="bottom",
            fontsize=fontsize,
            color="#444444",
            linespacing=1.4,
        )
        return (0.035 if len(lines) == 1 else 0.062), top

    def emit(
        self,
        task_number: int,
        figure_slug: str,
        directory: Path,
        builder: Callable[[Any], Any],
    ) -> Path | None:
        """Build, stamp, save and index one figure.

        ``builder`` receives ``matplotlib.pyplot`` and returns the Figure.
        """
        task, figure = self._spec(task_number, figure_slug)
        if not self.enabled:
            self._record(task, figure, None, "disabled", "plots are turned off")
            return None
        name = self.filename(task, figure) if task else self.comparison_filename(figure)
        path = Path(directory) / name
        path.parent.mkdir(parents=True, exist_ok=True)
        fig = builder(self.plt)
        try:
            bottom, top = self._stamp(fig, task, figure)
            fig.tight_layout(rect=(0, bottom, 1, top))
            fig.savefig(path, dpi=self.dpi)
        finally:
            self.plt.close(fig)
        self._record(task, figure, path, "written", None)
        return path

    def skip(self, task_number: int, figure_slug: str, reason: str) -> None:
        task, figure = self._spec(task_number, figure_slug)
        self._record(task, figure, None, "skipped", reason)

    def _record(
        self,
        task: TaskSpec | None,
        figure: FigureSpec,
        path: Path | None,
        status: str,
        reason: str | None,
    ) -> None:
        self.records.append(
            {
                "task": task.number if task else 0,
                "task_name": task.name if task else "Cross-method comparison",
                "task_directory": task.directory_name if task else COMPARISON_SLUG,
                "subtask": figure.subtask,
                "subtask_name": task.subtask_map[figure.subtask] if task else figure.title,
                "figure": figure.slug,
                "figure_title": figure.title,
                "coordinate_frame": figure.frame,
                "coordinate_frame_meaning": frame_description(figure.frame),
                "sources": list(figure.sources),
                "sensor_data": self.data_tag(figure.sources),
                "method": self.method,
                "status": status,
                "reason": reason,
                "filename": path.name if path else None,
                "path": str(path) if path else None,
            }
        )
        if self.progress is not None:
            self.progress.figure(
                figure.subtask, figure.slug, figure.frame, status, figure.sources
            )

    # -- index ----------------------------------------------------------
    def write_index(self, run_dir: Path) -> dict[str, Path]:
        run_dir = Path(run_dir)
        run_dir.mkdir(parents=True, exist_ok=True)
        written = [record for record in self.records if record["status"] == "written"]
        payload = {
            "run_id": self.run_id,
            "method": self.method,
            "figure_name_pattern": (
                "<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>"
                "_frame-<FRAME>_data-<DATASETS>.png"
            ),
            "datasets": self.dataset_tags,
            "figures_written": len(written),
            "figures_total": len(self.records),
            "figures": self.records,
        }
        json_path = run_dir / "figures_index.json"
        json_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        csv_path = run_dir / "figures_index.csv"
        columns = [
            "task",
            "task_name",
            "subtask",
            "subtask_name",
            "figure",
            "figure_title",
            "coordinate_frame",
            "sensor_data",
            "method",
            "status",
            "task_directory",
            "filename",
        ]
        with csv_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            for record in self.records:
                writer.writerow(record)
        return {"json": json_path, "csv": csv_path}


# ---------------------------------------------------------------------------
# Shared drawing helpers
# ---------------------------------------------------------------------------
def _triple_axes(plt, rows: int = 1, sharex: bool = True, width: float = 13.0, height: float = 3.2):
    return plt.subplots(rows, 3, figsize=(width, height * rows + 1.2), sharex=sharex, squeeze=False)


#: Keeps in-axes annotations legible where they overlap plotted data.
_ANNOTATION_BOX = {"facecolor": "white", "alpha": 0.78, "edgecolor": "none", "pad": 2.0}


def _annotate(axis, x: float, y: float, text: str, **kwargs: Any) -> None:
    axis.text(
        x,
        y,
        text,
        transform=axis.transAxes,
        fontsize=kwargs.pop("fontsize", 8),
        bbox=_ANNOTATION_BOX,
        zorder=5,
        **kwargs,
    )


#: Beyond this pitch the ZYX yaw/roll pair is ill-conditioned and can swing by
#: 180 degrees between neighbouring samples without the attitude actually moving.
GIMBAL_WARNING_PITCH_DEG = 80.0


def _note_euler_caveats(fig, *angle_series: np.ndarray) -> None:
    """Explain the two ways a ZYX Euler trace misleads: wrapping and gimbal lock.

    Both artifacts look like violent attitude changes but are properties of the
    parameterisation, not of the motion, so the plot says so rather than leaving
    a reader to mistake them for a filter failure.
    """
    series = [angles for angles in angle_series if angles.size]
    if not series:
        return
    notes: list[str] = []
    peak_pitch = max(float(np.max(np.abs(angles[:, 1]))) for angles in series)
    if peak_pitch >= GIMBAL_WARNING_PITCH_DEG:
        notes.append(
            f"pitch reaches {peak_pitch:.1f}°, so yaw and roll are ill-conditioned near ±90°"
        )
    wrapped = any(
        angles.shape[0] > 1
        and np.any(np.abs(np.diff(angles[:, [0, 2]], axis=0)) > 180.0)
        for angles in series
    )
    if wrapped:
        notes.append("yaw/roll steps of ~360° are atan2 wraps at ±180°, not attitude changes")
    if not notes:
        return
    # Handed to _stamp rather than drawn here, so the layout can reserve space
    # for it instead of letting it collide with the title.
    set_caveat(fig, "Note: " + "; ".join(notes) + ". The quaternion figure is unambiguous.")


def _decorate(axis, xlabel: str | None, ylabel: str | None, title: str | None = None) -> None:
    if title:
        axis.set_title(title)
    if xlabel:
        axis.set_xlabel(xlabel)
    if ylabel:
        axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)


def _time_series_triple(
    plt,
    time_s: np.ndarray,
    series: Sequence[tuple[str, np.ndarray, dict[str, Any]]],
    column_titles: Sequence[str],
    ylabel: str,
) -> Any:
    fig, axes = _triple_axes(plt)
    for column in range(3):
        axis = axes[0, column]
        for label, values, style in series:
            axis.plot(time_s, values[:, column], label=label, **style)
        _decorate(axis, "Time [s]", ylabel if column == 0 else None, column_titles[column])
    if len(series) > 1:
        axes[0, 0].legend(fontsize=8)
    return fig


def _state_grid(
    plt,
    time_s: np.ndarray,
    layers: Sequence[tuple[str, np.ndarray, np.ndarray, dict[str, Any]]],
) -> Any:
    """Two rows (position, velocity) by three NED columns."""
    fig, axes = _triple_axes(plt, rows=2)
    for column in range(3):
        for label, position, velocity, style in layers:
            axes[0, column].plot(time_s, position[:, column], label=label, **style)
            axes[1, column].plot(time_s, velocity[:, column], label=label, **style)
        _decorate(axes[0, column], None, "Position [m]" if column == 0 else None, NED_LABELS[column])
        _decorate(axes[1, column], "Time [s]", "Velocity [m/s]" if column == 0 else None)
    if len(layers) > 1:
        axes[0, 0].legend(fontsize=8)
    return fig


# ---------------------------------------------------------------------------
# Task 1
# ---------------------------------------------------------------------------
def draw_task1(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    imu, gnss, truth = ctx["imu"], ctx["gnss"], ctx["truth"]
    task1 = ctx["task1"]
    step = stride_for(len(imu.time_s), writer.max_points)

    def coverage(plt):
        fig, axis = plt.subplots(figsize=(11, 3.6))
        entries = [
            ("IMU", imu.time_s, f"{1.0 / imu.dt_s:.1f} Hz, {imu.rows} rows"),
            ("GNSS", gnss.time_s, f"{gnss.rows} epochs"),
        ]
        if truth is not None:
            entries.append(("Truth", truth.time_s, f"{truth.rows} states"))
        for index, (name, time_s, note) in enumerate(entries):
            axis.barh(index, time_s[-1] - time_s[0], left=time_s[0], height=0.45, alpha=0.75)
            axis.text(time_s[0], index + 0.32, f"{name}: {note}", fontsize=8.5)
        axis.set_yticks(range(len(entries)), [name for name, _, _ in entries])
        _decorate(axis, "Time since first sample [s]", None)
        return fig

    def gnss_raw(plt):
        fig, axes = _triple_axes(plt, rows=2, sharex=True)
        for column in range(3):
            axes[0, column].plot(gnss.time_s, gnss.position_ecef_m[:, column], marker="o", markersize=2.5, linewidth=0.9)
            axes[1, column].plot(gnss.time_s, gnss.velocity_ecef_mps[:, column], marker="o", markersize=2.5, linewidth=0.9, color="tab:orange")
            _decorate(axes[0, column], None, "ECEF position [m]" if column == 0 else None, f"ECEF {AXIS_LABELS[column].upper()}")
            _decorate(axes[1, column], "Time [s]", "ECEF velocity [m/s]" if column == 0 else None)
            axes[0, column].ticklabel_format(axis="y", style="plain", useOffset=False)
        return fig

    def imu_raw_pair(plt):
        fig, axes = _triple_axes(plt, rows=2)
        for column in range(3):
            axes[0, column].plot(imu.time_s[::step], imu.gyro_rps[::step, column], linewidth=0.8, color="tab:blue")
            axes[1, column].plot(imu.time_s[::step], imu.accel_mps2[::step, column], linewidth=0.8, color="tab:red")
            _decorate(axes[0, column], None, "Angular rate [rad/s]" if column == 0 else None, f"Body {AXIS_LABELS[column]}")
            _decorate(axes[1, column], "Time [s]", "Specific force [m/s²]" if column == 0 else None)
        return fig

    def origin_map(plt):
        latitude = float(task1["latitude_deg"])
        longitude = float(task1["longitude_deg"])
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
        axes[0].scatter([longitude], [latitude], color="crimson", s=55, zorder=3)
        axes[0].axhline(0, color="#888888", linewidth=0.6)
        axes[0].axvline(0, color="#888888", linewidth=0.6)
        axes[0].set(xlim=(-180, 180), ylim=(-90, 90))
        _decorate(axes[0], "Longitude [deg]", "Latitude [deg]", "Global position")
        span = 0.05
        axes[1].scatter([longitude], [latitude], color="crimson", s=70, zorder=3, label="Task 1 origin")
        axes[1].set(xlim=(longitude - span, longitude + span), ylim=(latitude - span, latitude + span))
        axes[1].ticklabel_format(useOffset=False, style="plain")
        _decorate(axes[1], "Longitude [deg]", "Latitude [deg]", "Local detail (±0.05°)")
        axes[1].legend(fontsize=8)
        _annotate(
            axes[1],
            0.02,
            0.02,
            f"lat {latitude:.6f}°   lon {longitude:.6f}°   alt {float(task1['altitude_m']):.2f} m",
        )
        return fig

    def reference_vectors(plt):
        gravity = np.asarray(task1["specific_force_reference_ned_mps2"], dtype=float)
        earth_rate = np.asarray(task1["earth_rate_reference_ned_rps"], dtype=float)
        fig, axes = plt.subplots(1, 3, figsize=(13, 4.0))
        axes[0].bar(NED_LABELS, gravity, color="tab:green")
        _decorate(axes[0], None, "Specific force [m/s²]", "Gravity reference in NED")
        axes[1].bar(NED_LABELS, earth_rate * 1e6, color="tab:orange")
        _decorate(axes[1], None, "Earth rate [µrad/s]", "Earth-rotation reference in NED")
        axes[2].axis("off")
        axes[2].text(
            0.0,
            0.95,
            "\n".join(
                [
                    f"Latitude          {float(task1['latitude_deg']):.6f}°",
                    f"Longitude         {float(task1['longitude_deg']):.6f}°",
                    f"Altitude          {float(task1['altitude_m']):.2f} m",
                    f"Normal gravity    {float(task1['normal_gravity_mps2']):.6f} m/s²",
                    f"Applied gravity   {float(task1['gravity_mps2']):.6f} m/s²",
                    f"Overridden        {bool(task1['gravity_was_overridden'])}",
                    f"Earth rate        {float(ctx['earth_rate_rps']):.6e} rad/s",
                    "",
                    "|g| points to NED Down (+z).",
                    "Earth rate has no East component by construction.",
                ]
            ),
            va="top",
            family="monospace",
            fontsize=9,
        )
        return fig

    writer.emit(1, "input_time_coverage", directory, coverage)
    writer.emit(1, "gnss_raw_position_velocity", directory, gnss_raw)
    writer.emit(1, "imu_raw_rates", directory, imu_raw_pair)
    writer.emit(1, "reference_origin_map", directory, origin_map)
    writer.emit(1, "gravity_earth_rate_vectors", directory, reference_vectors)


# ---------------------------------------------------------------------------
# Task 2
# ---------------------------------------------------------------------------
def draw_task2(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    imu = ctx["imu"]
    task2 = ctx["task2"]
    variances = ctx["variances"]
    start = int(task2["static_start_index"])
    end = int(task2["static_end_index_exclusive"])
    step = stride_for(len(imu.time_s), writer.max_points)

    def converted(plt):
        fig, axes = _triple_axes(plt, rows=2)
        for column in range(3):
            axes[0, column].plot(imu.time_s[::step], imu.gyro_rps[::step, column], linewidth=0.8, color="tab:blue")
            axes[1, column].plot(imu.time_s[::step], imu.accel_mps2[::step, column], linewidth=0.8, color="tab:red")
            _decorate(axes[0, column], None, "Angular rate [rad/s]" if column == 0 else None, f"Body {AXIS_LABELS[column]}")
            _decorate(axes[1, column], "Time [s]", "Specific force [m/s²]" if column == 0 else None)
        _annotate(axes[0, 0], 0.02, 0.9, f"measurement type: {ctx['imu_measurement_type']}")
        return fig

    def variance_scan(plt):
        fig, axis = plt.subplots(figsize=(11, 4.2))
        window_time = imu.time_s[: len(variances)]
        vstep = stride_for(len(window_time), writer.max_points)
        axis.semilogy(window_time[::vstep], np.maximum(variances[::vstep], 1e-18), linewidth=0.9)
        axis.axvline(imu.time_s[start], color="crimson", linestyle="--", label="selected window start")
        axis.scatter([imu.time_s[start]], [max(float(variances[start]), 1e-18)], color="crimson", zorder=3)
        _decorate(axis, "Window start time [s]", "Rolling feature variance (log)")
        axis.legend(fontsize=8)
        return fig

    def selection(plt):
        fig, axes = _triple_axes(plt, rows=2)
        for column in range(3):
            axes[0, column].plot(imu.time_s[::step], imu.accel_mps2[::step, column], linewidth=0.8, color="tab:red")
            axes[1, column].plot(imu.time_s[::step], imu.gyro_rps[::step, column], linewidth=0.8, color="tab:blue")
            for axis in (axes[0, column], axes[1, column]):
                axis.axvspan(imu.time_s[start], imu.time_s[end - 1], color="gold", alpha=0.35)
            _decorate(axes[0, column], None, "Specific force [m/s²]" if column == 0 else None, f"Body {AXIS_LABELS[column]}")
            _decorate(axes[1, column], "Time [s]", "Angular rate [rad/s]" if column == 0 else None)
        _annotate(
            axes[0, 0],
            0.02,
            0.05,
            f"static window {imu.time_s[start]:.3f}–{imu.time_s[end - 1]:.3f} s "
            f"({end - start} samples)",
        )
        return fig

    def body_vs_reference(plt):
        body = np.asarray(task2["body_vectors_unit"], dtype=float)
        reference = np.asarray(task2["reference_vectors_unit"], dtype=float)
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
        width = 0.36
        positions = np.arange(3)
        for index, (axis, name) in enumerate(zip(axes, ("Specific force", "Earth rate"))):
            axis.bar(positions - width / 2, body[index], width, label="measured (body)")
            axis.bar(positions + width / 2, reference[index], width, label="reference (NED)")
            axis.set_xticks(positions, [f"{AXIS_LABELS[i]} / {NED_LABELS[i]}" for i in range(3)])
            _decorate(axis, None, "Unit component" if index == 0 else None, name)
            axis.set_ylim(-1.1, 1.1)
            axis.legend(fontsize=8)
        return fig

    writer.emit(2, "converted_rates", directory, converted)
    writer.emit(2, "static_window_variance_scan", directory, variance_scan)
    writer.emit(2, "static_window_selection", directory, selection)
    writer.emit(2, "mean_body_vs_reference_vectors", directory, body_vs_reference)


# ---------------------------------------------------------------------------
# Task 3
# ---------------------------------------------------------------------------
def draw_task3(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    task3 = ctx["task3"]
    matrix = np.asarray(task3["c_body_to_ned"], dtype=float)
    quaternion = np.asarray(task3["quaternion_wxyz_body_to_ned"], dtype=float)

    def rotation_matrix(plt):
        fig, axis = plt.subplots(figsize=(7.0, 5.2))
        image = axis.imshow(matrix, cmap="coolwarm", vmin=-1, vmax=1)
        for row in range(3):
            for column in range(3):
                axis.text(column, row, f"{matrix[row, column]: .4f}", ha="center", va="center", fontsize=10)
        axis.set_xticks(range(3), [f"body {name}" for name in AXIS_LABELS])
        axis.set_yticks(range(3), NED_LABELS)
        axis.set_title(f"C_body→NED  ({ctx['method']})")
        fig.colorbar(image, ax=axis, shrink=0.8, label="matrix element")
        return fig

    def alignment_error(plt):
        fig, axis = plt.subplots(figsize=(8.0, 4.2))
        names = ("Gravity", "Earth rate")
        values = (float(task3["gravity_error_deg"]), float(task3["earth_rate_error_deg"]))
        bars = axis.bar(names, values, color=("tab:green", "tab:orange"))
        for bar, value in zip(bars, values):
            axis.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.4g}°", ha="center", va="bottom", fontsize=9)
        _decorate(axis, None, "Residual alignment error [deg]")
        axis.set_ylim(0, max(values) * 1.35 + 1e-6)
        return fig

    def quaternion_components(plt):
        fig, axis = plt.subplots(figsize=(8.0, 4.2))
        bars = axis.bar(QUATERNION_LABELS, quaternion, color="tab:blue")
        for bar, value in zip(bars, quaternion):
            axis.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.5f}", ha="center",
                      va="bottom" if value >= 0 else "top", fontsize=9)
        axis.set_ylim(-1.15, 1.15)
        axis.axhline(0, color="black", linewidth=0.6)
        _decorate(axis, None, "Component value")
        _annotate(axis, 0.02, 0.02, f"‖q‖ = {float(task3['quaternion_norm']):.12f}")
        return fig

    def euler(plt):
        angles = quaternion_series_to_euler_zyx_deg(quaternion.reshape(1, 4))[0]
        fig, axis = plt.subplots(figsize=(8.0, 4.2))
        bars = axis.bar(EULER_LABELS, angles, color=("tab:purple", "tab:cyan", "tab:olive"))
        for bar, value in zip(bars, angles):
            axis.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.4f}°", ha="center",
                      va="bottom" if value >= 0 else "top", fontsize=9)
        axis.axhline(0, color="black", linewidth=0.6)
        _decorate(axis, None, "Angle [deg]")
        return fig

    def biases(plt):
        accel = np.asarray(task3["accel_bias_body_mps2"], dtype=float)
        gyro = np.asarray(task3["gyro_bias_body_rps"], dtype=float)
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.2))
        axes[0].bar(AXIS_LABELS, accel, color="tab:red")
        _decorate(axes[0], "Body axis", "Accelerometer bias [m/s²]", "Accelerometer bias")
        axes[1].bar(AXIS_LABELS, gyro * 1e6, color="tab:blue")
        _decorate(axes[1], "Body axis", "Gyroscope bias [µrad/s]", "Gyroscope bias")
        for axis in axes:
            axis.axhline(0, color="black", linewidth=0.6)
        return fig

    writer.emit(3, "rotation_matrix", directory, rotation_matrix)
    writer.emit(3, "vector_alignment_error", directory, alignment_error)
    writer.emit(3, "initial_quaternion", directory, quaternion_components)
    writer.emit(3, "initial_euler_angles", directory, euler)
    writer.emit(3, "imu_bias_estimates", directory, biases)


# ---------------------------------------------------------------------------
# Task 4
# ---------------------------------------------------------------------------
def draw_task4(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    imu = ctx["imu"]
    task4 = ctx["task4"]
    time_s = task4["time_s"]
    step = stride_for(len(time_s), writer.max_points)
    t = time_s[::step]

    def screening(plt):
        fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
        axes[0].plot(t, np.linalg.norm(imu.accel_mps2[::step], axis=1), linewidth=0.8, color="tab:red")
        axes[0].axhline(ctx["max_specific_force"], color="black", linestyle="--", label="limit")
        _decorate(axes[0], None, "|specific force| [m/s²]")
        axes[0].legend(fontsize=8)
        axes[1].plot(t, np.linalg.norm(imu.gyro_rps[::step], axis=1), linewidth=0.8, color="tab:blue")
        axes[1].axhline(ctx["max_angular_rate"], color="black", linestyle="--", label="limit")
        _decorate(axes[1], "Time [s]", "|angular rate| [rad/s]")
        axes[1].legend(fontsize=8)
        screened = task4["range_screening"]
        _annotate(
            axes[0],
            0.02,
            0.9,
            f"interpolated: {screened['accelerometer_samples_interpolated']} accel, "
            f"{screened['gyroscope_samples_interpolated']} gyro",
        )
        return fig

    def bias_corrected(plt):
        accel_bias = np.asarray(ctx["accel_bias"], dtype=float)
        gyro_bias = np.asarray(ctx["gyro_bias"], dtype=float)
        fig, axes = _triple_axes(plt, rows=2)
        for column in range(3):
            axes[0, column].plot(t, ctx["accel_screened"][::step, column], linewidth=0.7, label="screened")
            axes[0, column].plot(t, ctx["accel_screened"][::step, column] - accel_bias[column], linewidth=0.7, label="bias removed")
            axes[1, column].plot(t, ctx["gyro_screened"][::step, column], linewidth=0.7, label="screened")
            axes[1, column].plot(t, ctx["gyro_screened"][::step, column] - gyro_bias[column], linewidth=0.7, label="bias removed")
            _decorate(axes[0, column], None, "Specific force [m/s²]" if column == 0 else None, f"Body {AXIS_LABELS[column]}")
            _decorate(axes[1, column], "Time [s]", "Angular rate [rad/s]" if column == 0 else None)
        axes[0, 0].legend(fontsize=8)
        return fig

    def propagated_quaternion(plt):
        quaternion = task4["quaternion"][::step]
        fig, axes = plt.subplots(2, 2, figsize=(11, 6.4), sharex=True)
        for index, axis in enumerate(axes.flat):
            axis.plot(t, quaternion[:, index], linewidth=0.9)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
            _decorate(axis, "Time [s]" if index >= 2 else None, f"q{QUATERNION_LABELS[index]}")
        return fig

    def propagated_euler(plt):
        angles = quaternion_series_to_euler_zyx_deg(task4["quaternion"])[::step]
        fig = _time_series_triple(
            plt,
            t,
            [("IMU-only", angles, {"linewidth": 0.9})],
            EULER_LABELS,
            "Angle [deg]",
        )
        _note_euler_caveats(fig, angles)
        return fig

    def position_velocity(plt):
        return _state_grid(
            plt,
            t,
            [("IMU-only", task4["position"][::step], task4["velocity"][::step], {"linewidth": 0.9})],
        )

    def acceleration(plt):
        return _time_series_triple(
            plt,
            t,
            [("IMU-only", task4["acceleration"][::step], {"linewidth": 0.7})],
            NED_LABELS,
            "Acceleration [m/s²]",
        )

    def ground_track(plt):
        position = task4["position"][::step]
        fig, axis = plt.subplots(figsize=(7.5, 6.2))
        axis.plot(position[:, 1], position[:, 0], linewidth=1.0)
        axis.scatter([position[0, 1]], [position[0, 0]], color="green", zorder=3, label="start")
        axis.scatter([position[-1, 1]], [position[-1, 0]], color="crimson", zorder=3, label="end")
        axis.set_aspect("equal", adjustable="datalim")
        _decorate(axis, "East [m]", "North [m]")
        axis.legend(fontsize=8)
        return fig

    writer.emit(4, "range_screening", directory, screening)
    writer.emit(4, "bias_corrected_imu", directory, bias_corrected)
    writer.emit(4, "propagated_quaternion", directory, propagated_quaternion)
    writer.emit(4, "propagated_euler_angles", directory, propagated_euler)
    writer.emit(4, "imu_only_position_velocity", directory, position_velocity)
    writer.emit(4, "imu_only_acceleration", directory, acceleration)
    writer.emit(4, "imu_only_ground_track", directory, ground_track)


# ---------------------------------------------------------------------------
# Task 5
# ---------------------------------------------------------------------------
def draw_task5(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    task4, task5 = ctx["task4"], ctx["task5"]
    time_s = task5["time_s"]
    step = stride_for(len(time_s), writer.max_points)
    t = time_s[::step]
    gnss_time = ctx["gnss_time_s"]
    gnss_position = ctx["gnss_position_ned"]
    gnss_velocity = ctx["gnss_velocity_ned"]

    def prediction_vs_gnss(plt):
        fig, axes = _triple_axes(plt, rows=2)
        for column in range(3):
            axes[0, column].plot(t, task5["position"][::step, column], linewidth=0.9, label="filter state")
            axes[0, column].scatter(gnss_time, gnss_position[:, column], s=9, color="crimson", zorder=3, label="GNSS")
            axes[1, column].plot(t, task5["velocity"][::step, column], linewidth=0.9, label="filter state")
            axes[1, column].scatter(gnss_time, gnss_velocity[:, column], s=9, color="crimson", zorder=3, label="GNSS")
            _decorate(axes[0, column], None, "Position [m]" if column == 0 else None, NED_LABELS[column])
            _decorate(axes[1, column], "Time [s]", "Velocity [m/s]" if column == 0 else None)
        axes[0, 0].legend(fontsize=8)
        return fig

    def innovations(plt):
        innovation_times = task5["innovation_times"]
        values = task5["innovations"]
        fig, axes = _triple_axes(plt, rows=2, sharex=True)
        if values.size == 0:
            for row in range(2):
                for column in range(3):
                    axes[row, column].text(0.5, 0.5, "no GNSS updates", ha="center", va="center", transform=axes[row, column].transAxes)
            return fig
        for column in range(3):
            axes[0, column].plot(innovation_times, values[:, column], marker="o", markersize=2.5, linewidth=0.7)
            axes[1, column].plot(innovation_times, values[:, column + 3], marker="o", markersize=2.5, linewidth=0.7, color="tab:orange")
            for axis in (axes[0, column], axes[1, column]):
                axis.axhline(0, color="black", linewidth=0.6)
            _decorate(axes[0, column], None, "Position innovation [m]" if column == 0 else None, NED_LABELS[column])
            _decorate(axes[1, column], "Time [s]", "Velocity innovation [m/s]" if column == 0 else None)
        return fig

    def fused(plt):
        return _state_grid(
            plt,
            t,
            [("Fused", task5["position"][::step], task5["velocity"][::step], {"linewidth": 0.9})],
        )

    def ground_track(plt):
        position = task5["position"][::step]
        fig, axis = plt.subplots(figsize=(7.5, 6.2))
        axis.plot(position[:, 1], position[:, 0], linewidth=1.0, label="fused")
        axis.scatter(gnss_position[:, 1], gnss_position[:, 0], s=14, color="crimson", zorder=3, label="GNSS fixes")
        axis.set_aspect("equal", adjustable="datalim")
        _decorate(axis, "East [m]", "North [m]")
        axis.legend(fontsize=8)
        return fig

    def fused_vs_imu(plt):
        return _state_grid(
            plt,
            t,
            [
                ("Fused", task5["position"][::step], task5["velocity"][::step], {"linewidth": 0.9}),
                ("IMU-only", task4["position"][::step], task4["velocity"][::step], {"linewidth": 0.8, "linestyle": "--"}),
            ],
        )

    writer.emit(5, "prediction_vs_gnss", directory, prediction_vs_gnss)
    writer.emit(5, "kalman_innovations", directory, innovations)
    writer.emit(5, "fused_position_velocity", directory, fused)
    writer.emit(5, "fused_ground_track", directory, ground_track)
    writer.emit(5, "fused_vs_imu_only", directory, fused_vs_imu)


# ---------------------------------------------------------------------------
# Task 6
# ---------------------------------------------------------------------------
def _skip_all(writer: FigureWriter, task_number: int, slugs: Iterable[str], reason: str) -> None:
    for slug in slugs:
        writer.skip(task_number, slug, reason)


TASK6_FIGURES = (
    "truth_time_alignment",
    "fused_vs_truth_position_velocity",
    "fused_vs_truth_ground_track",
    "height_above_origin",
    "quaternion_comparison",
    "euler_angle_comparison",
)


def draw_task6(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    overlay = ctx["overlay"]
    time_s = overlay["time_s"]
    step = stride_for(len(time_s), writer.max_points)
    t = time_s[::step]
    has_quaternion = "truth_quaternion" in overlay

    def alignment(plt):
        fig, axis = plt.subplots(figsize=(11, 3.6))
        entries = [
            ("Estimate (Task 5)", ctx["estimate_time_s"]),
            ("Truth (as loaded)", ctx["truth_time_s"]),
            ("Common overlap", time_s),
        ]
        for index, (name, series) in enumerate(entries):
            axis.barh(index, series[-1] - series[0], left=series[0], height=0.45, alpha=0.75)
        axis.set_yticks(range(len(entries)), [name for name, _ in entries])
        _decorate(axis, "Time [s]", None)
        _annotate(
            axis,
            0.01,
            0.04,
            f"attitude-only time offset applied: {ctx['attitude_offset_s']:+.3f} s   |   "
            f"{len(time_s)} common samples",
        )
        return fig

    def position_velocity(plt):
        return _state_grid(
            plt,
            t,
            [
                ("Fused", overlay["estimated_position"][::step], overlay["estimated_velocity"][::step], {"linewidth": 0.9}),
                ("Truth", overlay["truth_position"][::step], overlay["truth_velocity"][::step], {"linewidth": 0.9, "linestyle": "--"}),
            ],
        )

    def ground_track(plt):
        estimated = overlay["estimated_position"][::step]
        truth = overlay["truth_position"][::step]
        fig, axis = plt.subplots(figsize=(7.5, 6.2))
        axis.plot(estimated[:, 1], estimated[:, 0], linewidth=1.0, label="fused")
        axis.plot(truth[:, 1], truth[:, 0], linewidth=1.0, linestyle="--", label="truth")
        axis.set_aspect("equal", adjustable="datalim")
        _decorate(axis, "East [m]", "North [m]")
        axis.legend(fontsize=8)
        return fig

    def height(plt):
        fig, axes = plt.subplots(2, 1, figsize=(11, 6.2), sharex=True, height_ratios=(2, 1))
        axes[0].plot(t, -overlay["estimated_position"][::step, 2], linewidth=0.9, label="fused")
        axes[0].plot(t, -overlay["truth_position"][::step, 2], linewidth=0.9, linestyle="--", label="truth")
        _decorate(axes[0], None, "Height above origin [m]")
        axes[0].legend(fontsize=8)
        difference = -(overlay["estimated_position"][::step, 2] - overlay["truth_position"][::step, 2])
        axes[1].plot(t, difference, linewidth=0.8, color="tab:red")
        axes[1].axhline(0, color="black", linewidth=0.6)
        _decorate(axes[1], "Time [s]", "Fused − truth [m]")
        return fig

    def quaternion(plt):
        estimated = overlay["estimated_quaternion"][::step]
        truth = overlay["truth_quaternion"][::step]
        fig, axes = plt.subplots(2, 2, figsize=(11, 6.6), sharex=True)
        for index, axis in enumerate(axes.flat):
            axis.plot(t, estimated[:, index], linewidth=0.9, label="fused")
            axis.plot(t, truth[:, index], linewidth=0.9, linestyle="--", label="truth")
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
            _decorate(axis, "Time [s]" if index >= 2 else None, f"q{QUATERNION_LABELS[index]}")
        axes[0, 0].legend(fontsize=8)
        return fig

    def euler(plt):
        estimated = quaternion_series_to_euler_zyx_deg(overlay["estimated_quaternion"])[::step]
        truth = quaternion_series_to_euler_zyx_deg(overlay["truth_quaternion"])[::step]
        fig = _time_series_triple(
            plt,
            t,
            [
                ("Fused", estimated, {"linewidth": 0.9}),
                ("Truth", truth, {"linewidth": 0.9, "linestyle": "--"}),
            ],
            EULER_LABELS,
            "Angle [deg]",
        )
        _note_euler_caveats(fig, estimated, truth)
        return fig

    writer.emit(6, "truth_time_alignment", directory, alignment)
    writer.emit(6, "fused_vs_truth_position_velocity", directory, position_velocity)
    writer.emit(6, "fused_vs_truth_ground_track", directory, ground_track)
    writer.emit(6, "height_above_origin", directory, height)
    if has_quaternion:
        writer.emit(6, "quaternion_comparison", directory, quaternion)
        writer.emit(6, "euler_angle_comparison", directory, euler)
    else:
        _skip_all(writer, 6, ("quaternion_comparison", "euler_angle_comparison"),
                  "truth file has no quaternion columns")


def skip_task6(writer: FigureWriter, reason: str) -> None:
    _skip_all(writer, 6, TASK6_FIGURES, reason)


# ---------------------------------------------------------------------------
# Task 7
# ---------------------------------------------------------------------------
TASK7_TRUTH_FIGURES = (
    "position_residuals",
    "position_error_distribution",
    "velocity_residuals",
    "attitude_error",
)


def draw_task7(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    metrics = ctx["metrics"]

    if ctx.get("overlay") is not None:
        overlay = ctx["overlay"]
        time_s = overlay["time_s"]
        step = stride_for(len(time_s), writer.max_points)
        t = time_s[::step]
        position_error = ctx["position_error"]
        velocity_error = ctx["velocity_error"]

        def position_residuals(plt):
            fig = _time_series_triple(
                plt,
                t,
                [("Fused − truth", position_error[::step], {"linewidth": 0.8})],
                NED_LABELS,
                "Position error [m]",
            )
            for axis in fig.axes:
                axis.axhline(0, color="black", linewidth=0.6)
            return fig

        def velocity_residuals(plt):
            fig = _time_series_triple(
                plt,
                t,
                [("Fused − truth", velocity_error[::step], {"linewidth": 0.8})],
                NED_LABELS,
                "Velocity error [m/s]",
            )
            for axis in fig.axes:
                axis.axhline(0, color="black", linewidth=0.6)
            return fig

        def distribution(plt):
            norm = np.linalg.norm(position_error, axis=1)
            fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
            axes[0].hist(norm, bins=60, color="tab:blue", alpha=0.85)
            _decorate(axes[0], "Position error norm [m]", "Samples", "Histogram")
            ordered = np.sort(norm)
            axes[1].plot(ordered, np.linspace(0, 100, ordered.size), linewidth=1.1)
            for percentile in (50, 95):
                value = float(np.percentile(norm, percentile))
                axes[1].axvline(value, linestyle="--", linewidth=0.8,
                                label=f"p{percentile} = {value:.3f} m")
            _decorate(axes[1], "Position error norm [m]", "Percentile [%]", "Cumulative distribution")
            axes[1].legend(fontsize=8)
            return fig

        writer.emit(7, "position_residuals", directory, position_residuals)
        writer.emit(7, "position_error_distribution", directory, distribution)
        writer.emit(7, "velocity_residuals", directory, velocity_residuals)

        if ctx.get("attitude_error_deg") is not None:
            attitude = ctx["attitude_error_deg"]

            def attitude_error(plt):
                fig, axis = plt.subplots(figsize=(11, 4.4))
                axis.plot(t, attitude[::step], linewidth=0.8, color="tab:purple")
                axis.axhline(float(metrics["attitude_rmse_deg"]), color="black", linestyle="--",
                             linewidth=0.8, label=f"RMSE = {float(metrics['attitude_rmse_deg']):.4f}°")
                _decorate(axis, "Time [s]", "Attitude error [deg]")
                axis.legend(fontsize=8)
                return fig

            writer.emit(7, "attitude_error", directory, attitude_error)
        else:
            writer.skip(7, "attitude_error", "truth file has no quaternion columns")
    else:
        _skip_all(writer, 7, TASK7_TRUTH_FIGURES, ctx.get("skip_reason", "no truth file supplied"))

    def metric_summary(plt):
        items = [(key, value) for key, value in metrics.items() if isinstance(value, (int, float))]
        fig, axis = plt.subplots(figsize=(11, max(3.6, 0.42 * len(items) + 1.6)))
        if not items:
            axis.axis("off")
            axis.text(0.5, 0.5, "no scalar metrics available", ha="center", va="center")
            return fig
        names = [key for key, _ in items]
        values = [float(value) for _, value in items]
        positions = np.arange(len(items))
        axis.barh(positions, values, color="tab:blue")
        axis.set_yticks(positions, names, fontsize=8)
        axis.invert_yaxis()
        for position, value in zip(positions, values):
            axis.text(value, position, f" {value:.6g}", va="center", fontsize=8)
        axis.set_xscale("symlog", linthresh=1e-4)
        _decorate(axis, "Metric value (symlog scale)", None)
        return fig

    writer.emit(7, "metric_summary", directory, metric_summary)

    innovations = ctx.get("innovations")
    if innovations is not None and getattr(innovations, "size", 0):
        def innovation_summary(plt):
            position_norm = np.linalg.norm(innovations[:, :3], axis=1)
            velocity_norm = np.linalg.norm(innovations[:, 3:], axis=1)
            fig, axes = plt.subplots(1, 2, figsize=(12, 4.4))
            axes[0].hist(position_norm, bins=min(40, max(5, position_norm.size // 2)), color="tab:blue", alpha=0.85)
            _decorate(axes[0], "|position innovation| [m]", "Updates", "GNSS position innovations")
            axes[1].hist(velocity_norm, bins=min(40, max(5, velocity_norm.size // 2)), color="tab:orange", alpha=0.85)
            _decorate(axes[1], "|velocity innovation| [m/s]", "Updates", "GNSS velocity innovations")
            return fig

        writer.emit(7, "innovation_summary", directory, innovation_summary)
    else:
        writer.skip(7, "innovation_summary", "no GNSS updates were applied")


# ---------------------------------------------------------------------------
# Cross-method comparison
# ---------------------------------------------------------------------------
def draw_comparison(writer: FigureWriter, directory: Path, ctx: dict[str, Any]) -> None:
    results: dict[str, dict[str, Any]] = ctx["results"]
    methods = list(results)

    def initial_attitude(plt):
        fig, axes = plt.subplots(1, 2, figsize=(12.5, 4.6))
        width = 0.8 / max(len(methods), 1)
        positions = np.arange(4)
        for index, method in enumerate(methods):
            task3 = results[method]["tasks"].get(3)
            if task3 is None:
                continue
            quaternion = np.asarray(task3["quaternion_wxyz_body_to_ned"], dtype=float)
            axes[0].bar(positions + index * width - 0.4 + width / 2, quaternion, width, label=method)
        axes[0].set_xticks(positions, [f"q{name}" for name in QUATERNION_LABELS])
        axes[0].axhline(0, color="black", linewidth=0.6)
        axes[0].set_ylim(-1.15, 1.15)
        _decorate(axes[0], None, "Component value", "Initial quaternion")
        axes[0].legend(fontsize=8)
        error_positions = np.arange(2)
        for index, method in enumerate(methods):
            task3 = results[method]["tasks"].get(3)
            if task3 is None:
                continue
            values = [float(task3["gravity_error_deg"]), float(task3["earth_rate_error_deg"])]
            axes[1].bar(error_positions + index * width - 0.4 + width / 2, values, width, label=method)
        axes[1].set_xticks(error_positions, ["Gravity", "Earth rate"])
        axes[1].set_yscale("symlog", linthresh=1e-9)
        _decorate(axes[1], None, "Alignment error [deg]", "Wahba residual")
        axes[1].legend(fontsize=8)
        return fig

    def fused_position(plt):
        fig, axes = _triple_axes(plt)
        for method in methods:
            task5 = results[method]["tasks"].get(5)
            if task5 is None:
                continue
            step = stride_for(len(task5["time_s"]), writer.max_points)
            for column in range(3):
                axes[0, column].plot(task5["time_s"][::step], task5["position"][::step, column],
                                     linewidth=0.9, label=method)
        for column in range(3):
            _decorate(axes[0, column], "Time [s]", "Position [m]" if column == 0 else None, NED_LABELS[column])
        axes[0, 0].legend(fontsize=8)
        return fig

    def position_error(plt):
        fig, axes = plt.subplots(2, 1, figsize=(11, 6.4), sharex=True)
        for method in methods:
            task6 = results[method]["tasks"].get(6, {})
            if task6.get("status") != "complete":
                continue
            overlay = task6["overlay"]
            step = stride_for(len(overlay["time_s"]), writer.max_points)
            error = overlay["estimated_position"] - overlay["truth_position"]
            axes[0].plot(overlay["time_s"][::step], np.linalg.norm(error, axis=1)[::step],
                         linewidth=0.8, label=method)
            axes[1].plot(overlay["time_s"][::step], -error[::step, 2], linewidth=0.8, label=method)
        _decorate(axes[0], None, "‖position error‖ [m]")
        _decorate(axes[1], "Time [s]", "Height error [m]")
        axes[0].legend(fontsize=8)
        return fig

    def attitude_error(plt):
        fig, axis = plt.subplots(figsize=(11, 4.4))
        for method in methods:
            task6 = results[method]["tasks"].get(6, {})
            if task6.get("status") != "complete":
                continue
            overlay = task6["overlay"]
            if "truth_quaternion" not in overlay:
                continue
            step = stride_for(len(overlay["time_s"]), writer.max_points)
            dots = np.clip(np.abs(np.sum(overlay["estimated_quaternion"] * overlay["truth_quaternion"], axis=1)), 0.0, 1.0)
            axis.plot(overlay["time_s"][::step], (2.0 * np.degrees(np.arccos(dots)))[::step],
                      linewidth=0.8, label=method)
        _decorate(axis, "Time [s]", "Attitude error [deg]")
        axis.legend(fontsize=8)
        return fig

    def metric_bars(plt):
        metrics = ctx["metrics"]
        keys = sorted({key for values in metrics.values() for key, value in values.items()
                       if isinstance(value, (int, float))})
        if not keys:
            fig, axis = plt.subplots(figsize=(8.0, 3.6))
            axis.axis("off")
            axis.text(0.5, 0.5, "no comparable metrics", ha="center", va="center")
            return fig
        # Wrap into a grid; a single row of a dozen metrics is unreadably wide.
        columns = min(4, len(keys))
        rows = int(np.ceil(len(keys) / columns))
        fig, axes = plt.subplots(rows, columns, figsize=(3.4 * columns, 3.1 * rows + 1.0), squeeze=False)
        palette = ["tab:blue", "tab:orange", "tab:green"]
        for index, key in enumerate(keys):
            axis = axes[index // columns][index % columns]
            values = [float(metrics[method].get(key, np.nan)) for method in methods]
            axis.bar(methods, values, color=[palette[i % len(palette)] for i in range(len(methods))])
            axis.set_title(key, fontsize=9)
            axis.tick_params(axis="x", labelrotation=20, labelsize=8)
            axis.tick_params(axis="y", labelsize=8)
            axis.grid(True, axis="y", alpha=0.3)
        for index in range(len(keys), rows * columns):
            axes[index // columns][index % columns].axis("off")
        return fig

    writer.emit(0, "initial_attitude_by_method", directory, initial_attitude)
    writer.emit(0, "fused_position_by_method", directory, fused_position)
    has_truth_overlay = any(
        results[method]["tasks"].get(6, {}).get("status") == "complete" for method in methods
    )
    if has_truth_overlay:
        writer.emit(0, "position_error_by_method", directory, position_error)
        has_attitude = any(
            "truth_quaternion" in results[method]["tasks"].get(6, {}).get("overlay", {})
            for method in methods
            if results[method]["tasks"].get(6, {}).get("status") == "complete"
        )
        if has_attitude:
            writer.emit(0, "attitude_error_by_method", directory, attitude_error)
        else:
            writer.skip(0, "attitude_error_by_method", "truth file has no quaternion columns")
    else:
        writer.skip(0, "position_error_by_method", "no truth overlay available")
        writer.skip(0, "attitude_error_by_method", "no truth overlay available")
    writer.emit(0, "metric_bars_by_method", directory, metric_bars)
