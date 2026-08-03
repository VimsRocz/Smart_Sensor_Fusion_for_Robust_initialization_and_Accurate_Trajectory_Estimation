"""Single source of truth for task names, subtask names, and frame tags.

Every directory name, JSON summary, figure filename, and documentation table is
generated from :data:`TASKS` so the seven-task structure can never drift between
the code, the output tree, and the README.
"""

from __future__ import annotations

from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# Coordinate frames
# ---------------------------------------------------------------------------
# The tag is what appears in a figure filename; the description is what appears
# in the figure sub-title so a plot lifted into a report stays unambiguous.
FRAMES: dict[str, str] = {
    "ECEF": "Earth-Centred Earth-Fixed (WGS-84), metres",
    "GEODETIC": "Geodetic WGS-84 latitude/longitude/altitude",
    "BODY": "IMU body frame (x-forward, y-right, z-down)",
    "NED": "Local-level North-East-Down at the Task 1 origin",
    "BODY2NED": "Body-to-NED rotation (quaternion / Euler / DCM)",
    "BODYvsNED": "Measured body vectors against their NED references",
    "NONE": "Frame-independent quantity",
}


def frame_description(tag: str) -> str:
    """Return the human-readable meaning of a frame tag."""
    try:
        return FRAMES[tag]
    except KeyError as exc:  # pragma: no cover - guarded by _validate_catalog
        raise ValueError(f"Unknown coordinate frame tag {tag!r}") from exc


# ---------------------------------------------------------------------------
# Sensor-dataset tags
# ---------------------------------------------------------------------------
# Which input files a figure actually draws from. Combined tags are joined with
# "+" in the filename, e.g. data-IMU_X001_small+GNSS_X001_small.
SENSOR_IMU = "imu"
SENSOR_GNSS = "gnss"
SENSOR_TRUTH = "truth"
SENSOR_SOURCES = (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH)


@dataclass(frozen=True)
class FigureSpec:
    """One figure: what it shows, in which frame, from which sensor files."""

    subtask: str
    slug: str
    title: str
    frame: str
    sources: tuple[str, ...]
    #: Figures that need a truth file are skipped (and recorded as skipped)
    #: instead of failing when no truth file was supplied.
    requires_truth: bool = False


@dataclass(frozen=True)
class SubtaskSpec:
    number: str
    name: str


@dataclass(frozen=True)
class TaskSpec:
    number: int
    slug: str
    name: str
    purpose: str
    subtasks: tuple[SubtaskSpec, ...]
    figures: tuple[FigureSpec, ...] = field(default=())

    @property
    def directory_name(self) -> str:
        return f"task_{self.number:02d}_{self.slug}"

    @property
    def subtask_map(self) -> dict[str, str]:
        return {item.number: item.name for item in self.subtasks}


TASKS: tuple[TaskSpec, ...] = (
    TaskSpec(
        number=1,
        slug="inputs_reference",
        name="Inputs and navigation reference",
        purpose=(
            "Validate every input file against the documented contract and build the "
            "WGS-84 reference used by all later tasks."
        ),
        subtasks=(
            SubtaskSpec("1.1", "Validate IMU, GNSS and truth structure, units and timing"),
            SubtaskSpec("1.2", "Derive the WGS-84 origin and ECEF to NED rotation from the first GNSS fix"),
            SubtaskSpec("1.3", "Compute local normal gravity and the Earth-rotation vector in NED"),
        ),
        figures=(
            FigureSpec("1.1", "input_time_coverage", "Input time coverage and sample rates", "NONE", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH)),
            FigureSpec("1.1", "gnss_raw_position_velocity", "Raw GNSS position and velocity as supplied", "ECEF", (SENSOR_GNSS,)),
            FigureSpec("1.1", "imu_raw_rates", "IMU angular rate and specific force after unit handling", "BODY", (SENSOR_IMU,)),
            FigureSpec("1.2", "reference_origin_map", "Reference origin on the WGS-84 graticule", "GEODETIC", (SENSOR_GNSS,)),
            FigureSpec("1.3", "gravity_earth_rate_vectors", "Reference gravity and Earth-rate vectors", "NED", (SENSOR_GNSS,)),
        ),
    ),
    TaskSpec(
        number=2,
        slug="static_imu",
        name="Static interval and measured body vectors",
        purpose=(
            "Find the quietest stretch of IMU data and average it into the two body "
            "vectors that Wahba's problem needs."
        ),
        subtasks=(
            SubtaskSpec("2.1", "Convert delta-angle/delta-velocity increments into SI rates"),
            SubtaskSpec("2.2", "Select the minimum-variance static window"),
            SubtaskSpec("2.3", "Average and normalise the specific-force and Earth-rate body vectors"),
        ),
        figures=(
            FigureSpec("2.1", "converted_rates", "IMU angular rate and specific force used by Task 2", "BODY", (SENSOR_IMU,)),
            FigureSpec("2.2", "static_window_variance_scan", "Rolling-variance detector across the whole log", "BODY", (SENSOR_IMU,)),
            FigureSpec("2.2", "static_window_selection", "Selected static window on the IMU record", "BODY", (SENSOR_IMU,)),
            FigureSpec("2.3", "mean_body_vs_reference_vectors", "Averaged body vectors against their NED references", "BODYvsNED", (SENSOR_IMU, SENSOR_GNSS)),
        ),
    ),
    TaskSpec(
        number=3,
        slug="attitude_init",
        name="Initial attitude and IMU biases",
        purpose=(
            "Solve Wahba's problem with the selected method and turn the resulting "
            "attitude into accelerometer and gyroscope bias estimates."
        ),
        subtasks=(
            SubtaskSpec("3.1", "Solve the Body-to-NED alignment with TRIAD, Davenport or SVD"),
            SubtaskSpec("3.2", "Project onto SO(3) and normalise the scalar-first quaternion"),
            SubtaskSpec("3.3", "Estimate accelerometer and gyroscope bias in the solved attitude"),
        ),
        figures=(
            FigureSpec("3.1", "rotation_matrix", "Solved Body-to-NED direction cosine matrix", "BODY2NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("3.1", "vector_alignment_error", "Residual alignment error per observed vector", "BODY2NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("3.2", "initial_quaternion", "Initial quaternion components [w,x,y,z]", "BODY2NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("3.2", "initial_euler_angles", "Initial yaw, pitch and roll", "BODY2NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("3.3", "imu_bias_estimates", "Estimated accelerometer and gyroscope bias", "BODY", (SENSOR_IMU,)),
        ),
    ),
    TaskSpec(
        number=4,
        slug="inertial_propagation",
        name="IMU-only strapdown propagation",
        purpose=(
            "Integrate the bias-corrected IMU alone so the inertial solution can be "
            "judged before GNSS is allowed to correct it."
        ),
        subtasks=(
            SubtaskSpec("4.1", "Screen physical-range outliers and remove the Task 3 biases"),
            SubtaskSpec("4.2", "Propagate the Body-to-NED quaternion with Earth and transport rates"),
            SubtaskSpec("4.3", "Apply Coriolis compensation and integrate the NED state"),
        ),
        figures=(
            FigureSpec("4.1", "range_screening", "Specific-force and angular-rate magnitudes against their limits", "BODY", (SENSOR_IMU,)),
            FigureSpec("4.1", "bias_corrected_imu", "IMU signals before and after bias removal", "BODY", (SENSOR_IMU,)),
            FigureSpec("4.2", "propagated_quaternion", "Propagated Body-to-NED quaternion history", "BODY2NED", (SENSOR_IMU,)),
            FigureSpec("4.2", "propagated_euler_angles", "Propagated yaw, pitch and roll history", "BODY2NED", (SENSOR_IMU,)),
            FigureSpec("4.3", "imu_only_position_velocity", "IMU-only position and velocity", "NED", (SENSOR_IMU,)),
            FigureSpec("4.3", "imu_only_acceleration", "IMU-only resolved acceleration", "NED", (SENSOR_IMU,)),
            FigureSpec("4.3", "imu_only_ground_track", "IMU-only horizontal ground track", "NED", (SENSOR_IMU,)),
        ),
    ),
    TaskSpec(
        number=5,
        slug="gnss_imu_fusion",
        name="GNSS/IMU Kalman fusion",
        purpose=(
            "Run the six-state position/velocity filter that corrects the inertial "
            "solution at every GNSS epoch."
        ),
        subtasks=(
            SubtaskSpec("5.1", "Predict the six-state NED position/velocity from IMU acceleration"),
            SubtaskSpec("5.2", "Apply Joseph-form updates at each asynchronous GNSS epoch"),
            SubtaskSpec("5.3", "Export the fused state and the innovation history"),
        ),
        figures=(
            FigureSpec("5.1", "prediction_vs_gnss", "Filter prediction against the GNSS measurements", "NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("5.2", "kalman_innovations", "Position and velocity innovations at each update", "NED", (SENSOR_GNSS,)),
            FigureSpec("5.3", "fused_position_velocity", "Fused position and velocity", "NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("5.3", "fused_ground_track", "Fused horizontal ground track with GNSS fixes", "NED", (SENSOR_IMU, SENSOR_GNSS)),
            FigureSpec("5.3", "fused_vs_imu_only", "Fused solution against the IMU-only solution", "NED", (SENSOR_IMU, SENSOR_GNSS)),
        ),
    ),
    TaskSpec(
        number=6,
        slug="truth_overlay",
        name="Truth overlay in a common frame",
        purpose=(
            "Put truth and estimate in one frame, one time base, and one quaternion "
            "convention before anything is compared."
        ),
        subtasks=(
            SubtaskSpec("6.1", "Restrict to the common time range and apply the attitude-only time offset"),
            SubtaskSpec("6.2", "Convert truth ECEF position/velocity with the Task 1 origin and rotation"),
            SubtaskSpec("6.3", "Compare relative height defined as negative NED Down"),
            SubtaskSpec("6.4", "Normalise and hemisphere-align the Body-to-NED quaternions"),
        ),
        figures=(
            FigureSpec("6.1", "truth_time_alignment", "Truth and estimate time coverage after alignment", "NONE", (SENSOR_IMU, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("6.2", "fused_vs_truth_position_velocity", "Fused and truth position and velocity", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("6.2", "fused_vs_truth_ground_track", "Fused and truth horizontal ground track", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("6.3", "height_above_origin", "Relative height, defined as negative NED Down", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("6.4", "quaternion_comparison", "Fused and truth quaternion components", "BODY2NED", (SENSOR_IMU, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("6.4", "euler_angle_comparison", "Fused and truth yaw, pitch and roll", "BODY2NED", (SENSOR_IMU, SENSOR_TRUTH), requires_truth=True),
        ),
    ),
    TaskSpec(
        number=7,
        slug="evaluation",
        name="Residual evaluation and quality metrics",
        purpose=(
            "Reduce the Task 6 overlay to residual time histories and the scalar "
            "metrics used to rank the three methods."
        ),
        subtasks=(
            SubtaskSpec("7.1", "Compute NED position residuals"),
            SubtaskSpec("7.2", "Compute NED velocity residuals"),
            SubtaskSpec("7.3", "Compute the sign-invariant quaternion geodesic error"),
            SubtaskSpec("7.4", "Export the scalar comparison metrics"),
        ),
        figures=(
            FigureSpec("7.1", "position_residuals", "Fused minus truth position residuals", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("7.1", "position_error_distribution", "Position error norm distribution", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("7.2", "velocity_residuals", "Fused minus truth velocity residuals", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("7.3", "attitude_error", "Quaternion geodesic attitude error", "BODY2NED", (SENSOR_IMU, SENSOR_TRUTH), requires_truth=True),
            FigureSpec("7.4", "metric_summary", "Scalar accuracy metrics for this method", "NONE", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH)),
            FigureSpec("7.4", "innovation_summary", "GNSS innovation statistics used when truth is absent", "NED", (SENSOR_GNSS,)),
        ),
    ),
)

TASK_BY_NUMBER: dict[int, TaskSpec] = {task.number: task for task in TASKS}


# ---------------------------------------------------------------------------
# Cross-method comparison figures (written once per all-method run)
# ---------------------------------------------------------------------------
COMPARISON_SLUG = "comparison"

COMPARISON_FIGURES: tuple[FigureSpec, ...] = (
    FigureSpec("C.1", "initial_attitude_by_method", "Initial attitude solved by each method", "BODY2NED", (SENSOR_IMU, SENSOR_GNSS)),
    FigureSpec("C.2", "fused_position_by_method", "Fused position from each method", "NED", (SENSOR_IMU, SENSOR_GNSS)),
    FigureSpec("C.3", "position_error_by_method", "Position error norm from each method", "NED", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH), requires_truth=True),
    FigureSpec("C.4", "attitude_error_by_method", "Attitude error from each method", "BODY2NED", (SENSOR_IMU, SENSOR_TRUTH), requires_truth=True),
    FigureSpec("C.5", "metric_bars_by_method", "Task 7 metrics side by side", "NONE", (SENSOR_IMU, SENSOR_GNSS, SENSOR_TRUTH)),
)


def _validate_catalog() -> None:
    """Fail at import time if the catalog is internally inconsistent."""
    numbers = [task.number for task in TASKS]
    if numbers != list(range(1, len(TASKS) + 1)):
        raise ValueError("TASKS must be numbered 1..N without gaps")
    for task in TASKS:
        declared = {item.number for item in task.subtasks}
        for figure in task.figures:
            if figure.subtask not in declared:
                raise ValueError(
                    f"Task {task.number} figure {figure.slug!r} references undeclared subtask {figure.subtask}"
                )
            if figure.frame not in FRAMES:
                raise ValueError(f"Figure {figure.slug!r} uses unknown frame {figure.frame!r}")
            unknown = set(figure.sources) - set(SENSOR_SOURCES)
            if unknown:
                raise ValueError(f"Figure {figure.slug!r} uses unknown sensor sources {sorted(unknown)}")
        slugs = [figure.slug for figure in task.figures]
        if len(set(slugs)) != len(slugs):
            raise ValueError(f"Task {task.number} has duplicate figure slugs")
    comparison_slugs = [figure.slug for figure in COMPARISON_FIGURES]
    if len(set(comparison_slugs)) != len(comparison_slugs):
        raise ValueError("COMPARISON_FIGURES has duplicate slugs")


_validate_catalog()


def describe_catalog() -> str:
    """Render the full task/subtask/figure map as plain text for the CLI."""
    lines: list[str] = ["Task, subtask and figure catalog", ""]
    for task in TASKS:
        lines.append(f"Task {task.number} — {task.name}")
        lines.append(f"  output directory : {task.directory_name}/")
        lines.append(f"  purpose          : {task.purpose}")
        for subtask in task.subtasks:
            lines.append(f"  Subtask {subtask.number} — {subtask.name}")
            for figure in task.figures:
                if figure.subtask != subtask.number:
                    continue
                needs = " (needs truth)" if figure.requires_truth else ""
                lines.append(
                    f"      figure {figure.slug}: {figure.title}"
                    f" [frame {figure.frame}; data {'+'.join(figure.sources)}]{needs}"
                )
        lines.append("")
    lines.append("Cross-method comparison (written by an ALL / multi-method run)")
    lines.append(f"  output directory : {COMPARISON_SLUG}/")
    for figure in COMPARISON_FIGURES:
        needs = " (needs truth)" if figure.requires_truth else ""
        lines.append(
            f"      figure {figure.slug}: {figure.title}"
            f" [frame {figure.frame}; data {'+'.join(figure.sources)}]{needs}"
        )
    lines.append("")
    lines.append("Coordinate frame tags used in figure filenames")
    for tag, description in FRAMES.items():
        lines.append(f"  {tag:<10} {description}")
    lines.append("")
    lines.append(
        "Figure filename pattern:\n"
        "  <run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png"
    )
    return "\n".join(lines)


def catalog_as_dict() -> dict[str, object]:
    """Machine-readable catalog, embedded into every run manifest."""
    return {
        "frames": dict(FRAMES),
        "tasks": [
            {
                "number": task.number,
                "slug": task.slug,
                "name": task.name,
                "purpose": task.purpose,
                "directory": task.directory_name,
                "subtasks": [
                    {"number": item.number, "name": item.name} for item in task.subtasks
                ],
                "figures": [
                    {
                        "subtask": figure.subtask,
                        "slug": figure.slug,
                        "title": figure.title,
                        "frame": figure.frame,
                        "sources": list(figure.sources),
                        "requires_truth": figure.requires_truth,
                    }
                    for figure in task.figures
                ],
            }
            for task in TASKS
        ],
        "comparison_figures": [
            {
                "subtask": figure.subtask,
                "slug": figure.slug,
                "title": figure.title,
                "frame": figure.frame,
                "sources": list(figure.sources),
                "requires_truth": figure.requires_truth,
            }
            for figure in COMPARISON_FIGURES
        ],
    }
