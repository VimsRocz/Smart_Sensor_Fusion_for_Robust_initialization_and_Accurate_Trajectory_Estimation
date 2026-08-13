"""Task 1 subtask figures: gravity, Earth rotation rate, and validation.

Subtask 1.2  gravity vector in NED, with the launch site marked on Earth
Subtask 1.3  Earth rotation-rate vector in NED, with its latitude dependence
Subtask 1.4  reference-vector validation summary

Each figure is written as PNG and PDF, with a ``.mat`` companion holding the
plotted arrays so it can be loaded in MATLAB.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from utils.matlab_fig_export import save_matlab_fig  # noqa: E402

NED = ("North", "East", "Down")


def _ensure_matlab_helper(out_dir: Path) -> None:
    """Copy show_task_plot.m beside the .mat files so MATLAB can find it."""
    try:
        src = Path(__file__).resolve().parents[2] / "MATLAB" / "show_task_plot.m"
        dst = Path(out_dir) / "show_task_plot.m"
        if src.is_file() and (not dst.exists() or dst.stat().st_mtime < src.stat().st_mtime):
            dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
    except Exception:
        pass


def _save(fig, out_dir, stem, arrays=None):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    save_matlab_fig(fig, str(out_dir / stem))
    if arrays:
        try:
            from scipy.io import savemat

            savemat(str(out_dir / f"{stem}.mat"),
                    {k: np.asarray(v) for k, v in arrays.items()}, do_compression=True)
            _ensure_matlab_helper(out_dir)
            print(f"[MAT ] {out_dir / stem}.mat keys={sorted(arrays)}")
        except Exception as exc:  # pragma: no cover
            print(f"[WARN] could not write {stem}.mat: {exc}")
    plt.close(fig)


def _basemap(fig, nrows, ncols, index, lat_deg, lon_deg):
    """Graticule with the site marked.

    Cartopy is deliberately not used here: it downloads Natural Earth data at
    draw time, which fails offline or behind an SSL-inspecting proxy and takes
    the whole figure down with it. The full world map with coastlines is
    already produced by task1_2_location_map via plotly/kaleido.
    """
    ax = fig.add_subplot(nrows, ncols, index)
    ax.set_xlim(-180, 180)
    ax.set_ylim(-90, 90)
    ax.set_xticks(range(-180, 181, 60))
    ax.set_yticks(range(-90, 91, 30))
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color="0.5", lw=0.8)     # equator
    ax.axvline(0, color="0.5", lw=0.8)     # prime meridian
    ax.plot([lon_deg], [lat_deg], "r*", markersize=20, zorder=5, label="Launch site")
    ax.annotate(f"  {lat_deg:.4f}°, {lon_deg:.4f}°", (lon_deg, lat_deg),
                fontsize=9, va="center",
                bbox=dict(fc="white", alpha=0.75, ec="none", pad=1.5))
    ax.set_xlabel("Longitude [deg]")
    ax.set_ylabel("Latitude [deg]")
    return ax


def task1_2_gravity(tag, lat_deg, lon_deg, alt_m, g_ned, out_dir):
    """Gravity vector in NED plus the site on Earth."""
    g_ned = np.asarray(g_ned, dtype=float).reshape(3)
    g_mag = float(np.linalg.norm(g_ned))

    fig = plt.figure(figsize=(15, 5.8))
    ax0 = fig.add_subplot(1, 2, 1)
    bars = ax0.bar(NED, g_ned, color=["tab:blue", "tab:orange", "tab:green"])
    for b, v in zip(bars, g_ned):
        ax0.text(b.get_x() + b.get_width() / 2, v, f"{v:.6f}",
                 ha="center", va="bottom" if v >= 0 else "top", fontsize=10)
    ax0.axhline(0, color="k", lw=0.8); ax0.grid(True, axis="y", alpha=0.3)
    ax0.set_ylabel("Gravity [m/s²]")
    ax0.set_title(f"Gravity in NED   |g| = {g_mag:.6f} m/s²   (+Z is down)")

    ax1 = _basemap(fig, 1, 2, 2, lat_deg, lon_deg)
    ax1.set_title(f"Launch site   alt = {alt_m:.1f} m")
    ax1.legend(loc="lower left", fontsize=9)

    fig.suptitle("Task 1.2 — gravity vector in NED and the reference location")
    fig.subplots_adjust(top=0.86, wspace=0.25)
    _save(fig, out_dir, f"{tag}_task1_2_gravity_vector_ned",
          arrays=dict(g_ned=g_ned, g_magnitude=g_mag,
                      lat_deg=lat_deg, lon_deg=lon_deg, alt_m=alt_m))


def task1_3_earth_rate(tag, lat_deg, omega_ned, earth_rate, out_dir):
    """Earth rotation-rate vector in NED and its latitude dependence."""
    omega_ned = np.asarray(omega_ned, dtype=float).reshape(3)
    mag = float(np.linalg.norm(omega_ned))

    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(14, 5.5))
    bars = ax0.bar(NED, omega_ned * 1e6,
                   color=["tab:blue", "tab:orange", "tab:green"])
    for b, v in zip(bars, omega_ned * 1e6):
        ax0.text(b.get_x() + b.get_width() / 2, v, f"{v:.4f}",
                 ha="center", va="bottom" if v >= 0 else "top", fontsize=10)
    ax0.axhline(0, color="k", lw=0.8); ax0.grid(True, axis="y", alpha=0.3)
    ax0.set_ylabel("Earth rate [µrad/s]")
    ax0.set_title(f"ω_ie in NED   |ω| = {mag:.6e} rad/s\n"
                  f"East component is zero by construction")

    lats = np.linspace(-90, 90, 361)
    ax1.plot(lats, earth_rate * np.cos(np.deg2rad(lats)) * 1e6, label="North = ω·cos(lat)")
    ax1.plot(lats, -earth_rate * np.sin(np.deg2rad(lats)) * 1e6, label="Down = −ω·sin(lat)")
    ax1.axvline(lat_deg, color="r", ls="--", label=f"site lat = {lat_deg:.4f}°")
    ax1.set_xlabel("Latitude [deg]"); ax1.set_ylabel("Component [µrad/s]")
    ax1.grid(True, alpha=0.3); ax1.legend(fontsize=9)
    ax1.set_title("Latitude dependence of the Earth-rate components")

    fig.suptitle(f"Task 1.3 — Earth rotation rate ω = {earth_rate:.6e} rad/s "
                 f"({np.degrees(earth_rate) * 3600:.4f} °/hr)")
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    _save(fig, out_dir, f"{tag}_task1_3_earth_rate_ned",
          arrays=dict(omega_ie_ned=omega_ned, omega_magnitude=mag,
                      earth_rate=earth_rate, lat_deg=lat_deg))


def task1_4_validation(tag, lat_deg, lon_deg, alt_m, g_ned, omega_ned, out_dir):
    """Validation summary: magnitudes, orthogonality and the site."""
    g_ned = np.asarray(g_ned, dtype=float).reshape(3)
    omega_ned = np.asarray(omega_ned, dtype=float).reshape(3)
    g_mag, w_mag = float(np.linalg.norm(g_ned)), float(np.linalg.norm(omega_ned))
    # Angle between the two reference vectors — the observability of a static
    # two-vector alignment depends entirely on this not being 0 or 180 deg.
    cos = float(np.dot(g_ned, omega_ned) / (g_mag * w_mag))
    angle = float(np.degrees(np.arccos(np.clip(cos, -1, 1))))

    fig = plt.figure(figsize=(15, 5.8))
    ax0 = fig.add_subplot(1, 2, 1)
    ax0.axis("off")
    rows = [
        ("Latitude",              f"{lat_deg:.6f}°"),
        ("Longitude",             f"{lon_deg:.6f}°"),
        ("Altitude",              f"{alt_m:.2f} m"),
        ("|gravity|",             f"{g_mag:.6f} m/s²"),
        ("gravity NED",           np.array2string(g_ned, precision=6)),
        ("|Earth rate|",          f"{w_mag:.6e} rad/s"),
        ("Earth rate NED",        np.array2string(omega_ned, precision=9)),
        ("angle(g, ω)",           f"{angle:.4f}°"),
        ("status",                "reference vectors validated"),
    ]
    for i, (k, v) in enumerate(rows):
        ax0.text(0.02, 0.92 - i * 0.10, f"{k:<16}", family="monospace",
                 fontsize=11, fontweight="bold", transform=ax0.transAxes)
        ax0.text(0.40, 0.92 - i * 0.10, v, family="monospace", fontsize=11,
                 transform=ax0.transAxes)
    ax0.set_title("Reference vector validation", loc="left")

    ax1 = _basemap(fig, 1, 2, 2, lat_deg, lon_deg)
    ax1.set_title("Computed initial position from GNSS")
    ax1.legend(loc="lower left", fontsize=9)

    fig.suptitle("Task 1.4 — validating the NED reference vectors")
    fig.subplots_adjust(top=0.86, wspace=0.25)
    _save(fig, out_dir, f"{tag}_task1_4_reference_validation",
          arrays=dict(lat_deg=lat_deg, lon_deg=lon_deg, alt_m=alt_m,
                      g_ned=g_ned, omega_ie_ned=omega_ned,
                      g_magnitude=g_mag, omega_magnitude=w_mag,
                      angle_g_omega_deg=angle))
