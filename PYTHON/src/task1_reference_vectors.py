from __future__ import annotations
import json
import uuid
from pathlib import Path
import os
from utils.plot_save import save_plot, task_summary

import numpy as np
import pandas as pd
# Plotly is optional; fall back to matplotlib when unavailable
try:  # pragma: no cover - environment dependent
    import plotly.graph_objects as go
    import plotly.io as pio
    PLOTLY_AVAILABLE = True
except Exception:  # pragma: no cover - graceful degradation
    go = None
    pio = None
    PLOTLY_AVAILABLE = False

from utils import ecef_to_geodetic


def ensure_deg_latlon(lat_in, lon_in):
    lat = float(lat_in)
    lon = float(lon_in)
    if abs(lat) <= np.pi + 1e-9 and abs(lon) <= 2 * np.pi + 1e-9:
        lat = np.degrees(lat)
        lon = np.degrees(lon)
    if lon > 180:
        lon = ((lon + 180) % 360) - 180
    if not (-90 <= lat <= 90 and -180 <= lon <= 180):
        raise ValueError(f"Bad lat/lon after conversion: lat={lat}, lon={lon}")
    return lat, lon


def task1_reference_vectors(gnss_data: pd.DataFrame, output_dir: str | Path, run_id: str) -> Path:
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    if "Height_deg" in gnss_data.columns and "Height_m" not in gnss_data.columns:
        gnss_data = gnss_data.rename(columns={"Height_deg": "Height_m"})

    if {"Latitude_deg", "Longitude_deg"}.issubset(gnss_data.columns):
        valid = (
            gnss_data["Latitude_deg"].notna()
            & gnss_data["Longitude_deg"].notna()
            & (
                (gnss_data["Latitude_deg"].abs() > 1e-9)
                | (gnss_data["Longitude_deg"].abs() > 1e-9)
            )
        )
        if valid.any():
            row = gnss_data.loc[valid].iloc[0]
            lat_raw = float(row["Latitude_deg"])
            lon_raw = float(row["Longitude_deg"])
        else:
            row = gnss_data.loc[
                (gnss_data["X_ECEF_m"] != 0)
                | (gnss_data["Y_ECEF_m"] != 0)
                | (gnss_data["Z_ECEF_m"] != 0)
            ].iloc[0]
            lat_raw, lon_raw, _ = ecef_to_geodetic(
                row["X_ECEF_m"], row["Y_ECEF_m"], row["Z_ECEF_m"]
            )
    else:
        row = gnss_data.loc[
            (gnss_data["X_ECEF_m"] != 0)
            | (gnss_data["Y_ECEF_m"] != 0)
            | (gnss_data["Z_ECEF_m"] != 0)
        ].iloc[0]
        lat_raw, lon_raw, _ = ecef_to_geodetic(
            row["X_ECEF_m"], row["Y_ECEF_m"], row["Z_ECEF_m"]
        )

    lat_deg, lon_deg = ensure_deg_latlon(lat_raw, lon_raw)

    png_path = output_dir / f"{run_id}_task1_2_location_map.png"
    if PLOTLY_AVAILABLE:
        fig = go.Figure()
        fig.add_scattergeo(
            lon=[lon_deg], lat=[lat_deg], mode="markers", marker=dict(size=10, color="red")
        )
        fig.update_layout(
            title="Task 1.2 — Initial GNSS location",
            geo=dict(
                projection_type="equirectangular",
                showcountries=True,
                showcoastlines=True,
                showland=True,
                lataxis=dict(showgrid=True, dtick=15),
                lonaxis=dict(showgrid=True, dtick=30),
            ),
        )
        try:
            chrome = getattr(pio.kaleido.scope, "chromium", "unknown")
            print(f"[Task1] Kaleido chromium={chrome}")
            pio.write_image(fig, png_path, width=1200, height=800, scale=2)
            print(f"[SAVE] {png_path}")
        except Exception as ex:
            print(f"[Task1] Kaleido export failed: {ex}; using Matplotlib fallback")
            import matplotlib.pyplot as plt

            fig2, ax = plt.subplots(figsize=(8, 4))
            ax.set_xlim(-180, 180)
            ax.set_ylim(-90, 90)
            ax.scatter([lon_deg], [lat_deg], color="red")
            ax.set_xlabel("Lon [deg]")
            ax.set_ylabel("Lat [deg]")
            ax.set_title("Task 1.2 — Initial GNSS location (fallback)")
            save_plot(fig2, output_dir, run_id, "task1_2", "location_map")
            plt.close(fig2)
    else:
        # No plotly available — directly use matplotlib fallback
        import matplotlib.pyplot as plt

        fig2, ax = plt.subplots(figsize=(8, 4))
        ax.set_xlim(-180, 180)
        ax.set_ylim(-90, 90)
        ax.scatter([lon_deg], [lat_deg], color="red")
        ax.set_xlabel("Lon [deg]")
        ax.set_ylabel("Lat [deg]")
        ax.set_title("Task 1.2 — Initial GNSS location (fallback)")
        save_plot(fig2, output_dir, run_id, "task1_2", "location_map")
        plt.close(fig2)

    info = {
        "plot_id": uuid.uuid4().hex,
        "latitude": lat_deg,
        "longitude": lon_deg,
    }
    info_path = output_dir / f"{run_id}_task1_location_map_info.json"
    with info_path.open("w", encoding="utf-8") as f:
        json.dump(info, f, indent=2)

    size = os.path.getsize(png_path) if png_path.exists() else 0
    print(f"[Task1] lat={lat_deg:.5f} lon={lon_deg:.5f} dataset={run_id} -> {png_path} bytes={size}")
    task_summary("task1")
    return png_path
