# Dataset overview

This directory contains the logs used by the IMU–GNSS fusion scripts.

* **Run X001** – baseline trajectory with ideal sensors.
* **Run X002** – same path but with additional measurement noise.
* **Run X003** – reuses the GNSS trace from `X002` while injecting a constant bias into the IMU.

GNSS CSV files provide only Earth‑centred Earth‑fixed (ECEF) position and velocity.
The latitude, longitude and altitude columns are kept at zero.  The pipeline
recovers any needed geodetic coordinates from the ECEF values.

Each run also has a shortened `*_small` variant for quick testing. The checked-in
files contain 1,000 IMU samples, nine GNSS data epochs (plus the CSV header), and
99 truth states (plus the truth comment header).
