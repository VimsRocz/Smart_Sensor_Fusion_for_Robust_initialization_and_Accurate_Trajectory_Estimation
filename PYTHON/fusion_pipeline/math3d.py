"""Small dependency-free frame and quaternion utilities."""

from __future__ import annotations

import numpy as np

WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3


def ecef_to_geodetic(ecef_m: np.ndarray) -> tuple[float, float, float]:
    x, y, z = np.asarray(ecef_m, dtype=float).reshape(3)
    lon = np.arctan2(y, x)
    p = np.hypot(x, y)
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        s = np.sin(lat)
        n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * s * s)
        alt = p / max(np.cos(lat), 1e-15) - n
        next_lat = np.arctan2(z, p * (1.0 - WGS84_E2 * n / (n + alt)))
        if abs(next_lat - lat) < 1e-13:
            lat = next_lat
            break
        lat = next_lat
    s = np.sin(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * s * s)
    alt = p / max(np.cos(lat), 1e-15) - n
    return float(lat), float(lon), float(alt)


def ecef_to_ned_matrix(lat_rad: float, lon_rad: float) -> np.ndarray:
    sl, cl = np.sin(lat_rad), np.cos(lat_rad)
    so, co = np.sin(lon_rad), np.cos(lon_rad)
    return np.array(
        [[-sl * co, -sl * so, cl], [-so, co, 0.0], [-cl * co, -cl * so, -sl]],
        dtype=float,
    )


def normalize(vector: np.ndarray, label: str = "vector") -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError(f"Cannot normalize zero {label}")
    return vector / norm


def project_rotation(matrix: np.ndarray) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.asarray(matrix, dtype=float))
    d = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    return u @ d @ vt


def matrix_to_quaternion_wxyz(matrix: np.ndarray) -> np.ndarray:
    """Convert a proper rotation matrix to scalar-first quaternion."""
    m = project_rotation(matrix)
    trace = np.trace(m)
    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array([0.25 * s, (m[2, 1] - m[1, 2]) / s,
                      (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s])
    else:
        idx = int(np.argmax(np.diag(m)))
        if idx == 0:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            q = np.array([(m[2, 1] - m[1, 2]) / s, 0.25 * s,
                          (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s])
        elif idx == 1:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            q = np.array([(m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s,
                          0.25 * s, (m[1, 2] + m[2, 1]) / s])
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            q = np.array([(m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s,
                          (m[1, 2] + m[2, 1]) / s, 0.25 * s])
    q /= np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    return q


def quaternion_to_matrix(q_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(q_wxyz, dtype=float).reshape(4)
    q /= np.linalg.norm(q)
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    w0, x0, y0, z0 = np.asarray(left, dtype=float).reshape(4)
    w1, x1, y1, z1 = np.asarray(right, dtype=float).reshape(4)
    return np.array(
        [w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
         w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
         w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
         w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1]
    )


def quaternion_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    rotvec = np.asarray(rotvec, dtype=float).reshape(3)
    angle = np.linalg.norm(rotvec)
    if angle < 1e-12:
        return normalize(np.r_[1.0, 0.5 * rotvec], "quaternion")
    return np.r_[np.cos(angle / 2.0), np.sin(angle / 2.0) * rotvec / angle]


def quaternion_series_to_euler_zyx_deg(quaternions: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternions, dtype=float)
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    w, x, y, z = q.T
    yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0)))
    roll = np.degrees(np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    return np.column_stack([yaw, pitch, roll])


def align_quaternion_sign(reference: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    reference = np.asarray(reference, dtype=float)
    estimate = np.asarray(estimate, dtype=float).copy()
    dots = np.sum(reference * estimate, axis=1)
    estimate[dots < 0] *= -1.0
    return estimate
