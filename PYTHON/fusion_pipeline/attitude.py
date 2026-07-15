"""TRIAD, Davenport and SVD attitude initialisation methods."""

from __future__ import annotations

import numpy as np

from .math3d import matrix_to_quaternion_wxyz, normalize, project_rotation

METHODS = ("TRIAD", "Davenport", "SVD")


def _pairs(body_vectors: np.ndarray, reference_vectors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    body = np.asarray(body_vectors, dtype=float)
    reference = np.asarray(reference_vectors, dtype=float)
    if body.shape != (2, 3) or reference.shape != (2, 3):
        raise ValueError("body_vectors and reference_vectors must both be 2x3")
    return np.vstack([normalize(v, "body vector") for v in body]), np.vstack(
        [normalize(v, "reference vector") for v in reference]
    )


def triad(body_vectors: np.ndarray, reference_vectors: np.ndarray) -> np.ndarray:
    body, reference = _pairs(body_vectors, reference_vectors)

    def basis(vectors: np.ndarray) -> np.ndarray:
        first = vectors[0]
        cross = np.cross(vectors[0], vectors[1])
        if np.linalg.norm(cross) < 1e-8:
            raise ValueError("TRIAD vectors are nearly collinear")
        second = normalize(cross, "TRIAD cross product")
        third = np.cross(first, second)
        return np.column_stack([first, second, third])

    return project_rotation(basis(reference) @ basis(body).T)


def svd(body_vectors: np.ndarray, reference_vectors: np.ndarray, weights: np.ndarray) -> np.ndarray:
    body, reference = _pairs(body_vectors, reference_vectors)
    b_matrix = sum(
        float(weight) * np.outer(ref, measured)
        for weight, measured, ref in zip(weights, body, reference)
    )
    u, _, vt = np.linalg.svd(b_matrix)
    return u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt


def davenport(body_vectors: np.ndarray, reference_vectors: np.ndarray, weights: np.ndarray) -> np.ndarray:
    body, reference = _pairs(body_vectors, reference_vectors)
    b_matrix = sum(
        float(weight) * np.outer(ref, measured)
        for weight, measured, ref in zip(weights, body, reference)
    )
    sigma = np.trace(b_matrix)
    symmetric = b_matrix + b_matrix.T
    z = np.array(
        [b_matrix[1, 2] - b_matrix[2, 1],
         b_matrix[2, 0] - b_matrix[0, 2],
         b_matrix[0, 1] - b_matrix[1, 0]]
    )
    k = np.block(
        [[np.array([[sigma]]), z.reshape(1, 3)],
         [z.reshape(3, 1), symmetric - sigma * np.eye(3)]]
    )
    _, vectors = np.linalg.eigh(k)
    q_candidate = vectors[:, -1]
    # The K construction can encode either passive convention. Choose the one
    # that actually minimizes the stated body-to-NED vector residual.
    candidates = []
    for q in (q_candidate, np.r_[q_candidate[0], -q_candidate[1:]]):
        from .math3d import quaternion_to_matrix

        matrix = quaternion_to_matrix(q)
        error = sum(
            float(weight) * np.linalg.norm(matrix @ measured - ref) ** 2
            for weight, measured, ref in zip(weights, body, reference)
        )
        candidates.append((error, matrix))
    return project_rotation(min(candidates, key=lambda item: item[0])[1])


def solve_attitude(
    method: str,
    body_vectors: np.ndarray,
    reference_vectors: np.ndarray,
    weights: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, dict[str, float]]:
    canonical = next((name for name in METHODS if name.lower() == method.lower()), None)
    if canonical is None:
        raise ValueError(f"Unknown method {method!r}; choose {', '.join(METHODS)}")
    weights = np.asarray(weights if weights is not None else [0.9999, 0.0001], dtype=float)
    if weights.shape != (2,) or np.any(weights <= 0):
        raise ValueError("attitude weights must contain two positive values")
    weights /= np.sum(weights)
    if canonical == "TRIAD":
        matrix = triad(body_vectors, reference_vectors)
    elif canonical == "Davenport":
        matrix = davenport(body_vectors, reference_vectors, weights)
    else:
        matrix = svd(body_vectors, reference_vectors, weights)
    body, reference = _pairs(body_vectors, reference_vectors)
    errors = {}
    for name, measured, ref in zip(("gravity", "earth_rate"), body, reference):
        dot = np.clip(np.dot(matrix @ measured, ref), -1.0, 1.0)
        errors[f"{name}_error_deg"] = float(np.degrees(np.arccos(dot)))
    return matrix, matrix_to_quaternion_wxyz(matrix), errors
