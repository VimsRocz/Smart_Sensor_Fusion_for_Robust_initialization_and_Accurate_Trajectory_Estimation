import numpy as np

from fusion_pipeline.attitude import METHODS, solve_attitude


def _rotation_z(angle_deg: float) -> np.ndarray:
    angle = np.deg2rad(angle_deg)
    return np.array(
        [[np.cos(angle), -np.sin(angle), 0.0],
         [np.sin(angle), np.cos(angle), 0.0],
         [0.0, 0.0, 1.0]]
    )


def test_all_methods_recover_body_to_reference_rotation():
    expected = _rotation_z(32.0)
    reference = np.array([[0.0, 0.0, -1.0], [0.7, 0.0, -0.3]])
    reference /= np.linalg.norm(reference, axis=1, keepdims=True)
    body = (expected.T @ reference.T).T

    for method in METHODS:
        matrix, quaternion, errors = solve_attitude(method, body, reference)
        assert np.allclose(matrix, expected, atol=1e-9), method
        assert np.isclose(np.linalg.det(matrix), 1.0, atol=1e-12)
        assert np.isclose(np.linalg.norm(quaternion), 1.0, atol=1e-12)
        assert errors["gravity_error_deg"] < 1e-5
        assert errors["earth_rate_error_deg"] < 1e-5
