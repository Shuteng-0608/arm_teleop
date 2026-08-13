from pathlib import Path

import numpy as np
import yaml

from trajectory_csv.compiler import (
    ARM_AXIS_TRANSFORM,
    HAND_CALIBRATION,
    POSITION_SCALE,
    compile_document,
    compile_yaml,
)
from trajectory_csv.trajectory import parse_waypoints, rotvec_degrees_to_matrix, sample_trajectory


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
RATE_HZ = 100.0


def _config(kind):
    return {
        "type": kind,
        "target_linear_speed_m_s": 0.04,
        "max_linear_accel_m_s2": 0.10,
        "square_corner_speed_m_s": 0.02,
        "max_angular_speed_deg_s": 30.0,
        "max_angular_accel_deg_s2": 90.0,
    }


def _speed_around(sampled, point):
    index = int(
        np.flatnonzero(np.linalg.norm(sampled.position - point, axis=1) < 1e-10)[0]
    )
    speed = np.linalg.norm(np.diff(sampled.position, axis=0), axis=1) * RATE_HZ
    return speed[index - 1], speed[index]


def test_square_corner_speed_semantics_for_all_modes():
    waypoints = parse_waypoints(
        [
            {"position_delta_m": [0.0, 0.0, 0.0]},
            {"position_delta_m": [0.0, 0.04, 0.0]},
            {"position_delta_m": [0.0, 0.04, 0.04]},
            {"position_delta_m": [0.0, 0.0, 0.04]},
        ]
    )
    corner = np.array([0.0, 0.04, 0.0])

    stop = _speed_around(
        sample_trajectory(waypoints, RATE_HZ, _config("stop_and_go")), corner
    )
    lookahead = _speed_around(
        sample_trajectory(waypoints, RATE_HZ, _config("printer_lookahead")), corner
    )
    uniform = _speed_around(
        sample_trajectory(waypoints, RATE_HZ, _config("uniform_linear")), corner
    )

    assert max(stop) < 0.001
    assert min(lookahead) > 0.01
    np.testing.assert_allclose(uniform, [0.04, 0.04], atol=5e-4)


def test_uniform_circle_is_closed_constant_radius_and_arc_length_sampled():
    center = np.array([0.0, 0.0, 0.11])
    waypoints = parse_waypoints(
        [
            {"position_delta_m": [0.0, -0.03, 0.11]},
            {
                "circle": {
                    "center_delta_m": center.tolist(),
                    "normal": [1.0, 0.0, 0.0],
                    "turns": 1.0,
                    "direction": "counterclockwise",
                }
            },
        ]
    )
    sampled = sample_trajectory(waypoints, RATE_HZ, _config("uniform_linear"))

    radius = np.linalg.norm(sampled.position - center, axis=1)
    steps = np.linalg.norm(np.diff(sampled.position, axis=0), axis=1)
    np.testing.assert_allclose(radius, 0.03, atol=1e-12)
    np.testing.assert_allclose(sampled.position[0], sampled.position[-1], atol=1e-12)
    np.testing.assert_allclose(steps, steps[0], atol=1e-12)


def test_compiled_wrist_matrices_recover_requested_robot_deltas():
    trajectory = PACKAGE_ROOT / "trajectories" / "right_circle_printer_lookahead.yaml"
    compiled = compile_yaml(str(trajectory))

    for side in ("left", "right"):
        wrist = compiled[f"{side}_matrix"]
        hand_delta = wrist[:, :3, 3] - wrist[0, :3, 3]
        position = POSITION_SCALE * np.stack(
            (hand_delta[:, 1], hand_delta[:, 2], hand_delta[:, 0]), axis=-1
        )
        relative_rotation = wrist[:, :3, :3] @ HAND_CALIBRATION[side].T
        rotation = np.stack(
            [
                ARM_AXIS_TRANSFORM @ value @ ARM_AXIS_TRANSFORM.T
                for value in relative_rotation
            ]
        )
        assert np.allclose(position[0], 0.0)
        assert np.allclose(rotation[0], np.eye(3))
        if side == "left":
            np.testing.assert_allclose(position, 0.0, atol=1e-12)
        else:
            assert np.max(position[:, 2]) >= 0.109
        expected_identity = np.broadcast_to(np.eye(3), rotation.shape)
        np.testing.assert_allclose(
            rotation @ np.swapaxes(rotation, 1, 2), expected_identity, atol=1e-12
        )


def test_nonzero_rotation_delta_survives_csv_wrist_mapping():
    document = {
        "api_version": "trajectory_csv/v1",
        "trajectory": {
            "sample_rate_hz": 100,
            "coordinate_frame": "robot_base_delta",
            "interpolation": _config("uniform_linear"),
        },
        "left_hand": {"mode": "hold_initial"},
        "right_hand": {
            "mode": "waypoints",
            "orientation": {"mode": "fixed_initial"},
            "waypoints": [
                {"position_delta_m": [0.0, 0.0, 0.0]},
                {
                    "position_delta_m": [0.01, -0.02, 0.03],
                    "rotation_delta_rotvec_deg": [10.0, -20.0, 30.0],
                },
            ],
        },
    }
    compiled = compile_document(document)
    wrist = compiled["right_matrix"]
    relative = wrist[-1, :3, :3] @ HAND_CALIBRATION["right"].T
    recovered = ARM_AXIS_TRANSFORM @ relative @ ARM_AXIS_TRANSFORM.T

    np.testing.assert_allclose(
        recovered, rotvec_degrees_to_matrix([10.0, -20.0, 30.0]), atol=1e-12
    )
