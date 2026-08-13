from pathlib import Path

import numpy as np
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY_DIR = PACKAGE_ROOT / "trajectories"
EXPECTED_MODES = {"stop_and_go", "printer_lookahead", "uniform_linear"}


def _load_examples():
    examples = {}
    for path in sorted(TRAJECTORY_DIR.glob("right_square_*.yaml")):
        with path.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        mode = document["trajectory"]["interpolation"]["type"]
        examples[mode] = document
    return examples


def test_examples_cover_all_interpolation_modes():
    assert set(_load_examples()) == EXPECTED_MODES


def test_examples_share_the_same_right_hand_geometry():
    examples = _load_examples()
    paths = []
    for mode in sorted(EXPECTED_MODES):
        document = examples[mode]
        assert document["api_version"] == "trajectory_csv/v1"
        assert document["trajectory"]["coordinate_frame"] == "robot_base_delta"
        assert document["left_hand"]["mode"] == "hold_initial"
        assert document["right_hand"]["mode"] == "waypoints"
        assert document["right_hand"]["orientation"]["mode"] == "fixed_initial"
        paths.append(
            [waypoint["position_delta_m"] for waypoint in document["right_hand"]["waypoints"]]
        )
    assert paths[1:] == paths[:-1]


def test_square_is_closed_and_returns_home():
    for document in _load_examples().values():
        waypoints = document["right_hand"]["waypoints"]
        by_name = {waypoint["name"]: waypoint for waypoint in waypoints}
        assert by_name["square_closed"]["position_delta_m"] == by_name["square_bottom_left"]["position_delta_m"]
        assert waypoints[0]["position_delta_m"] == [0.0, 0.0, 0.0]
        assert waypoints[-1]["position_delta_m"] == [0.0, 0.0, 0.0]


def test_circle_example_has_valid_plane_and_radius():
    path = TRAJECTORY_DIR / "right_circle_printer_lookahead.yaml"
    with path.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    waypoints = document["right_hand"]["waypoints"]
    start = np.asarray(waypoints[2]["position_delta_m"], dtype=float)
    circle = waypoints[3]["circle"]
    center = np.asarray(circle["center_delta_m"], dtype=float)
    normal = np.asarray(circle["normal"], dtype=float)
    radial = start - center
    assert np.linalg.norm(radial) > 0.0
    assert np.linalg.norm(normal) > 0.0
    assert np.isclose(np.dot(radial, normal), 0.0)
    assert circle["turns"] > 0.0
