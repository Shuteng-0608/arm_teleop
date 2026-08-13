"""Trajectory primitives and deterministic fixed-rate interpolation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Mapping, Optional, Sequence

import numpy as np


_EPS = 1e-12


@dataclass(frozen=True)
class Waypoint:
    """A robot-base-frame end-effector delta from the initial pose."""

    position: np.ndarray
    rotation: np.ndarray
    name: str = ""
    stop: bool = False
    hold_s: float = 0.0
    circle_center: Optional[np.ndarray] = None
    circle_normal: Optional[np.ndarray] = None
    circle_angle_rad: float = 0.0


@dataclass(frozen=True)
class SampledTrajectory:
    time_s: np.ndarray
    position: np.ndarray
    rotation: np.ndarray


def rotvec_degrees_to_matrix(rotvec_degrees: Sequence[float]) -> np.ndarray:
    """Convert an axis-angle rotation vector in degrees to a 3x3 matrix."""
    rotvec = np.deg2rad(np.asarray(rotvec_degrees, dtype=np.float64))
    if rotvec.shape != (3,):
        raise ValueError("rotation_delta_rotvec_deg must contain three values")
    theta = float(np.linalg.norm(rotvec))
    if theta < _EPS:
        return np.eye(3, dtype=np.float64)
    axis = rotvec / theta
    skew = np.array(
        [[0.0, -axis[2], axis[1]],
         [axis[2], 0.0, -axis[0]],
         [-axis[1], axis[0], 0.0]],
        dtype=np.float64,
    )
    return np.eye(3) + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)


def matrix_to_rotvec(rotation: np.ndarray) -> np.ndarray:
    """Return the principal SO(3) logarithm as an axis-angle vector in radians."""
    matrix = np.asarray(rotation, dtype=np.float64)
    if matrix.shape != (3, 3):
        raise ValueError("rotation matrix must have shape (3, 3)")
    theta = float(np.arccos(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0)))
    antisymmetric = np.array(
        [matrix[2, 1] - matrix[1, 2],
         matrix[0, 2] - matrix[2, 0],
         matrix[1, 0] - matrix[0, 1]],
        dtype=np.float64,
    )
    if theta < 1e-8:
        return 0.5 * antisymmetric
    if np.pi - theta < 1e-6:
        symmetric = 0.5 * (matrix + np.eye(3))
        axis = np.sqrt(np.maximum(np.diag(symmetric), 0.0))
        index = int(np.argmax(axis))
        if axis[index] < _EPS:
            axis = np.array([1.0, 0.0, 0.0])
        else:
            for other in range(3):
                if other != index:
                    axis[other] = symmetric[index, other] / axis[index]
            axis /= np.linalg.norm(axis)
        if np.dot(axis, antisymmetric) < 0.0:
            axis = -axis
        return theta * axis
    return theta * antisymmetric / (2.0 * np.sin(theta))


def slerp_matrix(start: np.ndarray, end: np.ndarray, fraction: float) -> np.ndarray:
    """Shortest-path SLERP between two rotation matrices."""
    fraction = float(np.clip(fraction, 0.0, 1.0))
    relative_rotvec = matrix_to_rotvec(np.asarray(start).T @ np.asarray(end))
    increment = rotvec_degrees_to_matrix(np.rad2deg(relative_rotvec * fraction))
    return np.asarray(start, dtype=np.float64) @ increment


def parse_waypoints(values: Sequence[Mapping[str, Any]]) -> List[Waypoint]:
    if not values:
        raise ValueError("a waypoint hand requires at least one waypoint")
    waypoints = []
    previous_rotation = np.eye(3)
    previous_position = None
    for index, value in enumerate(values):
        try:
            circle = value.get("circle")
            if circle is not None:
                if previous_position is None:
                    raise ValueError("circle requires a preceding ordinary waypoint")
                center = np.asarray(circle["center_delta_m"], dtype=np.float64)
                normal = np.asarray(circle["normal"], dtype=np.float64)
                if center.shape != (3,) or normal.shape != (3,):
                    raise ValueError("circle center_delta_m and normal must contain three values")
                normal_norm = np.linalg.norm(normal)
                if normal_norm < _EPS:
                    raise ValueError("circle normal must be nonzero")
                normal = normal / normal_norm
                radial = previous_position - center
                if np.linalg.norm(radial) < _EPS:
                    raise ValueError("circle start must differ from center")
                if abs(np.dot(radial, normal)) > 1e-8:
                    raise ValueError("circle start and center must lie in the plane normal to normal")
                turns = float(circle.get("turns", 1.0))
                if turns <= 0.0:
                    raise ValueError("circle turns must be positive")
                direction = circle.get("direction", "counterclockwise")
                if direction not in ("counterclockwise", "clockwise"):
                    raise ValueError("circle direction must be counterclockwise or clockwise")
                circle_angle = (1.0 if direction == "counterclockwise" else -1.0) * 2.0 * np.pi * turns
                position = center + _rotate_vector(radial, normal, circle_angle)
            else:
                center = None
                normal = None
                circle_angle = 0.0
                position = np.asarray(value["position_delta_m"], dtype=np.float64)
            if "rotation_delta_rotvec_deg" in value:
                previous_rotation = rotvec_degrees_to_matrix(
                    value["rotation_delta_rotvec_deg"]
                )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"invalid waypoint {index}: {exc}") from exc
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError(f"waypoint {index} position_delta_m must be three finite values")
        hold_s = float(value.get("hold_s", 0.0))
        if hold_s < 0.0:
            raise ValueError(f"waypoint {index} hold_s must be nonnegative")
        waypoints.append(
            Waypoint(
                position=position,
                rotation=previous_rotation.copy(),
                name=str(value.get("name", "")),
                stop=bool(value.get("stop", False)),
                hold_s=hold_s,
                circle_center=center,
                circle_normal=normal,
                circle_angle_rad=circle_angle,
            )
        )
        previous_position = position
    return waypoints


def sample_trajectory(
    waypoints: Sequence[Waypoint], rate_hz: float, interpolation: Mapping[str, Any]
) -> SampledTrajectory:
    if rate_hz <= 0.0:
        raise ValueError("sample_rate_hz must be positive")
    kind = str(interpolation.get("type", "stop_and_go"))
    if kind == "stop_and_go":
        return _sample_stop_and_go(waypoints, rate_hz, interpolation)
    if kind == "printer_lookahead":
        return _sample_printer_lookahead(waypoints, rate_hz, interpolation)
    if kind == "uniform_linear":
        return _sample_uniform_linear(waypoints, rate_hz, interpolation)
    raise ValueError(f"unsupported interpolation.type: {kind}")


def _sample_stop_and_go(
    waypoints: Sequence[Waypoint], rate_hz: float, config: Mapping[str, Any]
) -> SampledTrajectory:
    durations = [
        _quintic_duration(_path_length(a, b), _rotation_distance(a, b), config)
        for a, b in zip(waypoints[:-1], waypoints[1:])
    ]
    return _sample_phases(waypoints, rate_hz, durations, "quintic")


def _sample_uniform_linear(
    waypoints: Sequence[Waypoint], rate_hz: float, config: Mapping[str, Any]
) -> SampledTrajectory:
    durations = [
        max(
            _path_length(a, b) / _linear_speed(config),
            _rotation_distance(a, b) / _angular_speed(config),
            1.0 / rate_hz,
        )
        for a, b in zip(waypoints[:-1], waypoints[1:])
    ]
    return _sample_phases(waypoints, rate_hz, durations, "linear")


def _sample_printer_lookahead(
    waypoints: Sequence[Waypoint], rate_hz: float, config: Mapping[str, Any]
) -> SampledTrajectory:
    if len(waypoints) == 1:
        return _sample_phases(waypoints, rate_hz, [], "linear")
    lengths = np.asarray([_path_length(a, b) for a, b in zip(waypoints[:-1], waypoints[1:])])
    rotation_lengths = np.asarray(
        [_rotation_distance(a, b) for a, b in zip(waypoints[:-1], waypoints[1:])]
    )
    speed_cap = _linear_speed(config)
    acceleration = float(config.get("max_linear_accel_m_s2", 0.25))
    square_speed = float(config.get("square_corner_speed_m_s", 0.0))
    angular_acceleration = np.deg2rad(
        float(config.get("max_angular_accel_deg_s2", 180.0))
    )
    if acceleration <= 0.0 or angular_acceleration <= 0.0 or square_speed < 0.0:
        raise ValueError("printer accelerations must be positive and corner speed nonnegative")

    vertex_speeds = np.zeros(len(waypoints), dtype=np.float64)
    for index in range(1, len(waypoints) - 1):
        if waypoints[index].stop or waypoints[index].hold_s > 0.0:
            continue
        incoming = _segment_tangent(waypoints[index - 1], waypoints[index], at_end=True)
        outgoing = _segment_tangent(waypoints[index], waypoints[index + 1], at_end=False)
        if np.linalg.norm(incoming) < _EPS or np.linalg.norm(outgoing) < _EPS:
            continue
        cosine = float(
            np.clip(
                np.dot(incoming, outgoing)
                / (np.linalg.norm(incoming) * np.linalg.norm(outgoing)),
                -1.0,
                1.0,
            )
        )
        if cosine > 1.0 - 1e-10:
            corner_limit = speed_cap
        elif cosine < -1.0 + 1e-10:
            corner_limit = 0.0
        else:
            # Exactly square_speed at 90 degrees, smoothly approaching zero
            # at reversal and the segment cap as the path becomes straight.
            corner_limit = square_speed * np.sqrt(max(0.0, 1.0 + cosine))
        vertex_speeds[index] = min(speed_cap, corner_limit)

    # Forward/reverse scans enforce how fast each junction is reachable under
    # the acceleration limit. Stops and held waypoints are already zero caps.
    for index, length in enumerate(lengths):
        reachable = np.sqrt(vertex_speeds[index] ** 2 + 2.0 * acceleration * length)
        vertex_speeds[index + 1] = min(vertex_speeds[index + 1], reachable)
    for index in range(len(lengths) - 1, -1, -1):
        reachable = np.sqrt(vertex_speeds[index + 1] ** 2 + 2.0 * acceleration * lengths[index])
        vertex_speeds[index] = min(vertex_speeds[index], reachable)

    profiles = [
        _motion_profile(
            lengths[index], vertex_speeds[index], vertex_speeds[index + 1],
            speed_cap, acceleration,
        )
        for index in range(len(lengths))
    ]
    angular_profiles = [
        _motion_profile(value, 0.0, 0.0, _angular_speed(config), angular_acceleration)
        for value in rotation_lengths
    ]
    durations = [
        max(profile["duration"], angular_profile["duration"], 1.0 / rate_hz)
        for profile, angular_profile in zip(profiles, angular_profiles)
    ]
    return _sample_phases(
        waypoints,
        rate_hz,
        durations,
        "printer",
        profiles=profiles,
        rotation_profiles=angular_profiles,
    )


def _sample_phases(
    waypoints: Sequence[Waypoint],
    rate_hz: float,
    durations: Sequence[float],
    mode: str,
    profiles: Sequence[Mapping[str, float]] = (),
    rotation_profiles: Sequence[Mapping[str, float]] = (),
) -> SampledTrajectory:
    positions = [waypoints[0].position.copy()]
    rotations = [waypoints[0].rotation.copy()]

    def append_hold(waypoint: Waypoint) -> None:
        for _ in range(int(round(waypoint.hold_s * rate_hz))):
            positions.append(waypoint.position.copy())
            rotations.append(waypoint.rotation.copy())

    append_hold(waypoints[0])
    for index, duration in enumerate(durations):
        steps = max(1, int(np.ceil(duration * rate_hz)))
        start = waypoints[index]
        end = waypoints[index + 1]
        for step in range(1, steps + 1):
            normalized_time = step / float(steps)
            if mode == "quintic":
                position_fraction = _quintic_smoothstep(normalized_time)
            elif mode == "printer" and profiles[index]["duration"] > _EPS:
                # Stretch the profile over the synchronized segment duration.
                # This keeps position and orientation arriving together when
                # the angular limits are the slower constraint.
                profile_time = normalized_time * profiles[index]["duration"]
                length = max(_path_length(start, end), _EPS)
                position_fraction = float(_profile_distance(profile_time, profiles[index]) / length)
            else:
                position_fraction = normalized_time
            if (
                mode == "printer"
                and rotation_profiles[index]["duration"] > _EPS
                and _rotation_distance(start, end) > _EPS
            ):
                rotation_time = normalized_time * rotation_profiles[index]["duration"]
                rotation_fraction = float(
                    _profile_distance(rotation_time, rotation_profiles[index])
                    / _rotation_distance(start, end)
                )
            else:
                rotation_fraction = normalized_time
            positions.append(_segment_position(start, end, position_fraction))
            rotations.append(slerp_matrix(start.rotation, end.rotation, rotation_fraction))
        append_hold(end)
    time_s = np.arange(len(positions), dtype=np.float64) / rate_hz
    return SampledTrajectory(time_s, np.stack(positions), np.stack(rotations))


def _path_length(start: Waypoint, end: Waypoint) -> float:
    if end.circle_center is not None:
        radius = np.linalg.norm(start.position - end.circle_center)
        return float(radius * abs(end.circle_angle_rad))
    return float(np.linalg.norm(end.position - start.position))


def _segment_position(start: Waypoint, end: Waypoint, fraction: float) -> np.ndarray:
    if end.circle_center is None:
        return start.position + fraction * (end.position - start.position)
    radial = start.position - end.circle_center
    return end.circle_center + _rotate_vector(
        radial, end.circle_normal, end.circle_angle_rad * fraction
    )


def _segment_tangent(start: Waypoint, end: Waypoint, at_end: bool) -> np.ndarray:
    if end.circle_center is None:
        return end.position - start.position
    radial = (end.position if at_end else start.position) - end.circle_center
    direction = 1.0 if end.circle_angle_rad > 0.0 else -1.0
    return direction * np.cross(end.circle_normal, radial)


def _rotate_vector(vector: np.ndarray, axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation used by analytic circle primitives."""
    return (
        vector * np.cos(angle)
        + np.cross(axis, vector) * np.sin(angle)
        + axis * np.dot(axis, vector) * (1.0 - np.cos(angle))
    )


def _rotation_distance(start: Waypoint, end: Waypoint) -> float:
    return float(np.linalg.norm(matrix_to_rotvec(start.rotation.T @ end.rotation)))


def _linear_speed(config: Mapping[str, Any]) -> float:
    speed = float(config.get("target_linear_speed_m_s", 0.1))
    if speed <= 0.0:
        raise ValueError("target_linear_speed_m_s must be positive")
    return speed


def _angular_speed(config: Mapping[str, Any]) -> float:
    speed = np.deg2rad(float(config.get("max_angular_speed_deg_s", 90.0)))
    if speed <= 0.0:
        raise ValueError("max_angular_speed_deg_s must be positive")
    return float(speed)


def _quintic_duration(
    linear_distance: float, angular_distance: float, config: Mapping[str, Any]
) -> float:
    linear_acceleration = float(config.get("max_linear_accel_m_s2", 0.25))
    angular_acceleration = np.deg2rad(
        float(config.get("max_angular_accel_deg_s2", 180.0))
    )
    if linear_acceleration <= 0.0 or angular_acceleration <= 0.0:
        raise ValueError("maximum accelerations must be positive")
    # Quintic smoothstep max |ds/du|=1.875 and max |d2s/du2|=10/sqrt(3).
    accel_coefficient = 10.0 / np.sqrt(3.0)
    return max(
        1.875 * linear_distance / _linear_speed(config),
        np.sqrt(accel_coefficient * linear_distance / linear_acceleration),
        1.875 * angular_distance / _angular_speed(config),
        np.sqrt(accel_coefficient * angular_distance / angular_acceleration),
        0.0,
    )


def _quintic_smoothstep(value: float) -> float:
    return value**3 * (10.0 + value * (-15.0 + 6.0 * value))


def _trapezoid_duration(distance: float, speed: float, acceleration: float) -> float:
    if distance < _EPS:
        return 0.0
    ramp_distance = speed * speed / acceleration
    if distance <= ramp_distance:
        return 2.0 * np.sqrt(distance / acceleration)
    return 2.0 * speed / acceleration + (distance - ramp_distance) / speed


def _motion_profile(
    distance: float,
    entry_speed: float,
    exit_speed: float,
    speed_cap: float,
    acceleration: float,
) -> Mapping[str, float]:
    if distance < _EPS:
        return {
            "duration": 0.0, "entry": 0.0, "peak": 0.0,
            "acceleration": acceleration, "t_accel": 0.0,
            "t_cruise": 0.0, "d_accel": 0.0, "d_cruise": 0.0,
        }
    triangular_peak = np.sqrt(
        acceleration * distance + 0.5 * (entry_speed**2 + exit_speed**2)
    )
    peak = min(speed_cap, triangular_peak)
    d_accel = max(0.0, (peak**2 - entry_speed**2) / (2.0 * acceleration))
    d_decel = max(0.0, (peak**2 - exit_speed**2) / (2.0 * acceleration))
    d_cruise = max(0.0, distance - d_accel - d_decel)
    t_accel = max(0.0, (peak - entry_speed) / acceleration)
    t_decel = max(0.0, (peak - exit_speed) / acceleration)
    t_cruise = d_cruise / peak if peak > _EPS else 0.0
    return {
        "duration": t_accel + t_cruise + t_decel,
        "entry": entry_speed,
        "peak": peak,
        "acceleration": acceleration,
        "t_accel": t_accel,
        "t_cruise": t_cruise,
        "d_accel": d_accel,
        "d_cruise": d_cruise,
    }


def _profile_distance(time_s: float, profile: Mapping[str, float]) -> float:
    if time_s <= profile["t_accel"]:
        return profile["entry"] * time_s + 0.5 * profile["acceleration"] * time_s**2
    cruise_end = profile["t_accel"] + profile["t_cruise"]
    if time_s <= cruise_end:
        return profile["d_accel"] + profile["peak"] * (time_s - profile["t_accel"])
    decel_time = time_s - cruise_end
    return (
        profile["d_accel"] + profile["d_cruise"]
        + profile["peak"] * decel_time
        - 0.5 * profile["acceleration"] * decel_time**2
    )
