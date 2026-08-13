"""将人工编写的双手轨迹 YAML 编译为 33 列位姿 CSV。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import numpy as np
import yaml

from .trajectory import SampledTrajectory, Waypoint, parse_waypoints, sample_trajectory, slerp_matrix


POSITION_SCALE = 1.5
ARM_AXIS_TRANSFORM = np.array(
    [[0.0, 1.0, 0.0],
     [0.0, 0.0, 1.0],
     [1.0, 0.0, 0.0]],
    dtype=np.float64,
)
HAND_CALIBRATION = {
    "right": np.array(
        [[0.0, 1.0, 0.0],
         [-1.0, 0.0, 0.0],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    ),
    "left": np.array(
        [[0.0, 1.0, 0.0],
         [1.0, 0.0, 0.0],
         [0.0, 0.0, -1.0]],
        dtype=np.float64,
    ),
}
DEFAULT_ANCHOR_POSITION = {
    # 取自参考文件 vp_raw_record_20260715_204843.csv 的首帧。
    "right": np.array([0.20179371535778046, 0.13955211639404297, 0.8338801860809326]),
    "left": np.array([-0.16510511934757233, 0.2037380039691925, 0.8283536434173584]),
}
COLUMN_ORDERS = {
    "left-right": ("left", "right"),
    "right-left": ("right", "left"),
}


def compile_yaml(source: str) -> Dict[str, np.ndarray]:
    with Path(source).open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    return compile_document(document)


def compile_to_csv(
    source: str,
    destination: str,
    column_order: str = "left-right",
) -> Path:
    return write_csv(destination, compile_yaml(source), column_order=column_order)


def compile_document(document: Mapping[str, Any]) -> Dict[str, np.ndarray]:
    if not isinstance(document, Mapping):
        raise ValueError("trajectory YAML root must be a mapping")
    if document.get("api_version") != "trajectory_csv/v1":
        raise ValueError("api_version must be trajectory_csv/v1")
    trajectory = document.get("trajectory")
    if not isinstance(trajectory, Mapping):
        raise ValueError("trajectory mapping is required")
    if trajectory.get("coordinate_frame") != "robot_base_delta":
        raise ValueError("trajectory.coordinate_frame must be robot_base_delta")

    rate_hz = float(trajectory.get("sample_rate_hz", 100.0))
    if rate_hz <= 0.0:
        raise ValueError("trajectory.sample_rate_hz must be positive")
    interpolation = trajectory.get("interpolation")
    if not isinstance(interpolation, Mapping):
        raise ValueError("trajectory.interpolation mapping is required")

    sampled = {
        side: _compile_hand(document.get(f"{side}_hand"), rate_hz, interpolation)
        for side in ("left", "right")
    }
    frame_count = max(len(sampled["left"].time_s), len(sampled["right"].time_s))
    time_s = np.arange(frame_count, dtype=np.float64) / rate_hz
    resampled = {
        side: _resample_sampled(sampled[side], time_s) for side in ("left", "right")
    }

    anchors = _anchor_positions(document.get("csv"))
    result = {"timestamp": time_s}
    for side in ("left", "right"):
        result[f"{side}_matrix"] = robot_delta_to_wrist_matrix(
            resampled[side].position,
            resampled[side].rotation,
            side,
            anchor_position=anchors[side],
        )
    return result


def robot_delta_to_wrist_matrix(
    position: np.ndarray,
    rotation: np.ndarray,
    side: str,
    anchor_position: np.ndarray,
    position_scale: float = POSITION_SCALE,
) -> np.ndarray:
    """反算离线 VPStreamer 所需的手腕矩阵。"""
    if side not in HAND_CALIBRATION:
        raise ValueError("side must be left or right")
    position = np.asarray(position, dtype=np.float64)
    rotation = np.asarray(rotation, dtype=np.float64)
    anchor_position = np.asarray(anchor_position, dtype=np.float64)
    if position.ndim != 2 or position.shape[1] != 3:
        raise ValueError("position must have shape (N,3)")
    if rotation.shape != (len(position), 3, 3):
        raise ValueError("rotation must have shape (N,3,3)")
    if anchor_position.shape != (3,):
        raise ValueError("anchor_position must have shape (3,)")

    wrist = np.broadcast_to(np.eye(4), (len(position), 4, 4)).copy()
    wrist[:, :3, 3] = anchor_position + np.stack(
        (position[:, 2], position[:, 0], position[:, 1]), axis=-1
    ) / position_scale
    wrist[:, :3, :3] = np.stack(
        [
            ARM_AXIS_TRANSFORM.T
            @ value
            @ ARM_AXIS_TRANSFORM
            @ HAND_CALIBRATION[side]
            for value in rotation
        ]
    )
    return wrist


def write_csv(
    destination: str,
    trajectory: Mapping[str, np.ndarray],
    column_order: str = "left-right",
) -> Path:
    if column_order not in COLUMN_ORDERS:
        raise ValueError(f"column_order must be one of: {', '.join(COLUMN_ORDERS)}")
    timestamp = np.asarray(trajectory["timestamp"], dtype=np.float64)
    matrices = {
        side: np.asarray(trajectory[f"{side}_matrix"], dtype=np.float64)
        for side in ("left", "right")
    }
    for side, matrix in matrices.items():
        if matrix.shape != (len(timestamp), 4, 4):
            raise ValueError(f"{side}_matrix must have shape ({len(timestamp)},4,4)")

    target = Path(destination)
    target.parent.mkdir(parents=True, exist_ok=True)
    sides = COLUMN_ORDERS[column_order]
    header = ["timestamp"]
    for side in sides:
        header.extend(f"{side}_raw_matrix_{row}{column}" for row in range(4) for column in range(4))

    with target.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index, value in enumerate(timestamp):
            row = [float(value)]
            for side in sides:
                row.extend(matrices[side][index].reshape(16).tolist())
            writer.writerow(row)
    return target


def _anchor_positions(csv_config: Any) -> Dict[str, np.ndarray]:
    positions = {side: value.copy() for side, value in DEFAULT_ANCHOR_POSITION.items()}
    if csv_config is None:
        return positions
    if not isinstance(csv_config, Mapping):
        raise ValueError("csv must be a mapping")
    configured = csv_config.get("initial_wrist_position_m", {})
    if not isinstance(configured, Mapping):
        raise ValueError("csv.initial_wrist_position_m must be a mapping")
    for side in ("left", "right"):
        if side in configured:
            value = np.asarray(configured[side], dtype=np.float64)
            if value.shape != (3,):
                raise ValueError(f"csv.initial_wrist_position_m.{side} must contain 3 values")
            positions[side] = value
    return positions


def _compile_hand(
    hand: Any, rate_hz: float, interpolation: Mapping[str, Any]
) -> SampledTrajectory:
    if not isinstance(hand, Mapping):
        raise ValueError("left_hand and right_hand mappings are required")
    mode = hand.get("mode", "waypoints")
    if mode == "hold_initial":
        return sample_trajectory([_anchor()], rate_hz, interpolation)
    if mode != "waypoints":
        raise ValueError("hand mode must be hold_initial or waypoints")
    orientation = hand.get("orientation", {"mode": "fixed_initial"})
    if not isinstance(orientation, Mapping) or orientation.get("mode") != "fixed_initial":
        raise ValueError("orientation.mode must be fixed_initial")
    values = hand.get("waypoints")
    if not isinstance(values, list):
        raise ValueError("waypoint hand requires a waypoints list")
    return sample_trajectory(_ensure_anchor(parse_waypoints(values)), rate_hz, interpolation)


def _anchor() -> Waypoint:
    return Waypoint(position=np.zeros(3), rotation=np.eye(3), name="anchor")


def _ensure_anchor(waypoints: Sequence[Waypoint]) -> Sequence[Waypoint]:
    first = waypoints[0]
    if np.allclose(first.position, 0.0) and np.allclose(first.rotation, np.eye(3)):
        return waypoints
    return [_anchor(), *waypoints]


def _resample_sampled(
    sampled: SampledTrajectory, target_time: np.ndarray
) -> SampledTrajectory:
    if len(sampled.time_s) == 1:
        return SampledTrajectory(
            target_time,
            np.broadcast_to(sampled.position[0], (len(target_time), 3)).copy(),
            np.broadcast_to(sampled.rotation[0], (len(target_time), 3, 3)).copy(),
        )
    positions = np.stack(
        [np.interp(target_time, sampled.time_s, sampled.position[:, axis]) for axis in range(3)],
        axis=-1,
    )
    rotations = []
    for timestamp in target_time:
        if timestamp >= sampled.time_s[-1]:
            rotations.append(sampled.rotation[-1])
            continue
        index = max(0, int(np.searchsorted(sampled.time_s, timestamp, side="right") - 1))
        fraction = (timestamp - sampled.time_s[index]) / (
            sampled.time_s[index + 1] - sampled.time_s[index]
        )
        rotations.append(
            slerp_matrix(sampled.rotation[index], sampled.rotation[index + 1], fraction)
        )
    return SampledTrajectory(target_time, positions, np.stack(rotations))


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="人工编写的 trajectory_csv/v1 YAML")
    parser.add_argument("--output", required=True, help="输出 CSV 路径")
    parser.add_argument(
        "--column-order",
        choices=tuple(COLUMN_ORDERS),
        default="left-right",
        help="矩阵列顺序；默认按需求输出左手后右手",
    )
    args = parser.parse_args(argv)
    output = compile_to_csv(args.source, args.output, args.column_order)
    print(str(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
