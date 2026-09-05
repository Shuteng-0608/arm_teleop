import csv
import hashlib
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation


INITIAL_RIGHT_ROBOT_POSE = np.array(
    [0.3011, -0.3580, 0.2282, 3.1923149, -0.036102, -0.0007987],
    dtype=float,
)
INITIAL_RIGHT_ROBOT_POSE_FOR_PREDICTOR = np.array(
    [0.26164, -0.35714, 0.06982, 3.1923149, -0.036102, -0.0007987],
    dtype=float,
)
INITIAL_RIGHT_JOINTS = np.array(
    [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005],
    dtype=float,
)
INITIAL_RIGHT_ARM_ANGLE = -0.7

_RIGHT_HAND_REFERENCE_ROTATION = np.array(
    [[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]],
    dtype=float,
)
_HAND_TO_ARM_AXES = np.array(
    [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]],
    dtype=float,
)


@dataclass(frozen=True)
class RightWristFrame:
    index: int
    timestamp: float
    transform: Optional[np.ndarray]
    target_pose: Optional[np.ndarray] = None
    initial_joints: Optional[tuple] = None
    initial_arm_angle: Optional[float] = None


def _validated_transform(transform):
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("right wrist transform must be 4x4")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("right wrist transform contains non-finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("right wrist transform has an invalid homogeneous row")
    return matrix


def _validated_target_pose(target_pose):
    target = np.asarray(target_pose, dtype=float)
    if target.shape != (7,) or not np.all(np.isfinite(target)):
        raise ValueError("target pose must contain seven finite values")
    quaternion_norm = np.linalg.norm(target[3:])
    if quaternion_norm <= 1.0e-12:
        raise ValueError("target pose quaternion must be non-zero")
    target[3:] /= quaternion_norm
    return target


def _load_raw_wrist_frames(reader):
    frames = []
    previous_timestamp = None
    for index, row in enumerate(reader):
        timestamp = float(row["timestamp"])
        if not np.isfinite(timestamp):
            raise ValueError("frame {} has a non-finite timestamp".format(index))
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ValueError(
                "trajectory timestamps must be strictly increasing at frame {}".format(
                    index
                )
            )

        transform = np.empty((4, 4), dtype=float)
        for matrix_row in range(4):
            for matrix_column in range(4):
                transform[matrix_row, matrix_column] = float(
                    row[
                        "right_raw_matrix_{}{}".format(
                            matrix_row, matrix_column
                        )
                    ]
                )
        frames.append(
            RightWristFrame(
                index=index,
                timestamp=timestamp,
                transform=_validated_transform(transform).copy(),
            )
        )
        previous_timestamp = timestamp
    return frames


def _load_target_pose_frames(reader):
    frames = []
    previous_timestamp = None
    initial_joints = None
    initial_arm_angle = None
    initial_joint_fields = ["initial_q{}".format(index) for index in range(1, 8)]
    for index, row in enumerate(reader):
        frame_index = int(row["frame_index"])
        if frame_index != index:
            raise ValueError("frame_index is not contiguous at row {}".format(index))
        timestamp = float(row["timestamp_s"])
        if not np.isfinite(timestamp):
            raise ValueError("frame {} has a non-finite timestamp".format(index))
        if previous_timestamp is not None and timestamp <= previous_timestamp:
            raise ValueError(
                "trajectory timestamps must be strictly increasing at frame {}".format(
                    index
                )
            )
        target = _validated_target_pose(
            [
                row["target_x"],
                row["target_y"],
                row["target_z"],
                row["target_qw"],
                row["target_qx"],
                row["target_qy"],
                row["target_qz"],
            ]
        )

        if index == 0 and all(field in row for field in initial_joint_fields):
            initial_joints = tuple(float(row[field]) for field in initial_joint_fields)
            if not all(np.isfinite(initial_joints)):
                raise ValueError("initial joints must be finite")
            if "initial_arm_angle" in row:
                initial_arm_angle = float(row["initial_arm_angle"])
                if not np.isfinite(initial_arm_angle):
                    raise ValueError("initial arm angle must be finite")

        frames.append(
            RightWristFrame(
                index=frame_index,
                timestamp=timestamp,
                transform=None,
                target_pose=target,
                initial_joints=initial_joints if index == 0 else None,
                initial_arm_angle=initial_arm_angle if index == 0 else None,
            )
        )
        previous_timestamp = timestamp
    return frames


def load_right_wrist_frames(csv_path):
    with open(csv_path, "r", encoding="utf-8", newline="") as input_file:
        reader = csv.DictReader(input_file)
        fields = set(reader.fieldnames or [])
        raw_fields = {"timestamp"}
        raw_fields.update(
            "right_raw_matrix_{}{}".format(row, column)
            for row in range(4)
            for column in range(4)
        )
        target_fields = {
            "frame_index",
            "timestamp_s",
            "target_x",
            "target_y",
            "target_z",
            "target_qw",
            "target_qx",
            "target_qy",
            "target_qz",
        }
        if raw_fields.issubset(fields):
            frames = _load_raw_wrist_frames(reader)
        elif target_fields.issubset(fields):
            frames = _load_target_pose_frames(reader)
        else:
            missing = sorted((raw_fields | target_fields) - fields)
            raise ValueError(
                "trajectory CSV is missing required columns for raw or target format: {}".format(
                    ", ".join(missing)
                )
            )

    if not frames:
        raise ValueError("trajectory CSV contains no data frames")
    return frames


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        for chunk in iter(lambda: input_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def map_right_hand_to_robot_euler(
    hand_transform,
    initial_hand_position,
    initial_robot_pose=INITIAL_RIGHT_ROBOT_POSE,
):
    hand_transform = _validated_transform(hand_transform)
    initial_hand_position = np.asarray(initial_hand_position, dtype=float)
    if initial_hand_position.shape != (3,) or not np.all(
        np.isfinite(initial_hand_position)
    ):
        raise ValueError("initial right hand position must contain three finite values")

    target_pose = np.asarray(initial_robot_pose, dtype=float).copy()
    if target_pose.shape != (6,) or not np.all(np.isfinite(target_pose)):
        raise ValueError("initial right robot pose must contain six finite values")

    hand_offset = hand_transform[:3, 3] - initial_hand_position
    target_pose[0] += hand_offset[1]
    target_pose[1] += hand_offset[2]
    target_pose[2] += hand_offset[0]

    relative_rotation = (
        hand_transform[:3, :3]
        @ np.linalg.inv(_RIGHT_HAND_REFERENCE_ROTATION)
    )
    rotation_in_arm = (
        _HAND_TO_ARM_AXES @ relative_rotation @ _HAND_TO_ARM_AXES.T
    )
    initial_robot_rotation = Rotation.from_euler(
        "XYZ", target_pose[3:], degrees=False
    ).as_matrix()
    target_pose[3:] = Rotation.from_matrix(
        rotation_in_arm @ initial_robot_rotation
    ).as_euler("XYZ", degrees=False)
    return target_pose


def euler_pose_to_quaternion_wxyz(euler_pose):
    euler_pose = np.asarray(euler_pose, dtype=float)
    if euler_pose.shape != (6,) or not np.all(np.isfinite(euler_pose)):
        raise ValueError("Euler pose must contain six finite values")
    quaternion_xyzw = Rotation.from_euler(
        "XYZ", euler_pose[3:], degrees=False
    ).as_quat()
    return np.concatenate(
        [euler_pose[:3], [quaternion_xyzw[3]], quaternion_xyzw[:3]]
    )


def euler_pose_to_quaternion_xyzw(euler_pose):
    euler_pose = np.asarray(euler_pose, dtype=float)
    if euler_pose.shape != (6,) or not np.all(np.isfinite(euler_pose)):
        raise ValueError("Euler pose must contain six finite values")
    quaternion_xyzw = Rotation.from_euler(
        "XYZ", euler_pose[3:], degrees=False
    ).as_quat()
    return np.concatenate([euler_pose[:3], quaternion_xyzw])


class RightArmTrajectoryMapper:
    def __init__(self, initial_hand_transform):
        initial_hand_transform = _validated_transform(initial_hand_transform)
        self.initial_hand_position = initial_hand_transform[:3, 3].copy()

    def ik_target(self, hand_transform):
        euler_pose = map_right_hand_to_robot_euler(
            hand_transform,
            self.initial_hand_position,
            INITIAL_RIGHT_ROBOT_POSE,
        )
        return euler_pose_to_quaternion_wxyz(euler_pose)

    def predictor_target(self, hand_transform):
        euler_pose = map_right_hand_to_robot_euler(
            hand_transform,
            self.initial_hand_position,
            INITIAL_RIGHT_ROBOT_POSE_FOR_PREDICTOR,
        )
        return euler_pose_to_quaternion_xyzw(euler_pose)
