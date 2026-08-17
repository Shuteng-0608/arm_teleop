import csv
import hashlib
from dataclasses import dataclass

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
    transform: np.ndarray


def _validated_transform(transform):
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4):
        raise ValueError("right wrist transform must be 4x4")
    if not np.all(np.isfinite(matrix)):
        raise ValueError("right wrist transform contains non-finite values")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError("right wrist transform has an invalid homogeneous row")
    return matrix


def load_right_wrist_frames(csv_path):
    frames = []
    previous_timestamp = None
    with open(csv_path, "r", encoding="utf-8", newline="") as input_file:
        reader = csv.DictReader(input_file)
        required = {"timestamp"}
        required.update(
            "right_raw_matrix_{}{}".format(row, column)
            for row in range(4)
            for column in range(4)
        )
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "trajectory CSV is missing columns: {}".format(
                    ", ".join(sorted(missing))
                )
            )

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
