import csv
import hashlib
import math
from dataclasses import dataclass


JOINT_LIMITS = (
    (-3.1, 3.1),
    (-3.1, 3.1),
    (-3.1, 3.1),
    (0.0, 2.26),
    (-3.1, 3.1),
    (-1.22, 1.22),
    (-0.7853981633974483, 0.7853981633974483),
)
EXPECTED_SOURCE_SHA256 = (
    "84f41a2dba293eaa2fbd0c9286ecab18b754bb321dd67dfa7b1bf60ffe4ce10b"
)
EXPECTED_TRAJECTORY_SHA256 = (
    "55731dbcbf89006f3476ab4ef13f1568bd6c67b9f26fdb2e173057c4729644f4"
)


@dataclass(frozen=True)
class JointFrame:
    index: int
    timestamp: float
    joints: tuple


@dataclass(frozen=True)
class TrajectorySummary:
    frame_count: int
    duration: float
    minimum_dt: float
    maximum_dt: float
    maximum_step: float
    maximum_velocity: float
    file_sha256: str


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as input_file:
        for chunk in iter(lambda: input_file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_true(value, field, index):
    if value != "True":
        raise ValueError(
            "frame {} has invalid {}={!r}".format(index, field, value)
        )


def load_redundancy_trajectory(
    path,
    expected_frame_count=769,
    expected_source_sha256=EXPECTED_SOURCE_SHA256,
    expected_file_sha256=EXPECTED_TRAJECTORY_SHA256,
    maximum_step=0.03,
    maximum_velocity=0.8,
):
    actual_file_sha256 = file_sha256(path)
    if expected_file_sha256 and actual_file_sha256 != expected_file_sha256:
        raise ValueError(
            "trajectory SHA-256 mismatch: expected {}, got {}".format(
                expected_file_sha256, actual_file_sha256
            )
        )

    joint_fields = ["redundancy_q{}".format(index) for index in range(1, 8)]
    required_fields = {
        "frame_index",
        "source_timestamp",
        "source_sha256",
        "redundancy_success",
        "redundancy_held_previous",
        "redundancy_status",
    }.union(joint_fields)
    frames = []

    with open(path, "r", encoding="utf-8", newline="") as input_file:
        reader = csv.DictReader(input_file)
        missing = required_fields.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "trajectory is missing columns: {}".format(
                    ", ".join(sorted(missing))
                )
            )

        for index, row in enumerate(reader):
            if int(row["frame_index"]) != index:
                raise ValueError("frame_index is not contiguous at row {}".format(index))
            if expected_source_sha256 and row["source_sha256"] != expected_source_sha256:
                raise ValueError("source SHA-256 mismatch at frame {}".format(index))
            _parse_true(row["redundancy_success"], "redundancy_success", index)
            if row["redundancy_held_previous"] != "False":
                raise ValueError(
                    "frame {} reused a previous solution".format(index)
                )
            if row["redundancy_status"] != "redundancy_selector:selected":
                raise ValueError(
                    "frame {} was not selected by the redundancy solver".format(index)
                )

            timestamp = float(row["source_timestamp"])
            joints = tuple(float(row[field]) for field in joint_fields)
            if not math.isfinite(timestamp) or not all(
                math.isfinite(value) for value in joints
            ):
                raise ValueError("frame {} contains a non-finite value".format(index))
            if frames and timestamp <= frames[-1].timestamp:
                raise ValueError(
                    "timestamps are not strictly increasing at frame {}".format(index)
                )
            for joint_index, (value, limits) in enumerate(
                zip(joints, JOINT_LIMITS), start=1
            ):
                if not limits[0] <= value <= limits[1]:
                    raise ValueError(
                        "frame {} q{}={} is outside [{}, {}]".format(
                            index, joint_index, value, limits[0], limits[1]
                        )
                    )
            frames.append(JointFrame(index, timestamp, joints))

    if len(frames) != expected_frame_count:
        raise ValueError(
            "expected {} frames, got {}".format(expected_frame_count, len(frames))
        )

    time_steps = [
        current.timestamp - previous.timestamp
        for previous, current in zip(frames, frames[1:])
    ]
    joint_steps = [
        max(abs(current - previous) for previous, current in zip(a.joints, b.joints))
        for a, b in zip(frames, frames[1:])
    ]
    joint_velocities = [
        step / dt for step, dt in zip(joint_steps, time_steps)
    ]
    observed_step = max(joint_steps)
    observed_velocity = max(joint_velocities)
    if observed_step > maximum_step:
        raise ValueError(
            "maximum joint step {:.6f} rad exceeds {:.6f} rad".format(
                observed_step, maximum_step
            )
        )
    if observed_velocity > maximum_velocity:
        raise ValueError(
            "maximum joint velocity {:.6f} rad/s exceeds {:.6f} rad/s".format(
                observed_velocity, maximum_velocity
            )
        )

    return frames, TrajectorySummary(
        frame_count=len(frames),
        duration=frames[-1].timestamp - frames[0].timestamp,
        minimum_dt=min(time_steps),
        maximum_dt=max(time_steps),
        maximum_step=observed_step,
        maximum_velocity=observed_velocity,
        file_sha256=actual_file_sha256,
    )
