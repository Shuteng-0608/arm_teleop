import math
from dataclasses import dataclass

from core.right_arm_trajectory import file_sha256, load_right_wrist_frames
from core.right_joint_playback import JOINT_LIMITS


EXPECTED_SOURCE_SHA256 = (
    "84f41a2dba293eaa2fbd0c9286ecab18b754bb321dd67dfa7b1bf60ffe4ce10b"
)


@dataclass(frozen=True)
class SourceTrajectorySummary:
    frame_count: int
    duration: float
    minimum_dt: float
    maximum_dt: float
    file_sha256: str


@dataclass(frozen=True)
class JointTransition:
    maximum_step: float
    maximum_velocity: float


def load_teleop_trajectory(
    path,
    expected_frame_count=769,
    expected_file_sha256=EXPECTED_SOURCE_SHA256,
):
    actual_hash = file_sha256(path)
    if expected_file_sha256 and actual_hash != expected_file_sha256:
        raise ValueError(
            "source trajectory SHA-256 mismatch: expected {}, got {}".format(
                expected_file_sha256, actual_hash
            )
        )
    frames = load_right_wrist_frames(path)
    if len(frames) != expected_frame_count:
        raise ValueError(
            "expected {} source frames, got {}".format(
                expected_frame_count, len(frames)
            )
        )
    time_steps = [
        current.timestamp - previous.timestamp
        for previous, current in zip(frames, frames[1:])
    ]
    return frames, SourceTrajectorySummary(
        frame_count=len(frames),
        duration=frames[-1].timestamp - frames[0].timestamp,
        minimum_dt=min(time_steps),
        maximum_dt=max(time_steps),
        file_sha256=actual_hash,
    )


def validate_online_solution(
    joints,
    previous_joints=None,
    source_dt=None,
    maximum_step=0.03,
    maximum_velocity=0.8,
):
    values = tuple(float(value) for value in joints)
    if len(values) != 7 or not all(math.isfinite(value) for value in values):
        raise ValueError("IK solution must contain seven finite joints")
    for index, (value, limits) in enumerate(zip(values, JOINT_LIMITS), start=1):
        if not limits[0] <= value <= limits[1]:
            raise ValueError(
                "IK q{}={} is outside [{}, {}]".format(
                    index, value, limits[0], limits[1]
                )
            )

    if previous_joints is None:
        return values, JointTransition(0.0, 0.0)
    previous = tuple(float(value) for value in previous_joints)
    if len(previous) != 7 or not all(math.isfinite(value) for value in previous):
        raise ValueError("previous IK solution must contain seven finite joints")
    if source_dt is None or not math.isfinite(source_dt) or source_dt <= 0.0:
        raise ValueError("source_dt must be positive for an IK transition")

    observed_step = max(abs(a - b) for a, b in zip(previous, values))
    observed_velocity = observed_step / source_dt
    if observed_step > maximum_step:
        raise ValueError(
            "online IK step {:.6f} rad exceeds {:.6f} rad".format(
                observed_step, maximum_step
            )
        )
    if observed_velocity > maximum_velocity:
        raise ValueError(
            "online IK velocity {:.6f} rad/s exceeds {:.6f} rad/s".format(
                observed_velocity, maximum_velocity
            )
        )
    return values, JointTransition(observed_step, observed_velocity)


def rounded_solver_state(joints):
    return [round(float(value), 4) for value in joints]
