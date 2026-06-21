"""MH6 dry-run end-effector integration."""

from .mh6_controller import (
    ActuatorCommand,
    CommandRateLimiter,
    LowDimHandCommand,
    TeleopCalibration,
    low_dim_to_actuator_command,
)
from .mh6_mapping import MH6HandMapper, MappingCalibration, validate_points

__all__ = [
    "ActuatorCommand",
    "CommandRateLimiter",
    "LowDimHandCommand",
    "MH6HandMapper",
    "MH6HandTeleopROS",
    "MappingCalibration",
    "TeleopCalibration",
    "low_dim_to_actuator_command",
    "validate_points",
]


def __getattr__(name):
    if name == "MH6HandTeleopROS":
        from .mh6_teleop_ros import MH6HandTeleopROS

        return MH6HandTeleopROS
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
