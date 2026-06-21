"""MH6 dry-run end-effector integration."""

from .mh6_backend import (
    DisabledHardwareBackend,
    DryRunBackend,
    MH6Backend,
    ModbusHardwareBackend,
    build_mh6_register_block,
    create_backend,
)
from .mh6_controller import (
    ActuatorCommand,
    CommandRateLimiter,
    LowDimHandCommand,
    TeleopCalibration,
    low_dim_to_actuator_command,
)
from .mh6_mapping import MH6HandMapper, MappingCalibration, validate_points
from .mh6_safety import (
    REQUIRED_HARDWARE_CONFIRM,
    HardwareSafetyConfig,
    HardwareSafetyDecision,
    evaluate_hardware_safety,
    validate_actuator_command,
)

__all__ = [
    "ActuatorCommand",
    "CommandRateLimiter",
    "DisabledHardwareBackend",
    "DryRunBackend",
    "LowDimHandCommand",
    "MH6Backend",
    "MH6HandMapper",
    "MH6HandTeleopROS",
    "ModbusHardwareBackend",
    "MappingCalibration",
    "HardwareSafetyConfig",
    "HardwareSafetyDecision",
    "REQUIRED_HARDWARE_CONFIRM",
    "TeleopCalibration",
    "build_mh6_register_block",
    "create_backend",
    "evaluate_hardware_safety",
    "low_dim_to_actuator_command",
    "validate_actuator_command",
    "validate_points",
]


def __getattr__(name):
    if name == "MH6HandTeleopROS":
        from .mh6_teleop_ros import MH6HandTeleopROS

        return MH6HandTeleopROS
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
