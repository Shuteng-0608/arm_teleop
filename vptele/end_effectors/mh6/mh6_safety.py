"""Pure safety policy helpers for MH6 hardware gating."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

from end_effectors.mh6.mh6_controller import ActuatorCommand, FINGER_NAMES, TeleopCalibration


REQUIRED_HARDWARE_CONFIRM = "I_UNDERSTAND_MH6_HARDWARE_RISK"
MAX_INITIAL_HARDWARE_UPDATE_HZ = 20.0
MAX_PALM_TIME_MS = 1000


@dataclass(frozen=True)
class HardwareSafetyConfig:
    backend: str
    enable_hardware: bool
    dry_run: bool
    hardware_confirm: str
    port: str
    baudrate: int
    hardware_update_hz: float
    max_consecutive_failures: int
    command_timeout_sec: float


@dataclass(frozen=True)
class HardwareSafetyDecision:
    requested: bool
    eligible: bool
    reason: str


def parse_hardware_safety_config(config: dict) -> HardwareSafetyConfig:
    config = config or {}
    return HardwareSafetyConfig(
        backend=str(config.get("backend", "dry_run")).strip().lower(),
        enable_hardware=bool(config.get("enable_hardware", False)),
        dry_run=bool(config.get("dry_run", True)),
        hardware_confirm=str(config.get("hardware_confirm", "") or ""),
        port=str(config.get("port", "") or "").strip(),
        baudrate=_as_int(config.get("baudrate", 0), "baudrate"),
        hardware_update_hz=_as_float(config.get("hardware_update_hz", 0.0), "hardware_update_hz"),
        max_consecutive_failures=_as_int(
            config.get("max_consecutive_failures", 0),
            "max_consecutive_failures",
        ),
        command_timeout_sec=_as_float(
            config.get("command_timeout_sec", 0.0),
            "command_timeout_sec",
        ),
    )


def evaluate_hardware_safety(config: dict) -> HardwareSafetyDecision:
    try:
        safety_config = parse_hardware_safety_config(config)
    except (TypeError, ValueError) as exc:
        return HardwareSafetyDecision(
            requested=_hardware_requested(config or {}),
            eligible=False,
            reason=f"invalid hardware safety config: {exc}",
        )

    requested = (
        safety_config.enable_hardware
        or safety_config.backend in ("hardware", "modbus")
    )
    if not requested:
        return HardwareSafetyDecision(
            requested=False,
            eligible=False,
            reason="hardware not requested",
        )

    checks = (
        (safety_config.backend == "modbus", "backend must be 'modbus'"),
        (safety_config.enable_hardware is True, "enable_hardware must be true"),
        (safety_config.dry_run is False, "dry_run must be false"),
        (
            safety_config.hardware_confirm == REQUIRED_HARDWARE_CONFIRM,
            "hardware_confirm token is missing or incorrect",
        ),
        (bool(safety_config.port), "port must be a non-empty string"),
        (safety_config.baudrate > 0, "baudrate must be a positive integer"),
        (
            0.0 < safety_config.hardware_update_hz <= MAX_INITIAL_HARDWARE_UPDATE_HZ,
            f"hardware_update_hz must be in (0, {MAX_INITIAL_HARDWARE_UPDATE_HZ}]",
        ),
        (safety_config.command_timeout_sec > 0.0, "command_timeout_sec must be positive"),
        (
            safety_config.max_consecutive_failures > 0,
            "max_consecutive_failures must be positive",
        ),
    )
    for ok, reason in checks:
        if not ok:
            return HardwareSafetyDecision(requested=True, eligible=False, reason=reason)

    return HardwareSafetyDecision(
        requested=True,
        eligible=True,
        reason=(
            "hardware configuration is eligible, but ModbusHardwareBackend is not "
            "implemented in this milestone"
        ),
    )


def validate_actuator_command(
    command: ActuatorCommand,
    calibration: TeleopCalibration,
) -> Tuple[bool, str]:
    if len(command.finger_positions) != 5:
        return False, "finger_positions must contain exactly 5 values"
    if len(command.palm_positions) != 3:
        return False, "palm_positions must contain exactly 3 values"
    if len(command.palm_times) != 3:
        return False, "palm_times must contain exactly 3 values"

    for idx, value in enumerate(command.finger_positions):
        ok, reason = _is_finite_int(value)
        if not ok:
            return False, f"finger_positions[{idx}] {reason}"
        finger = FINGER_NAMES[idx]
        lo = min(
            calibration.finger_open_positions[finger],
            calibration.finger_closed_positions[finger],
        )
        hi = max(
            calibration.finger_open_positions[finger],
            calibration.finger_closed_positions[finger],
        )
        if not lo <= int(value) <= hi:
            return False, f"finger_positions[{idx}]={value} outside range [{lo}, {hi}]"

    if len(calibration.palm_servos) < 3:
        return False, "calibration must define at least 3 palm servos"
    for idx, value in enumerate(command.palm_positions):
        ok, reason = _is_finite_int(value)
        if not ok:
            return False, f"palm_positions[{idx}] {reason}"
        servo = calibration.palm_servos[idx]
        lo = min(servo.open_position, servo.closed_position)
        hi = max(servo.open_position, servo.closed_position)
        if not lo <= int(value) <= hi:
            return False, f"palm_positions[{idx}]={value} outside range [{lo}, {hi}]"

    for idx, value in enumerate(command.palm_times):
        ok, reason = _is_finite_int(value)
        if not ok:
            return False, f"palm_times[{idx}] {reason}"
        if not 1 <= int(value) <= MAX_PALM_TIME_MS:
            return False, f"palm_times[{idx}]={value} outside range [1, {MAX_PALM_TIME_MS}]"

    return True, "ok"


def validate_safety_helpers(calibration: TeleopCalibration) -> dict:
    dry_run_decision = evaluate_hardware_safety({
        "backend": "dry_run",
        "enable_hardware": False,
        "dry_run": True,
        "hardware_confirm": "",
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "hardware_update_hz": 20,
        "command_timeout_sec": 0.5,
        "max_consecutive_failures": 3,
    })
    eligible_decision = evaluate_hardware_safety({
        "backend": "modbus",
        "enable_hardware": True,
        "dry_run": False,
        "hardware_confirm": REQUIRED_HARDWARE_CONFIRM,
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "hardware_update_hz": 20,
        "command_timeout_sec": 0.5,
        "max_consecutive_failures": 3,
    })
    missing_confirm_decision = evaluate_hardware_safety({
        "backend": "modbus",
        "enable_hardware": True,
        "dry_run": False,
        "hardware_confirm": "",
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "hardware_update_hz": 20,
        "command_timeout_sec": 0.5,
        "max_consecutive_failures": 3,
    })
    invalid_baud_decision = evaluate_hardware_safety({
        "backend": "modbus",
        "enable_hardware": True,
        "dry_run": False,
        "hardware_confirm": REQUIRED_HARDWARE_CONFIRM,
        "port": "/dev/ttyUSB0",
        "baudrate": 0,
        "hardware_update_hz": 20,
        "command_timeout_sec": 0.5,
        "max_consecutive_failures": 3,
    })
    invalid_rate_decision = evaluate_hardware_safety({
        "backend": "modbus",
        "enable_hardware": True,
        "dry_run": False,
        "hardware_confirm": REQUIRED_HARDWARE_CONFIRM,
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "hardware_update_hz": 50,
        "command_timeout_sec": 0.5,
        "max_consecutive_failures": 3,
    })

    valid_command = ActuatorCommand(
        finger_ids=list(calibration.finger_ids),
        finger_positions=[
            calibration.finger_open_positions[finger]
            for finger in FINGER_NAMES
        ],
        palm_ids=[servo.id for servo in calibration.palm_servos[:3]],
        palm_positions=[
            servo.open_position
            for servo in calibration.palm_servos[:3]
        ],
        palm_times=[50, 50, 50],
    )
    valid_command_result = validate_actuator_command(valid_command, calibration)
    invalid_command = ActuatorCommand(
        finger_ids=list(calibration.finger_ids),
        finger_positions=[999999, 0, 0, 0, 0],
        palm_ids=[servo.id for servo in calibration.palm_servos[:3]],
        palm_positions=[
            servo.open_position
            for servo in calibration.palm_servos[:3]
        ],
        palm_times=[50, 50, 50],
    )
    invalid_command_result = validate_actuator_command(invalid_command, calibration)

    return {
        "dry_run_not_eligible": not dry_run_decision.requested and not dry_run_decision.eligible,
        "confirmed_modbus_eligible": eligible_decision.requested and eligible_decision.eligible,
        "missing_confirm_ineligible": missing_confirm_decision.requested and not missing_confirm_decision.eligible,
        "invalid_baudrate_ineligible": invalid_baud_decision.requested and not invalid_baud_decision.eligible,
        "invalid_update_rate_ineligible": invalid_rate_decision.requested and not invalid_rate_decision.eligible,
        "valid_command_ok": valid_command_result == (True, "ok"),
        "invalid_command_rejected": invalid_command_result[0] is False,
    }


def _hardware_requested(config: dict) -> bool:
    backend = str(config.get("backend", "dry_run")).strip().lower()
    return bool(config.get("enable_hardware", False)) or backend in ("hardware", "modbus")


def _is_finite_int(value) -> Tuple[bool, str]:
    if isinstance(value, bool):
        return False, "must be an integer"
    if not isinstance(value, int):
        return False, "must be an integer"
    if not math.isfinite(float(value)):
        return False, "must be finite"
    return True, "ok"


def _as_int(value, name: str) -> int:
    if isinstance(value, bool):
        raise TypeError(f"{name} must be an integer")
    if not isinstance(value, int):
        raise TypeError(f"{name} must be an integer")
    return int(value)


def _as_float(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be finite")
    return result
