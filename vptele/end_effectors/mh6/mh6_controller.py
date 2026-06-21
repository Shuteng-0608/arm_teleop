"""Pure MH6 dry-run actuator conversion and command limiting."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Union


FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
LONG_FINGER_NAMES = ("index", "middle", "ring", "little")
PALM_SOURCE_NAMES = ("thumbSide", "littleSide", "UL", "UR", "LL", "LR")
CALIBRATION_EPSILON = 1e-8


@dataclass
class PalmServoConfig:
    id: int
    name: str
    open_position: int
    closed_position: int
    time: int
    source: Optional[str] = None
    weights: Dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, object]:
        data = {
            "id": self.id,
            "name": self.name,
            "open_position": self.open_position,
            "closed_position": self.closed_position,
            "time": self.time,
        }
        if self.weights:
            data["weights"] = dict(self.weights)
        else:
            data["source"] = self.source
        return data


@dataclass
class TeleopCalibration:
    curl_open: Dict[str, float] = field(default_factory=lambda: {
        "thumb": 0.10,
        "index": 0.20,
        "middle": 0.20,
        "ring": 0.20,
        "little": 0.20,
    })
    curl_closed: Dict[str, float] = field(default_factory=lambda: {
        "thumb": 1.40,
        "index": 2.50,
        "middle": 2.60,
        "ring": 2.55,
        "little": 2.40,
    })
    opposition_open_dist: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.095,
        "middle": 0.105,
        "ring": 0.120,
        "little": 0.135,
    })
    opposition_closed_dist: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.018,
        "middle": 0.020,
        "ring": 0.025,
        "little": 0.030,
    })
    opposition_threshold: float = 0.35
    thumb_curl_gain: float = 0.70
    grasp_weights: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.20,
        "middle": 0.30,
        "ring": 0.30,
        "little": 0.20,
    })
    opposition_horizontal_weights: Dict[str, float] = field(default_factory=lambda: {
        "index": 0.20,
        "middle": 0.35,
        "ring": 0.70,
        "little": 1.00,
    })
    opposition_vertical_weights: Dict[str, float] = field(default_factory=lambda: {
        "index": -0.25,
        "middle": -0.45,
        "ring": 0.75,
        "little": 1.00,
    })
    horizontal_from_grasp: float = 0.55
    horizontal_from_tripod: float = 0.20
    horizontal_from_opposition: float = 0.35
    vertical_from_finger_bias: float = 0.30
    vertical_from_opposition: float = 0.70
    finger_ids: List[int] = field(default_factory=lambda: [1, 2, 3, 4, 5])
    finger_open_positions: Dict[str, int] = field(default_factory=lambda: {
        "thumb": 20,
        "index": 20,
        "middle": 20,
        "ring": 20,
        "little": 20,
    })
    finger_closed_positions: Dict[str, int] = field(default_factory=lambda: {
        "thumb": 1200,
        "index": 1950,
        "middle": 1950,
        "ring": 1950,
        "little": 1950,
    })
    palm_servos: List[PalmServoConfig] = field(default_factory=lambda: [
        PalmServoConfig(1, "palm_1", 500, 650, 80, source="thumbSide"),
        PalmServoConfig(2, "palm_2", 500, 600, 80, weights={"UR": 0.5, "LR": 0.5}),
        PalmServoConfig(3, "palm_3", 500, 560, 80, source="littleSide"),
    ])
    max_finger_delta_per_sec: float = 800.0
    max_palm_delta_per_sec: float = 400.0

    def to_dict(self) -> Dict[str, object]:
        return {
            "curl_open": dict(self.curl_open),
            "curl_closed": dict(self.curl_closed),
            "opposition_open_dist": dict(self.opposition_open_dist),
            "opposition_closed_dist": dict(self.opposition_closed_dist),
            "opposition_threshold": self.opposition_threshold,
            "thumb_curl_gain": self.thumb_curl_gain,
            "grasp_weights": dict(self.grasp_weights),
            "opposition_horizontal_weights": dict(self.opposition_horizontal_weights),
            "opposition_vertical_weights": dict(self.opposition_vertical_weights),
            "horizontal_from_grasp": self.horizontal_from_grasp,
            "horizontal_from_tripod": self.horizontal_from_tripod,
            "horizontal_from_opposition": self.horizontal_from_opposition,
            "vertical_from_finger_bias": self.vertical_from_finger_bias,
            "vertical_from_opposition": self.vertical_from_opposition,
            "finger_ids": list(self.finger_ids),
            "finger_open_positions": dict(self.finger_open_positions),
            "finger_closed_positions": dict(self.finger_closed_positions),
            "palm_servos": [servo.to_dict() for servo in self.palm_servos],
            "max_finger_delta_per_sec": self.max_finger_delta_per_sec,
            "max_palm_delta_per_sec": self.max_palm_delta_per_sec,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "TeleopCalibration":
        if not isinstance(data, dict):
            raise ValueError("Calibration JSON root must be an object")

        required_keys = set(cls().to_dict().keys())
        missing = sorted(required_keys - set(data.keys()))
        if missing:
            raise ValueError(f"Calibration missing required keys: {', '.join(missing)}")

        calibration = cls(
            curl_open=_require_float_dict(data, "curl_open", FINGER_NAMES),
            curl_closed=_require_float_dict(data, "curl_closed", FINGER_NAMES),
            opposition_open_dist=_require_float_dict(data, "opposition_open_dist", LONG_FINGER_NAMES),
            opposition_closed_dist=_require_float_dict(data, "opposition_closed_dist", LONG_FINGER_NAMES),
            opposition_threshold=_require_float(data, "opposition_threshold"),
            thumb_curl_gain=_require_float(data, "thumb_curl_gain"),
            grasp_weights=_require_float_dict(data, "grasp_weights", LONG_FINGER_NAMES),
            opposition_horizontal_weights=_require_float_dict(
                data, "opposition_horizontal_weights", LONG_FINGER_NAMES
            ),
            opposition_vertical_weights=_require_float_dict(
                data, "opposition_vertical_weights", LONG_FINGER_NAMES
            ),
            horizontal_from_grasp=_require_float(data, "horizontal_from_grasp"),
            horizontal_from_tripod=_require_float(data, "horizontal_from_tripod"),
            horizontal_from_opposition=_require_float(data, "horizontal_from_opposition"),
            vertical_from_finger_bias=_require_float(data, "vertical_from_finger_bias"),
            vertical_from_opposition=_require_float(data, "vertical_from_opposition"),
            finger_ids=_require_int_list(data, "finger_ids"),
            finger_open_positions=_require_int_dict(data, "finger_open_positions", FINGER_NAMES),
            finger_closed_positions=_require_int_dict(data, "finger_closed_positions", FINGER_NAMES),
            palm_servos=_require_palm_servos(data, "palm_servos"),
            max_finger_delta_per_sec=_require_float(data, "max_finger_delta_per_sec"),
            max_palm_delta_per_sec=_require_float(data, "max_palm_delta_per_sec"),
        )
        calibration.validate()
        return calibration

    @classmethod
    def load_json(cls, path: Union[str, Path]) -> "TeleopCalibration":
        json_path = Path(path)
        with json_path.open("r", encoding="utf-8") as fp:
            data = json.load(fp)
        return cls.from_dict(data)

    def validate(self) -> None:
        _validate_keys(self.curl_open, FINGER_NAMES, "curl_open")
        _validate_keys(self.curl_closed, FINGER_NAMES, "curl_closed")
        _validate_keys(self.opposition_open_dist, LONG_FINGER_NAMES, "opposition_open_dist")
        _validate_keys(self.opposition_closed_dist, LONG_FINGER_NAMES, "opposition_closed_dist")
        _validate_keys(self.grasp_weights, LONG_FINGER_NAMES, "grasp_weights")
        _validate_keys(self.opposition_horizontal_weights, LONG_FINGER_NAMES, "opposition_horizontal_weights")
        _validate_keys(self.opposition_vertical_weights, LONG_FINGER_NAMES, "opposition_vertical_weights")
        _validate_keys(self.finger_open_positions, FINGER_NAMES, "finger_open_positions")
        _validate_keys(self.finger_closed_positions, FINGER_NAMES, "finger_closed_positions")

        if len(self.finger_ids) != len(FINGER_NAMES):
            raise ValueError("finger_ids must contain exactly five IDs")
        if not self.palm_servos:
            raise ValueError("palm_servos must contain at least one servo")

        for field_name, values in (
            ("curl_open", self.curl_open),
            ("curl_closed", self.curl_closed),
            ("opposition_open_dist", self.opposition_open_dist),
            ("opposition_closed_dist", self.opposition_closed_dist),
            ("grasp_weights", self.grasp_weights),
            ("opposition_horizontal_weights", self.opposition_horizontal_weights),
            ("opposition_vertical_weights", self.opposition_vertical_weights),
        ):
            _validate_finite_values(values.values(), field_name)

        _validate_finite_values([
            self.opposition_threshold,
            self.thumb_curl_gain,
            self.horizontal_from_grasp,
            self.horizontal_from_tripod,
            self.horizontal_from_opposition,
            self.vertical_from_finger_bias,
            self.vertical_from_opposition,
            self.max_finger_delta_per_sec,
            self.max_palm_delta_per_sec,
        ], "scalar calibration values")

        if not 0.0 <= self.opposition_threshold < 1.0:
            raise ValueError("opposition_threshold must be in the range [0, 1)")
        if self.max_finger_delta_per_sec < 0.0 or self.max_palm_delta_per_sec < 0.0:
            raise ValueError("rate limits must be non-negative")

        for finger in FINGER_NAMES:
            if abs(self.curl_closed[finger] - self.curl_open[finger]) <= CALIBRATION_EPSILON:
                raise ValueError(f"curl_open and curl_closed are degenerate for finger: {finger}")

        for finger in LONG_FINGER_NAMES:
            delta = self.opposition_open_dist[finger] - self.opposition_closed_dist[finger]
            if abs(delta) <= CALIBRATION_EPSILON:
                raise ValueError(
                    "opposition_open_dist and opposition_closed_dist are degenerate "
                    f"for thumb-{finger}"
                )

        for device_id in self.finger_ids:
            if not 0 <= device_id <= 255:
                raise ValueError("finger_ids values must be in 0..255")

        seen_palm_names = set()
        for servo in self.palm_servos:
            _validate_palm_servo(servo)
            if servo.name in seen_palm_names:
                raise ValueError(f"Duplicate palm servo name: {servo.name}")
            seen_palm_names.add(servo.name)


@dataclass
class LowDimHandCommand:
    u_thumb: float = 0.0
    u_index: float = 0.0
    u_middle: float = 0.0
    u_ring: float = 0.0
    u_little: float = 0.0
    u_h: float = 0.0
    u_v: float = 0.0

    @classmethod
    def from_mapping_result(cls, low_dim: Dict[str, float]) -> "LowDimHandCommand":
        return cls(
            u_thumb=float(low_dim["u_thumb"]),
            u_index=float(low_dim["u_index"]),
            u_middle=float(low_dim["u_middle"]),
            u_ring=float(low_dim["u_ring"]),
            u_little=float(low_dim["u_little"]),
            u_h=float(low_dim["u_h"]),
            u_v=float(low_dim["u_v"]),
        )


@dataclass
class ActuatorCommand:
    finger_ids: List[int] = field(default_factory=list)
    finger_positions: List[int] = field(default_factory=list)
    palm_ids: List[int] = field(default_factory=list)
    palm_positions: List[int] = field(default_factory=list)
    palm_times: List[int] = field(default_factory=list)


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float:
    if abs(in_max - in_min) <= 1e-12:
        return out_min
    ratio = (value - in_min) / (in_max - in_min)
    return out_min + ratio * (out_max - out_min)


def clamp_to_range(value: float, endpoint_a: float, endpoint_b: float) -> int:
    return int(round(clip(value, min(endpoint_a, endpoint_b), max(endpoint_a, endpoint_b))))


def expand_palm_blocks(command: LowDimHandCommand) -> Dict[str, float]:
    thumb_side = clip(command.u_h - command.u_v, 0.0, 1.0)
    little_side = clip(command.u_h + command.u_v, 0.0, 1.0)
    return {
        "UL": thumb_side,
        "LL": thumb_side,
        "UR": little_side,
        "LR": little_side,
        "thumbSide": thumb_side,
        "littleSide": little_side,
    }


def palm_source_value(source: str, palm_blocks: Dict[str, float]) -> float:
    return palm_blocks[source]


def palm_servo_command_value(servo: PalmServoConfig, palm_blocks: Dict[str, float]) -> float:
    if servo.weights:
        total_weight = sum(abs(weight) for weight in servo.weights.values())
        if total_weight <= 1e-12:
            return 0.0
        return sum(
            palm_source_value(source, palm_blocks) * weight
            for source, weight in servo.weights.items()
        ) / total_weight
    if servo.source is None:
        return 0.0
    return palm_source_value(servo.source, palm_blocks)


def low_dim_to_actuator_command(
    command: LowDimHandCommand,
    calibration: TeleopCalibration,
    palm_times: Optional[Sequence[int]] = None,
) -> ActuatorCommand:
    normalized = {
        "thumb": clip(command.u_thumb, 0.0, 1.0),
        "index": clip(command.u_index, 0.0, 1.0),
        "middle": clip(command.u_middle, 0.0, 1.0),
        "ring": clip(command.u_ring, 0.0, 1.0),
        "little": clip(command.u_little, 0.0, 1.0),
    }

    finger_positions = []
    for finger in FINGER_NAMES:
        open_pos = calibration.finger_open_positions[finger]
        closed_pos = calibration.finger_closed_positions[finger]
        mapped = map_range(normalized[finger], 0.0, 1.0, open_pos, closed_pos)
        finger_positions.append(clamp_to_range(mapped, open_pos, closed_pos))

    palm_blocks = expand_palm_blocks(command)
    palm_positions = []
    for servo in calibration.palm_servos:
        u_palm = clip(palm_servo_command_value(servo, palm_blocks), 0.0, 1.0)
        mapped = map_range(u_palm, 0.0, 1.0, servo.open_position, servo.closed_position)
        palm_positions.append(clamp_to_range(mapped, servo.open_position, servo.closed_position))

    if palm_times is None:
        output_palm_times = [servo.time for servo in calibration.palm_servos]
    else:
        output_palm_times = [int(clip(value, 0, 65535)) for value in palm_times]
        output_palm_times = output_palm_times[:len(palm_positions)]
        while len(output_palm_times) < len(palm_positions):
            output_palm_times.append(calibration.palm_servos[len(output_palm_times)].time)

    return ActuatorCommand(
        finger_ids=list(calibration.finger_ids[:len(finger_positions)]),
        finger_positions=finger_positions,
        palm_ids=[servo.id for servo in calibration.palm_servos],
        palm_positions=palm_positions,
        palm_times=output_palm_times,
    )


class CommandRateLimiter:
    """Per-update actuator command limiter for dry-run safety inspection."""

    def __init__(
        self,
        calibration: TeleopCalibration,
        enabled: bool = True,
        max_delta_per_update: int = 80,
    ) -> None:
        self.calibration = calibration
        self.enabled = bool(enabled)
        self.max_delta_per_update = max(0, int(max_delta_per_update))
        self.previous: Optional[ActuatorCommand] = None

    def reset(self) -> None:
        self.previous = None

    def apply(self, target: ActuatorCommand) -> ActuatorCommand:
        clamped = clamp_actuator_command(target, self.calibration)
        if not self.enabled or self.previous is None:
            self.previous = clamped
            return clamped

        limited = ActuatorCommand(
            finger_ids=list(clamped.finger_ids),
            finger_positions=[
                _limit_step(prev, cur, self.max_delta_per_update)
                for prev, cur in zip(self.previous.finger_positions, clamped.finger_positions)
            ],
            palm_ids=list(clamped.palm_ids),
            palm_positions=[
                _limit_step(prev, cur, self.max_delta_per_update)
                for prev, cur in zip(self.previous.palm_positions, clamped.palm_positions)
            ],
            palm_times=list(clamped.palm_times),
        )
        limited = clamp_actuator_command(limited, self.calibration)
        self.previous = limited
        return limited


def clamp_actuator_command(command: ActuatorCommand, calibration: TeleopCalibration) -> ActuatorCommand:
    finger_positions = []
    for idx, position in enumerate(command.finger_positions[:len(FINGER_NAMES)]):
        finger = FINGER_NAMES[idx]
        finger_positions.append(
            clamp_to_range(
                position,
                calibration.finger_open_positions[finger],
                calibration.finger_closed_positions[finger],
            )
        )

    palm_positions = []
    for idx, position in enumerate(command.palm_positions[:len(calibration.palm_servos)]):
        servo = calibration.palm_servos[idx]
        palm_positions.append(clamp_to_range(position, servo.open_position, servo.closed_position))

    palm_times = [int(clip(value, 0, 65535)) for value in command.palm_times[:len(palm_positions)]]
    while len(palm_times) < len(palm_positions):
        palm_times.append(calibration.palm_servos[len(palm_times)].time)

    return ActuatorCommand(
        finger_ids=list(command.finger_ids[:len(finger_positions)]),
        finger_positions=finger_positions,
        palm_ids=list(command.palm_ids[:len(palm_positions)]),
        palm_positions=palm_positions,
        palm_times=palm_times,
    )


def load_calibration(path: Optional[Union[str, Path]] = None) -> TeleopCalibration:
    if path is None:
        return TeleopCalibration()
    return TeleopCalibration.load_json(path)


def validate_controller_with_synthetic_command(calibration_path: Union[str, Path]) -> Dict[str, object]:
    calibration = load_calibration(calibration_path)
    low_dim = LowDimHandCommand(
        u_thumb=0.5,
        u_index=0.2,
        u_middle=0.4,
        u_ring=0.6,
        u_little=0.8,
        u_h=0.5,
        u_v=0.1,
    )
    raw_command = low_dim_to_actuator_command(low_dim, calibration, palm_times=[50, 50, 50])
    limiter = CommandRateLimiter(calibration, enabled=True, max_delta_per_update=80)
    first = limiter.apply(raw_command)
    target = low_dim_to_actuator_command(
        LowDimHandCommand(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, -1.0),
        calibration,
        palm_times=[50, 50, 50],
    )
    limited = limiter.apply(target)
    clamped = clamp_actuator_command(limited, calibration)
    return {
        "finger_count": len(clamped.finger_positions),
        "palm_count": len(clamped.palm_positions),
        "palm_times": list(clamped.palm_times),
        "rate_limited_valid": clamped == limited,
        "first_finger_positions": list(first.finger_positions),
        "limited_finger_positions": list(limited.finger_positions),
    }


def _limit_step(previous: int, target: int, max_delta: int) -> int:
    if max_delta <= 0:
        return target
    delta = int(target) - int(previous)
    if abs(delta) <= max_delta:
        return int(target)
    return int(previous) + int(math.copysign(max_delta, delta))


def _validate_keys(values: Dict[str, object], required_keys: Sequence[str], name: str) -> None:
    missing = [key for key in required_keys if key not in values]
    if missing:
        raise ValueError(f"{name} missing required keys: {', '.join(missing)}")


def _validate_finite_values(values: Sequence[float], name: str) -> None:
    for value in values:
        if not math.isfinite(float(value)):
            raise ValueError(f"{name} must contain finite numbers")


def _require_mapping(data: Dict[str, object], key: str) -> Dict[str, object]:
    value = data[key]
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be an object")
    return value


def _require_float(data: Dict[str, object], key: str) -> float:
    value = data[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{key} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{key} must be finite")
    return result


def _require_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _require_float_dict(
    data: Dict[str, object],
    key: str,
    required_keys: Optional[Sequence[str]] = None,
) -> Dict[str, float]:
    source = _require_mapping(data, key)
    if required_keys is not None:
        _validate_keys(source, required_keys, key)
    result = {}
    for item_key, value in source.items():
        if not isinstance(item_key, str):
            raise ValueError(f"{key} keys must be strings")
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{key}.{item_key} must be a number")
        result[item_key] = float(value)
    _validate_finite_values(list(result.values()), key)
    return result


def _require_int_dict(
    data: Dict[str, object],
    key: str,
    required_keys: Optional[Sequence[str]] = None,
) -> Dict[str, int]:
    source = _require_mapping(data, key)
    if required_keys is not None:
        _validate_keys(source, required_keys, key)
    result = {}
    for item_key, value in source.items():
        if not isinstance(item_key, str):
            raise ValueError(f"{key} keys must be strings")
        result[item_key] = _require_int(value, f"{key}.{item_key}")
    return result


def _require_int_list(data: Dict[str, object], key: str) -> List[int]:
    value = data[key]
    if not isinstance(value, list):
        raise ValueError(f"{key} must be a list")
    return [_require_int(item, f"{key}[{idx}]") for idx, item in enumerate(value)]


def _require_palm_servos(data: Dict[str, object], key: str) -> List[PalmServoConfig]:
    value = data[key]
    if not isinstance(value, list):
        raise ValueError(f"{key} must be a list")

    servos = []
    for idx, item in enumerate(value):
        if not isinstance(item, dict):
            raise ValueError(f"{key}[{idx}] must be an object")

        has_source = "source" in item
        has_weights = "weights" in item
        if has_source == has_weights:
            raise ValueError(f"{key}[{idx}] must contain exactly one of source or weights")

        name = item.get("name")
        if not isinstance(name, str) or not name:
            raise ValueError(f"{key}[{idx}].name must be a non-empty string")

        source = None
        weights = {}
        if has_source:
            source_value = item["source"]
            if not isinstance(source_value, str):
                raise ValueError(f"{key}[{idx}].source must be a string")
            source = source_value
        else:
            weights = _require_float_dict({"weights": item["weights"]}, "weights")

        servo = PalmServoConfig(
            id=_require_int(item.get("id"), f"{key}[{idx}].id"),
            name=name,
            open_position=_require_int(item.get("open_position"), f"{key}[{idx}].open_position"),
            closed_position=_require_int(item.get("closed_position"), f"{key}[{idx}].closed_position"),
            time=_require_int(item.get("time"), f"{key}[{idx}].time"),
            source=source,
            weights=weights,
        )
        _validate_palm_servo(servo)
        servos.append(servo)
    return servos


def _validate_palm_servo(servo: PalmServoConfig) -> None:
    if not 0 <= servo.id <= 255:
        raise ValueError(f"palm servo {servo.name} id must be in 0..255")
    if not 0 <= servo.time <= 65535:
        raise ValueError(f"palm servo {servo.name} time must be in 0..65535")
    _validate_finite_values(
        [servo.open_position, servo.closed_position],
        f"palm servo {servo.name} actuator positions",
    )

    if servo.weights:
        for source_name, weight in servo.weights.items():
            if source_name not in PALM_SOURCE_NAMES:
                raise ValueError(
                    f"palm servo {servo.name} weights contain invalid source: {source_name}"
                )
            if not math.isfinite(weight):
                raise ValueError(f"palm servo {servo.name} weights must be finite")
    elif servo.source not in PALM_SOURCE_NAMES:
        raise ValueError(f"palm servo {servo.name} source is invalid: {servo.source}")
