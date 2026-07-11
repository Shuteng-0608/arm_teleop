"""MH6 palm motor calibration, separate from geometry and teleoperation intent."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

try:
    from .alpha_normalization import (
        AlphaRange,
        signed_to_alpha_triplet,
    )
except ImportError:  # Support importing mh6_palm_solver from vptele on PYTHONPATH.
    from alpha_normalization import AlphaRange, signed_to_alpha_triplet


ARPHA2_STAR_RANGE = AlphaRange(minimum=-90.8, zero=0.0, maximum=31.1)
ARPHA3_RANGE = AlphaRange(minimum=-180.0, zero=0.0, maximum=59.0)
DEFAULT_MOTOR_LIMITS: Tuple[Tuple[float, float], ...] = (
    (0.0, 1000.0),
    (0.0, 1000.0),
    (0.0, 1000.0),
)


def require_arpha1_range(arpha1_range: Optional[AlphaRange]) -> AlphaRange:
    """Reject missing arpha1 calibration; no positive limit is assumed."""

    if arpha1_range is None:
        raise ValueError(
            "arpha1_range is required because the positive arpha1 limit is unknown"
        )
    if not isinstance(arpha1_range, AlphaRange):
        raise ValueError("arpha1_range must be an AlphaRange")
    return arpha1_range


def _motor_value(alpha: float, motor_index: int) -> float:
    if motor_index == 0:
        return 500.0 + (99.0 / 23.6) * alpha
    if motor_index == 1:
        return 500.0 - (380.0 / 90.8) * alpha
    if motor_index == 2:
        return 247.0 - (753.0 / 180.0) * alpha
    raise ValueError(f"invalid motor index: {motor_index}")


def _apply_motor_limit(
    value: float,
    limits: Tuple[float, float],
    motor_index: int,
    clip: bool,
) -> float:
    lower, upper = (float(limits[0]), float(limits[1]))
    if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
        raise ValueError(f"invalid limits for motor{motor_index + 1}")
    if clip:
        return min(max(value, lower), upper)
    if not lower <= value <= upper:
        raise ValueError(
            f"motor{motor_index + 1} value {value} is outside [{lower}, {upper}]"
        )
    return value


def signed_to_motor_triplet(
    signed_triplet: Iterable[float],
    *,
    arpha1_range: Optional[AlphaRange] = None,
    motor_limits: Iterable[Tuple[float, float]] = DEFAULT_MOTOR_LIMITS,
    clip: bool = True,
) -> list[float]:
    """Convert signed ``(arpha1, arpha2_star, arpha3)`` to motor targets.

    Motor 2 is calibrated with the raw angle, so this function explicitly uses
    ``arpha2_raw = -arpha2_star``.
    """

    arpha1_range = require_arpha1_range(arpha1_range)
    limits = tuple(motor_limits)
    if len(limits) != 3:
        raise ValueError("motor_limits must contain 3 (min, max) pairs")

    arpha1, arpha2_star, arpha3 = signed_to_alpha_triplet(
        signed_triplet,
        (arpha1_range, ARPHA2_STAR_RANGE, ARPHA3_RANGE),
        clip=clip,
    )
    arpha2_raw = -arpha2_star
    alphas_for_motors = (arpha1, arpha2_raw, arpha3)
    motors = [
        _apply_motor_limit(
            _motor_value(alpha, motor_index),
            limits[motor_index],
            motor_index,
            clip,
        )
        for motor_index, alpha in enumerate(alphas_for_motors)
    ]
    return [round(value, 4) for value in motors]
