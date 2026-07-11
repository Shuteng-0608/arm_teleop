"""Piecewise signed normalization for MH6 palm angles.

This module deliberately contains no geometry solver or motor calibration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Tuple


@dataclass(frozen=True)
class AlphaRange:
    """Three-point calibration for one angle, in degrees."""

    minimum: float
    zero: float
    maximum: float

    def __post_init__(self) -> None:
        values = (float(self.minimum), float(self.zero), float(self.maximum))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("alpha range values must be finite")
        if not values[0] < values[1] < values[2]:
            raise ValueError("alpha range must satisfy minimum < zero < maximum")


SignedTriplet = Tuple[float, float, float]
AlphaTriplet = Tuple[float, float, float]


def _finite_float(value: float, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite number") from exc
    if not math.isfinite(result):
        raise ValueError(f"{label} must be a finite number")
    return result


def _bounded(value: float, lower: float, upper: float, label: str, clip: bool) -> float:
    if clip:
        return min(max(value, lower), upper)
    if not lower <= value <= upper:
        raise ValueError(f"{label} must be in [{lower}, {upper}], got {value}")
    return value


def alpha_to_signed_normalized(
    alpha: float,
    alpha_min: float,
    alpha_zero: float,
    alpha_max: float,
    *,
    clip: bool = True,
) -> float:
    """Map ``alpha_min/zero/max`` to ``-1/0/1`` using two linear segments."""

    alpha_range = AlphaRange(alpha_min, alpha_zero, alpha_max)
    value = _finite_float(alpha, "alpha")
    value = _bounded(
        value,
        alpha_range.minimum,
        alpha_range.maximum,
        "alpha",
        clip,
    )
    if value <= alpha_range.zero:
        return (value - alpha_range.zero) / (
            alpha_range.zero - alpha_range.minimum
        )
    return (value - alpha_range.zero) / (
        alpha_range.maximum - alpha_range.zero
    )


def signed_normalized_to_alpha(
    value: float,
    alpha_min: float,
    alpha_zero: float,
    alpha_max: float,
    *,
    clip: bool = True,
) -> float:
    """Invert :func:`alpha_to_signed_normalized`."""

    alpha_range = AlphaRange(alpha_min, alpha_zero, alpha_max)
    signed = _finite_float(value, "signed normalized value")
    signed = _bounded(signed, -1.0, 1.0, "signed normalized value", clip)
    if signed <= 0.0:
        return alpha_range.zero + signed * (
            alpha_range.zero - alpha_range.minimum
        )
    return alpha_range.zero + signed * (
        alpha_range.maximum - alpha_range.zero
    )


def alpha_triplet_to_signed(
    alpha_triplet: Iterable[float],
    ranges: Iterable[AlphaRange],
    *,
    clip: bool = True,
) -> SignedTriplet:
    """Map ``(arpha1, arpha2_star, arpha3)`` to a signed triplet."""

    values = tuple(alpha_triplet)
    alpha_ranges = tuple(ranges)
    if len(values) != 3 or len(alpha_ranges) != 3:
        raise ValueError("alpha triplet and ranges must each contain 3 values")
    return tuple(
        alpha_to_signed_normalized(
            alpha,
            alpha_range.minimum,
            alpha_range.zero,
            alpha_range.maximum,
            clip=clip,
        )
        for alpha, alpha_range in zip(values, alpha_ranges)
    )


def signed_to_alpha_triplet(
    signed_triplet: Iterable[float],
    ranges: Iterable[AlphaRange],
    *,
    clip: bool = True,
) -> AlphaTriplet:
    """Map a signed triplet to ``(arpha1, arpha2_star, arpha3)``."""

    values = tuple(signed_triplet)
    alpha_ranges = tuple(ranges)
    if len(values) != 3 or len(alpha_ranges) != 3:
        raise ValueError("signed triplet and ranges must each contain 3 values")
    return tuple(
        signed_normalized_to_alpha(
            value,
            alpha_range.minimum,
            alpha_range.zero,
            alpha_range.maximum,
            clip=clip,
        )
        for value, alpha_range in zip(values, alpha_ranges)
    )
