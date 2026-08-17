"""Deterministic depth/direction/amplitude coverage for in-hole recovery."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List

import numpy as np


@dataclass(frozen=True)
class InHoleDisturbanceSample:
    cycle: int
    index: int
    depth_index: int
    direction_index: int
    amplitude_index: int
    depth_fraction: float
    direction_deg: float
    amplitude_mm: float

    def to_dict(self) -> Dict[str, object]:
        return {
            "scripted_in_hole_disturbance_cycle": int(self.cycle),
            "scripted_in_hole_disturbance_index": int(self.index),
            "scripted_in_hole_depth_index": int(self.depth_index),
            "scripted_in_hole_direction_index": int(self.direction_index),
            "scripted_in_hole_amplitude_index": int(self.amplitude_index),
            "scripted_in_hole_depth_fraction": float(self.depth_fraction),
            "scripted_in_hole_direction_deg": float(self.direction_deg),
            "scripted_in_hole_amplitude_mm": float(self.amplitude_mm),
            "scripted_in_hole_cell_label": (
                f"D{self.depth_index + 1}"
                f"A{self.direction_index + 1}"
                f"M{self.amplitude_index + 1}"
            ),
        }


class InHoleDisturbanceScheduler:
    """Visit each depth/direction/amplitude cell once per cycle."""

    def __init__(
        self,
        depth_fractions: Iterable[float],
        direction_bins: int,
        amplitudes_mm: Iterable[float],
        *,
        order: str = "shuffled",
        seed: int = 0,
        start_cycle: int = 0,
        start_index: int = 0,
    ) -> None:
        self.depth_fractions = tuple(float(value) for value in depth_fractions)
        self.direction_bins = int(direction_bins)
        self.amplitudes_mm = tuple(float(value) for value in amplitudes_mm)
        self.order = str(order).lower()
        self.seed = int(seed)
        self.cycle = int(start_cycle)
        self.index = int(start_index)

        if not self.depth_fractions or any(
            not np.isfinite(value) or not 0.0 < value < 1.0
            for value in self.depth_fractions
        ):
            raise ValueError("depth_fractions must contain finite values in (0, 1)")
        if len(set(self.depth_fractions)) != len(self.depth_fractions):
            raise ValueError("depth_fractions must not contain duplicates")
        if not self.amplitudes_mm or any(
            not np.isfinite(value) or value <= 0.0
            for value in self.amplitudes_mm
        ):
            raise ValueError("amplitudes_mm must contain positive finite values")
        if len(set(self.amplitudes_mm)) != len(self.amplitudes_mm):
            raise ValueError("amplitudes_mm must not contain duplicates")
        if self.direction_bins <= 0:
            raise ValueError("direction_bins must be positive")
        if self.order not in {"row_major", "shuffled"}:
            raise ValueError("order must be row_major or shuffled")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if self.cycle < 0 or not 0 <= self.index < self.size:
            raise ValueError("invalid disturbance start cursor")

    @property
    def size(self) -> int:
        return (
            len(self.depth_fractions)
            * self.direction_bins
            * len(self.amplitudes_mm)
        )

    def _order_for_cycle(self, cycle: int) -> List[int]:
        order = list(range(self.size))
        if self.order == "shuffled":
            np.random.default_rng(self.seed + int(cycle)).shuffle(order)
        return order

    def current(self) -> InHoleDisturbanceSample:
        flat = self._order_for_cycle(self.cycle)[self.index]
        cells_per_depth = self.direction_bins * len(self.amplitudes_mm)
        depth_index = flat // cells_per_depth
        remainder = flat % cells_per_depth
        direction_index = remainder // len(self.amplitudes_mm)
        amplitude_index = remainder % len(self.amplitudes_mm)
        return InHoleDisturbanceSample(
            cycle=self.cycle,
            index=self.index,
            depth_index=depth_index,
            direction_index=direction_index,
            amplitude_index=amplitude_index,
            depth_fraction=self.depth_fractions[depth_index],
            direction_deg=360.0 * direction_index / self.direction_bins,
            amplitude_mm=self.amplitudes_mm[amplitude_index],
        )

    def advance(self) -> InHoleDisturbanceSample:
        self.index += 1
        if self.index >= self.size:
            self.index = 0
            self.cycle += 1
        return self.current()

    def take(self) -> InHoleDisturbanceSample:
        sample = self.current()
        self.advance()
        return sample
