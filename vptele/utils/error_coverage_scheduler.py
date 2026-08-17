"""Deterministic radial/angular coverage for scripted rim contacts."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List

import numpy as np


@dataclass(frozen=True)
class RimContactSample:
    cycle: int
    index: int
    radius_index: int
    angle_index: int
    radius_mm: float
    nominal_angle_deg: float
    angle_deg: float

    def to_dict(self) -> Dict[str, object]:
        return {
            "scripted_error_coverage_mode": "stratified_radius_angle",
            "scripted_error_cycle": int(self.cycle),
            "scripted_error_index": int(self.index),
            "scripted_error_radius_index": int(self.radius_index),
            "scripted_error_angle_index": int(self.angle_index),
            "scripted_error_radius_mm": float(self.radius_mm),
            "scripted_error_nominal_angle_deg": float(self.nominal_angle_deg),
            "scripted_error_angle_deg": float(self.angle_deg),
            "scripted_error_cell_label": (
                f"R{self.radius_index + 1}A{self.angle_index + 1}"
            ),
        }


class RimContactCoverageScheduler:
    """Visit every configured radius/angle bin once per deterministic cycle."""

    def __init__(
        self,
        radii_mm: Iterable[float],
        angle_bins: int,
        *,
        order: str = "shuffled",
        seed: int = 0,
        angle_jitter_deg: float = 0.0,
        start_cycle: int = 0,
        start_index: int = 0,
    ) -> None:
        self.radii_mm = tuple(float(value) for value in radii_mm)
        self.angle_bins = int(angle_bins)
        self.order = str(order).lower()
        self.seed = int(seed)
        self.angle_jitter_deg = float(angle_jitter_deg)
        self.cycle = int(start_cycle)
        self.index = int(start_index)
        if not self.radii_mm or any(
            not np.isfinite(value) or value <= 0.0 for value in self.radii_mm
        ):
            raise ValueError("radii_mm must contain positive finite values")
        if len(set(self.radii_mm)) != len(self.radii_mm):
            raise ValueError("radii_mm must not contain duplicates")
        if self.angle_bins <= 0:
            raise ValueError("angle_bins must be positive")
        if self.order not in {"row_major", "shuffled"}:
            raise ValueError("order must be row_major or shuffled")
        if self.angle_jitter_deg < 0.0:
            raise ValueError("angle_jitter_deg must be non-negative")
        if self.angle_jitter_deg > 180.0 / self.angle_bins:
            raise ValueError("angle_jitter_deg must not exceed half an angle bin")
        if self.seed < 0:
            raise ValueError("seed must be non-negative")
        if self.cycle < 0 or not 0 <= self.index < self.size:
            raise ValueError("invalid coverage start cursor")

    @property
    def size(self) -> int:
        return len(self.radii_mm) * self.angle_bins

    def _order_for_cycle(self, cycle: int) -> List[int]:
        order = list(range(self.size))
        if self.order == "shuffled":
            np.random.default_rng(self.seed + int(cycle)).shuffle(order)
        return order

    def current(self) -> RimContactSample:
        flat = self._order_for_cycle(self.cycle)[self.index]
        radius_index = flat // self.angle_bins
        angle_index = flat % self.angle_bins
        nominal = 360.0 * angle_index / self.angle_bins
        jitter = 0.0
        if self.angle_jitter_deg > 0.0:
            # A separate deterministic stream keeps jitter reproducible even
            # when traversal order changes.
            jitter_seed = (
                self.seed * 1_000_003
                + self.cycle * self.size
                + flat
                + 17
            )
            jitter = float(
                np.random.default_rng(jitter_seed).uniform(
                    -self.angle_jitter_deg, self.angle_jitter_deg
                )
            )
        angle = (nominal + jitter) % 360.0
        return RimContactSample(
            cycle=self.cycle,
            index=self.index,
            radius_index=radius_index,
            angle_index=angle_index,
            radius_mm=self.radii_mm[radius_index],
            nominal_angle_deg=nominal,
            angle_deg=angle,
        )

    def advance(self) -> RimContactSample:
        self.index += 1
        if self.index >= self.size:
            self.index = 0
            self.cycle += 1
        return self.current()

    def take(self) -> RimContactSample:
        sample = self.current()
        self.advance()
        return sample
