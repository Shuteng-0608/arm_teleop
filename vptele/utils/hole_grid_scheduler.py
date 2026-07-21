#!/usr/bin/env python3
"""Deterministic, coverage-balanced scheduler for peg-in-hole positions."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Optional, Sequence, Tuple

import numpy as np


class HoleGridScheduler:
    """Visit every X-Z grid cell exactly once per cycle.

    Rows are numbered from high Z to low Z and columns from low X to high X,
    matching the operator-facing R1C1 ... RnCm diagram. Public metadata uses
    one-based indices; ``start_cycle`` and ``start_index`` are zero-based
    configuration values so a stopped collection can be resumed precisely.
    """

    VALID_SAMPLE_MODES = {"center", "uniform_in_cell"}
    VALID_TRAVERSAL_ORDERS = {"row_major", "shuffled"}
    VALID_ADVANCE_POLICIES = {"on_keep", "on_attempt"}

    def __init__(
        self,
        rows: int = 5,
        cols: int = 5,
        x_range: Sequence[float] = (-0.06, 0.06),
        y_offset: float = 0.0,
        z_range: Sequence[float] = (-0.06, 0.06),
        sample_mode: str = "center",
        traversal_order: str = "shuffled",
        seed: Optional[int] = 42,
        start_cycle: int = 0,
        start_index: int = 0,
    ) -> None:
        self.rows = self._positive_int(rows, "rows")
        self.cols = self._positive_int(cols, "cols")
        self.x_range = self._range_pair(x_range, "x_range")
        self.z_range = self._range_pair(z_range, "z_range")
        self.y_offset = float(y_offset)

        self.sample_mode = str(sample_mode).strip().lower()
        if self.sample_mode not in self.VALID_SAMPLE_MODES:
            raise ValueError(
                f"sample_mode must be one of {sorted(self.VALID_SAMPLE_MODES)}, "
                f"got {sample_mode!r}"
            )

        self.traversal_order = str(traversal_order).strip().lower()
        if self.traversal_order not in self.VALID_TRAVERSAL_ORDERS:
            raise ValueError(
                "traversal_order must be one of "
                f"{sorted(self.VALID_TRAVERSAL_ORDERS)}, got {traversal_order!r}"
            )

        if seed is None:
            entropy = np.random.SeedSequence().generate_state(1, dtype=np.uint32)[0]
            self.seed = int(entropy)
            self.seed_was_generated = True
        else:
            self.seed = int(seed)
            if self.seed < 0:
                raise ValueError("seed must be non-negative or None")
            self.seed_was_generated = False

        self.cycle_index = self._nonnegative_int(start_cycle, "start_cycle")
        self.sequence_index = self._nonnegative_int(start_index, "start_index")
        if self.sequence_index >= self.total_cells:
            raise ValueError(
                f"start_index must be in [0, {self.total_cells - 1}], "
                f"got {self.sequence_index}"
            )

        self._cells = self._build_cells()
        self._order = self._order_for_cycle(self.cycle_index)
        self._current_sample = self._make_current_sample()

    @property
    def total_cells(self) -> int:
        return self.rows * self.cols

    def current(self) -> Dict[str, Any]:
        """Return an isolated copy of the currently assigned cell sample."""
        return deepcopy(self._current_sample)

    def advance(self) -> Dict[str, Any]:
        """Advance after a kept episode and return the newly assigned sample."""
        self.sequence_index += 1
        if self.sequence_index >= self.total_cells:
            self.cycle_index += 1
            self.sequence_index = 0
            self._order = self._order_for_cycle(self.cycle_index)

        self._current_sample = self._make_current_sample()
        return self.current()

    def complete_episode(
        self,
        keep: bool,
        advance_policy: str = "on_keep",
    ) -> Dict[str, Any]:
        """Apply the collection policy after an episode is reviewed."""
        policy = str(advance_policy).strip().lower()
        if policy not in self.VALID_ADVANCE_POLICIES:
            raise ValueError(
                f"advance_policy must be one of {sorted(self.VALID_ADVANCE_POLICIES)}"
            )
        if bool(keep) or policy == "on_attempt":
            return self.advance()
        return self.current()

    def state_dict(self) -> Dict[str, Any]:
        """Return restart information for logs or a future persistent checkpoint."""
        return {
            "cycle_index": self.cycle_index,
            "sequence_index": self.sequence_index,
            "seed": self.seed,
            "seed_was_generated": self.seed_was_generated,
            "order": [int(i) for i in self._order],
        }

    def _build_cells(self):
        x_edges = np.linspace(self.x_range[0], self.x_range[1], self.cols + 1)
        z_edges = np.linspace(self.z_range[0], self.z_range[1], self.rows + 1)
        cells = []

        for row in range(self.rows):
            z_low = float(z_edges[self.rows - row - 1])
            z_high = float(z_edges[self.rows - row])
            for col in range(self.cols):
                x_low = float(x_edges[col])
                x_high = float(x_edges[col + 1])
                cells.append(
                    {
                        "hole_grid_row": row + 1,
                        "hole_grid_col": col + 1,
                        "hole_grid_cell_linear_index": row * self.cols + col + 1,
                        "hole_grid_cell_label": f"R{row + 1}C{col + 1}",
                        "hole_cell_x_bounds": [x_low, x_high],
                        "hole_cell_z_bounds": [z_low, z_high],
                        "hole_cell_center_offset": [
                            self._clean_zero(0.5 * (x_low + x_high)),
                            self.y_offset,
                            self._clean_zero(0.5 * (z_low + z_high)),
                        ],
                    }
                )

        return cells

    def _order_for_cycle(self, cycle_index: int) -> np.ndarray:
        order = np.arange(self.total_cells, dtype=int)
        if self.traversal_order == "shuffled":
            rng = self._rng(cycle_index, 0, 0)
            rng.shuffle(order)
        return order

    def _make_current_sample(self) -> Dict[str, Any]:
        cell_array_index = int(self._order[self.sequence_index])
        cell = deepcopy(self._cells[cell_array_index])
        center = cell["hole_cell_center_offset"]

        if self.sample_mode == "uniform_in_cell":
            rng = self._rng(self.cycle_index, cell_array_index, 1)
            x = float(rng.uniform(*cell["hole_cell_x_bounds"]))
            z = float(rng.uniform(*cell["hole_cell_z_bounds"]))
        else:
            x = float(center[0])
            z = float(center[2])

        cell.update(
            {
                "hole_sampling_mode": "grid",
                "hole_grid_rows": self.rows,
                "hole_grid_cols": self.cols,
                "hole_grid_cycle": self.cycle_index + 1,
                "hole_grid_index": self.sequence_index + 1,
                "hole_grid_seed": self.seed,
                "hole_grid_seed_was_generated": self.seed_was_generated,
                "hole_grid_traversal_order": self.traversal_order,
                "hole_grid_sample_mode": self.sample_mode,
                "hole_actual_offset_xyz": [x, self.y_offset, z],
            }
        )
        return cell

    def _rng(self, cycle_index: int, cell_index: int, stream: int):
        seed_sequence = np.random.SeedSequence(
            [self.seed, int(cycle_index), int(cell_index), int(stream)]
        )
        return np.random.default_rng(seed_sequence)

    @staticmethod
    def _positive_int(value: int, name: str) -> int:
        parsed = int(value)
        if parsed <= 0:
            raise ValueError(f"{name} must be positive, got {value!r}")
        return parsed

    @staticmethod
    def _nonnegative_int(value: int, name: str) -> int:
        parsed = int(value)
        if parsed < 0:
            raise ValueError(f"{name} must be non-negative, got {value!r}")
        return parsed

    @staticmethod
    def _range_pair(value: Sequence[float], name: str) -> Tuple[float, float]:
        if len(value) != 2:
            raise ValueError(f"{name} must contain exactly two values")
        low, high = float(value[0]), float(value[1])
        if not np.isfinite(low) or not np.isfinite(high) or low >= high:
            raise ValueError(f"{name} must satisfy finite low < high, got {value!r}")
        return low, high

    @staticmethod
    def _clean_zero(value: float) -> float:
        return 0.0 if abs(value) < 1e-15 else float(value)
