"""Small dependency-free state model for retained-count collection batches."""

from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass
class BatchProgress:
    target_kept: int
    max_attempts: int
    max_consecutive_rejections: int = 0
    attempted: int = 0
    kept: int = 0
    rejected: int = 0
    consecutive_rejections: int = 0

    @property
    def complete(self) -> bool:
        return self.kept >= self.target_kept

    @property
    def exhausted(self) -> bool:
        attempts_exhausted = self.attempted >= self.max_attempts
        consecutive_exhausted = bool(
            self.max_consecutive_rejections > 0
            and self.consecutive_rejections >= self.max_consecutive_rejections
        )
        return attempts_exhausted or consecutive_exhausted

    @property
    def can_attempt(self) -> bool:
        return not self.complete and not self.exhausted

    def begin_attempt(self) -> int:
        if not self.can_attempt:
            raise RuntimeError("batch cannot start another attempt")
        self.attempted += 1
        return self.attempted

    def register(self, kept: bool) -> None:
        if kept:
            self.kept += 1
            self.consecutive_rejections = 0
        else:
            self.rejected += 1
            self.consecutive_rejections += 1

    def to_dict(self):
        return asdict(self)
