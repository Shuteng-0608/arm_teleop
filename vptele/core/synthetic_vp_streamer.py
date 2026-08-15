"""Deterministic Vision Pro-compatible tracking source for offline tests."""

from __future__ import annotations

import time

import numpy as np


class SyntheticVPStreamer:
    """Generate frames with the legacy shapes documented by avp-stream."""

    def __init__(self, *, amplitude_m: float = 0.015, period_s: float = 6.0):
        self.amplitude_m = float(amplitude_m)
        self.period_s = max(float(period_s), 0.1)
        self.started_at = time.monotonic()
        self._closed = False

    def get_latest(self):
        if self._closed:
            return None
        phase = 2.0 * np.pi * (time.monotonic() - self.started_at) / self.period_s
        wrist = np.eye(4, dtype=np.float64)
        wrist[:3, 3] = self.amplitude_m * np.asarray(
            [0.35 * np.sin(phase * 0.5), np.sin(phase), 0.5 * np.cos(phase)]
        )
        head = np.eye(4, dtype=np.float64)[np.newaxis]
        fingers = np.repeat(np.eye(4, dtype=np.float64)[np.newaxis], 25, axis=0)
        arm = np.repeat(np.eye(4, dtype=np.float64)[np.newaxis], 27, axis=0)
        return {
            "head": head,
            "right_wrist": wrist[np.newaxis],
            "right_fingers": fingers,
            "right_arm": arm,
            "right_pinch_distance": 0.04,
        }

    @property
    def latest(self):
        return self.get_latest()

    def get_hand_position(self, hand="right"):
        return self.get_latest().get(f"{hand}_wrist")

    def get_fingers_data(self, hand="right"):
        return self.get_latest().get(f"{hand}_fingers")

    def get_head_data(self):
        return self.get_latest().get("head")

    def start_video_stream(self, *args, **kwargs):
        return False

    def update_video_frame(self, _frame_bgr):
        return False

    def close(self):
        self._closed = True
