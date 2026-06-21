#!/usr/bin/env python3
"""
Pure MH6 hand teleoperation mapping.

This module maps 27x3 Vision Pro hand keypoints into MH6 low-dimensional hand
intentions. It intentionally contains no AVP streaming, ROS, Modbus, or hardware
control code.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

import numpy as np


FINGER_JOINTS = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8, 9),
    "middle": (10, 11, 12, 13, 14),
    "ring": (15, 16, 17, 18, 19),
    "little": (20, 21, 22, 23, 24),
}
TIP_INDICES = {
    "thumb": 4,
    "index": 9,
    "middle": 14,
    "ring": 19,
    "little": 24,
}
FINGER_NAMES = ("thumb", "index", "middle", "ring", "little")
LONG_FINGERS = ("index", "middle", "ring", "little")


@dataclass
class MappingCalibration:
    """Calibration for normalized hand intention mapping.

    Finger normalized values use 0 as fully open/extended and 1 as fully
    curled/closed. Palm normalized values follow the same open-to-flexed
    convention, except lateral palm command u_v uses -1..1.
    """

    curl_open: Dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 0.0,
            "index": 0.0,
            "middle": 0.0,
            "ring": 0.0,
            "little": 0.0,
        }
    )
    curl_closed: Dict[str, float] = field(
        default_factory=lambda: {
            "thumb": 1.40,
            "index": 2.40,
            "middle": 2.60,
            "ring": 2.60,
            "little": 2.40,
        }
    )
    opposition_open_dist: Dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.090,
            "middle": 0.105,
            "ring": 0.120,
            "little": 0.135,
        }
    )
    opposition_closed_dist: Dict[str, float] = field(
        default_factory=lambda: {
            "index": 0.018,
            "middle": 0.022,
            "ring": 0.026,
            "little": 0.030,
        }
    )
    opposition_threshold: float = 0.35


def validate_points(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.shape != (27, 3):
        raise ValueError(f"points must have shape (27, 3), got {arr.shape}")
    if not np.all(np.isfinite(arr)):
        raise ValueError("points must contain only finite values")
    return arr


def clip(x: float, lo: float, hi: float) -> float:
    return float(min(max(float(x), lo), hi))


def angle_between(v1: np.ndarray, v2: np.ndarray) -> float:
    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))
    if n1 <= 1e-12 or n2 <= 1e-12:
        return 0.0
    cos_theta = clip(float(np.dot(v1, v2)) / (n1 * n2), -1.0, 1.0)
    return float(np.arccos(cos_theta))


def finger_curl(points: np.ndarray, joint_indices: Iterable[int]) -> float:
    joint_points = points[list(joint_indices)]
    if len(joint_points) < 3:
        return 0.0
    vectors = np.diff(joint_points, axis=0)
    return float(
        sum(angle_between(vectors[i], vectors[i + 1]) for i in range(len(vectors) - 1))
    )


def distance(a: np.ndarray, b: np.ndarray) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def normalize(value: float, open_value: float, closed_value: float) -> float:
    denom = float(closed_value) - float(open_value)
    if abs(denom) <= 1e-12:
        raise ValueError("open and closed calibration values must not be equal")
    return clip((float(value) - float(open_value)) / denom, 0.0, 1.0)


def threshold_strength(value: float, threshold: float) -> float:
    threshold = clip(threshold, 0.0, 0.999999)
    value = clip(value, 0.0, 1.0)
    if value <= threshold:
        return 0.0
    return clip((value - threshold) / (1.0 - threshold), 0.0, 1.0)


class MH6HandMapper:
    def __init__(self, calibration: Optional[MappingCalibration] = None) -> None:
        self.calibration = calibration if calibration is not None else MappingCalibration()

    def calibrate_open(self, samples: List[np.ndarray]) -> None:
        if not samples:
            raise ValueError("calibrate_open requires at least one sample")

        curls = {finger: [] for finger in FINGER_NAMES}
        opposition_distances = {finger: [] for finger in LONG_FINGERS}
        for sample in samples:
            points = validate_points(sample)
            raw_curls = self.compute_finger_curls(points)
            for finger in FINGER_NAMES:
                curls[finger].append(raw_curls[finger])

            thumb_tip = points[TIP_INDICES["thumb"]]
            for finger in LONG_FINGERS:
                opposition_distances[finger].append(distance(thumb_tip, points[TIP_INDICES[finger]]))

        self.calibration.curl_open = {
            finger: float(np.mean(values)) for finger, values in curls.items()
        }
        self.calibration.opposition_open_dist = {
            finger: float(np.mean(values)) for finger, values in opposition_distances.items()
        }

    def compute_finger_curls(self, points: np.ndarray) -> Dict[str, float]:
        points = validate_points(points)
        return {
            finger: finger_curl(points, FINGER_JOINTS[finger])
            for finger in FINGER_NAMES
        }

    def compute_normalized_curls(self, points: np.ndarray) -> Dict[str, float]:
        raw = self.compute_finger_curls(points)
        return {
            finger: normalize(
                raw[finger],
                self.calibration.curl_open[finger],
                self.calibration.curl_closed[finger],
            )
            for finger in FINGER_NAMES
        }

    def compute_opposition(self, points: np.ndarray) -> Dict[str, float]:
        points = validate_points(points)
        thumb_tip = points[TIP_INDICES["thumb"]]
        opposition = {}
        for finger, key in (
            ("index", "p_I"),
            ("middle", "p_M"),
            ("ring", "p_R"),
            ("little", "p_L"),
        ):
            raw_strength = normalize(
                distance(thumb_tip, points[TIP_INDICES[finger]]),
                self.calibration.opposition_open_dist[finger],
                self.calibration.opposition_closed_dist[finger],
            )
            opposition[key] = threshold_strength(raw_strength, self.calibration.opposition_threshold)
        return opposition

    def compute_low_dim(self, points: np.ndarray) -> Dict[str, float]:
        """Return the 7D normalized command.

        Fingers are 0=open/extended and 1=curled/closed. u_h is 0=open/flat
        palm and 1=maximum palm enclosure/flexion. u_v is -1=index/middle side,
        0=neutral, and +1=ring/little side.
        """

        return self.step(points)["low_dim"]

    def step(self, points: np.ndarray) -> Dict[str, Dict[str, float]]:
        """Return debug-friendly mapping outputs with normalized conventions.

        low_dim finger values are 0=open and 1=closed. Palm block values are
        0=open and 1=maximum corresponding block flexion.
        """

        points = validate_points(points)
        curl_raw = self.compute_finger_curls(points)
        curl_norm = self.compute_normalized_curls(points)
        opposition = self.compute_opposition(points)

        p_i = opposition["p_I"]
        p_m = opposition["p_M"]
        p_r = opposition["p_R"]
        p_l = opposition["p_L"]
        p_opp = max(p_i, p_m, p_r, p_l)

        u_thumb = clip(max(0.7 * curl_norm["thumb"], p_opp), 0.0, 1.0)
        u_index = clip(max(curl_norm["index"], p_i), 0.0, 1.0)
        u_middle = clip(max(curl_norm["middle"], p_m), 0.0, 1.0)
        u_ring = clip(max(curl_norm["ring"], p_r), 0.0, 1.0)
        u_little = clip(max(curl_norm["little"], p_l), 0.0, 1.0)

        g = 0.2 * u_index + 0.3 * u_middle + 0.3 * u_ring + 0.2 * u_little
        t = min(u_thumb, u_index, u_middle)
        o_h = clip(0.20 * p_i + 0.35 * p_m + 0.70 * p_r + 1.00 * p_l, 0.0, 1.0)
        u_h = clip(0.55 * g + 0.20 * t + 0.35 * o_h, 0.0, 1.0)

        b_f = 0.5 * (u_ring + u_little) - 0.5 * (u_index + u_middle)
        o_v = clip(-0.25 * p_i - 0.45 * p_m + 0.75 * p_r + 1.00 * p_l, -1.0, 1.0)
        u_v = clip(0.30 * b_f + 0.70 * o_v, -1.0, 1.0)

        # Palm block expansion:
        # - u_h: 0=open/flat, 1=maximum enclosure/flexion
        # - u_v: -1=index/middle side, 0=neutral, +1=ring/little side
        # - block outputs: 0=open, 1=maximum corresponding block flexion
        thumb_side = clip(u_h - u_v, 0.0, 1.0)
        little_side = clip(u_h + u_v, 0.0, 1.0)

        return {
            "curl_raw": curl_raw,
            "curl_norm": curl_norm,
            "opposition": opposition,
            "intent": {
                "P_opp": p_opp,
                "g": g,
                "t": t,
                "o_h": o_h,
                "b_f": b_f,
                "o_v": o_v,
            },
            "low_dim": {
                "u_thumb": u_thumb,
                "u_index": u_index,
                "u_middle": u_middle,
                "u_ring": u_ring,
                "u_little": u_little,
                "u_h": u_h,
                "u_v": u_v,
            },
            "palm": {
                "thumbSide": thumb_side,
                "littleSide": little_side,
                "UL": thumb_side,
                "UR": little_side,
                "LL": thumb_side,
                "LR": little_side,
            },
        }
