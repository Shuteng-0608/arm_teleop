from __future__ import annotations

from typing import Any, Optional, Tuple

import numpy as np
import rospy


def _get_candidate(vp_data: Any, key: str) -> Optional[Any]:
    if vp_data is None:
        return None
    if hasattr(vp_data, key):
        value = getattr(vp_data, key)
        if value is not None:
            return value
    if isinstance(vp_data, dict):
        return vp_data.get(key)
    return None


def _candidate_sources(hand: str) -> Tuple[str, ...]:
    return (
        hand,
        f"{hand}_arm",
        f"{hand}_fingers",
    )


def _transforms_to_points(transforms: Any, allow_legacy_25_fingers: bool) -> Optional[np.ndarray]:
    arr = np.asarray(transforms, dtype=float)
    if arr.ndim != 3 or arr.shape[1:] != (4, 4):
        return None
    if not np.all(np.isfinite(arr)):
        return None

    if arr.shape[0] >= 27:
        points = arr[:27, :3, 3]
    elif arr.shape[0] == 25 and allow_legacy_25_fingers:
        points_25 = arr[:, :3, 3]
        points = np.empty((27, 3), dtype=float)
        points[:25] = points_25
        points[25] = points_25[0]
        points[26] = points_25[0]
        rospy.logwarn_throttle(
            5.0,
            "MH6 Vision Pro adapter received legacy 25-joint hand data; padding joints 25 and 26 with wrist point.",
        )
    else:
        return None

    if points.shape != (27, 3):
        return None
    if not np.all(np.isfinite(points)):
        return None
    if np.allclose(points, 0.0):
        return None
    if not _passes_hand_scale_check(points):
        return None
    return points.copy()


def _passes_hand_scale_check(points: np.ndarray) -> bool:
    wrist = points[0]
    fingertip_indices = [4, 9, 14, 19, 24]
    fingertip_distances = np.linalg.norm(points[fingertip_indices] - wrist, axis=1)
    max_distance = float(np.max(fingertip_distances))
    return 0.02 <= max_distance <= 0.50


def extract_mh6_points(
    vp_data: Any,
    hand: str,
    allow_legacy_25_fingers: bool = True,
) -> Optional[np.ndarray]:
    if hand not in ("left", "right"):
        raise ValueError("hand must be 'left' or 'right'")

    for key in _candidate_sources(hand):
        candidate = _get_candidate(vp_data, key)
        if candidate is None:
            continue
        points = _transforms_to_points(candidate, allow_legacy_25_fingers)
        if points is not None:
            return points

    return None
