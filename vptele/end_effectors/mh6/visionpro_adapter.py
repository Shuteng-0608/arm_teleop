"""VisionProTeleop hand-data adapter for MH6 dry-run mapping."""

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import rospy
except ImportError:  # Allows local static helper validation without ROS installed.
    rospy = None


@dataclass
class CanonicalHandPoints:
    points: np.ndarray
    source_name: str
    source_type: str
    original_joint_count: int
    padded_forearm: bool = False


def _warn_throttle(period, message, *args):
    if rospy is not None:
        rospy.logwarn_throttle(period, message, *args)


def _get_source(vp_data, name):
    if vp_data is None:
        return None

    if hasattr(vp_data, name):
        value = getattr(vp_data, name)
        if value is not None:
            return value

    if isinstance(vp_data, dict):
        return vp_data.get(name)

    try:
        return vp_data[name]
    except (KeyError, TypeError, IndexError):
        return None


def _source_type(source_name):
    if source_name in ("left", "right"):
        return "attribute_27"
    if source_name.endswith("_arm"):
        return "dict_arm_27"
    return "dict_fingers_25"


def _transforms_to_canonical(transforms, source_name):
    try:
        transforms = np.asarray(transforms, dtype=float)
    except (TypeError, ValueError):
        _warn_throttle(5.0, "MH6 adapter: %s cannot be converted to float array", source_name)
        return None

    if transforms.ndim != 3 or transforms.shape[1:] != (4, 4):
        _warn_throttle(
            5.0,
            "MH6 adapter: %s has invalid shape %s, expected Nx4x4",
            source_name,
            transforms.shape,
        )
        return None

    if not np.all(np.isfinite(transforms)):
        _warn_throttle(5.0, "MH6 adapter: %s contains non-finite values", source_name)
        return None

    joint_count = transforms.shape[0]
    if joint_count >= 27:
        return CanonicalHandPoints(
            points=transforms[:27, :3, 3].copy(),
            source_name=source_name,
            source_type=_source_type(source_name),
            original_joint_count=joint_count,
            padded_forearm=False,
        )

    if joint_count == 25:
        points_25 = transforms[:, :3, 3]
        points_27 = np.zeros((27, 3), dtype=float)
        points_27[:25] = points_25
        points_27[25] = points_25[0]
        points_27[26] = points_25[0]
        _warn_throttle(
            10.0,
            "MH6 adapter: %s uses legacy 25-joint fingers; padded forearm joints "
            "25 and 26 with wrist point",
            source_name,
        )
        return CanonicalHandPoints(
            points=points_27,
            source_name=source_name,
            source_type="dict_fingers_25_padded",
            original_joint_count=joint_count,
            padded_forearm=True,
        )

    _warn_throttle(
        5.0,
        "MH6 adapter: %s has unsupported joint count %d, expected 25 or at least 27",
        source_name,
        joint_count,
    )
    return None


def extract_canonical_hand_data(
    vp_data,
    hand: str,
    prefer_full_skeleton: bool = True,
    allow_legacy_25_fingers: bool = True,
) -> Optional[CanonicalHandPoints]:
    """Return canonical MH6 hand data with points shape (27, 3), or None.

    Preference order follows VisionProTeleop:
    attribute-style data.left/right, legacy dict left_arm/right_arm, then
    legacy 25-joint left_fingers/right_fingers when allowed.
    """
    hand = str(hand).strip().lower()
    if hand not in ("left", "right"):
        raise ValueError("hand must be 'left' or 'right'")

    source_names = []
    if prefer_full_skeleton:
        source_names.extend([hand, f"{hand}_arm"])
    else:
        source_names.append(f"{hand}_arm")

    if allow_legacy_25_fingers:
        source_names.append(f"{hand}_fingers")

    for source_name in source_names:
        source = _get_source(vp_data, source_name)
        if source is None:
            continue
        canonical = _transforms_to_canonical(source, source_name)
        if canonical is None:
            continue
        if canonical.points.shape != (27, 3) or not np.all(np.isfinite(canonical.points)):
            _warn_throttle(
                5.0,
                "MH6 adapter: %s produced invalid canonical points shape %s",
                source_name,
                canonical.points.shape,
            )
            continue
        return canonical

    _warn_throttle(
        5.0,
        "MH6 adapter: no valid %s hand source found; checked %s",
        hand,
        ", ".join(source_names),
    )
    return None


def extract_canonical_hand_points(
    vp_data,
    hand: str,
    prefer_full_skeleton: bool = True,
    allow_legacy_25_fingers: bool = True,
) -> Optional[np.ndarray]:
    canonical = extract_canonical_hand_data(
        vp_data,
        hand,
        prefer_full_skeleton=prefer_full_skeleton,
        allow_legacy_25_fingers=allow_legacy_25_fingers,
    )
    return None if canonical is None else canonical.points


def _make_transforms(count):
    transforms = np.zeros((count, 4, 4), dtype=float)
    transforms[:, 3, 3] = 1.0
    for idx in range(count):
        transforms[idx, :3, :3] = np.eye(3)
        transforms[idx, :3, 3] = [idx, idx + 0.1, idx + 0.2]
    return transforms


def validate_adapter_with_synthetic_arrays():
    """Small ROS-free validation helper for the adapter contract."""
    points_27 = extract_canonical_hand_points({"right_arm": _make_transforms(27)}, "right")
    points_25 = extract_canonical_hand_points({"right_fingers": _make_transforms(25)}, "right")
    invalid = extract_canonical_hand_points({"right_arm": np.zeros((24, 4, 4))}, "right")

    return {
        "full_27_shape": None if points_27 is None else points_27.shape,
        "legacy_25_shape": None if points_25 is None else points_25.shape,
        "legacy_padding_matches_wrist": bool(
            points_25 is not None
            and np.allclose(points_25[25], points_25[0])
            and np.allclose(points_25[26], points_25[0])
        ),
        "invalid_returns_none": invalid is None,
    }
