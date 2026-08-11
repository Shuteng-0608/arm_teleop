"""Deterministic online quality gate for completed MuJoCo HDF5 episodes."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Sequence, Union

import h5py
import numpy as np


@dataclass
class EpisodeQualityDecision:
    """Serializable result used by automatic collection and batch manifests."""

    accepted: bool
    reasons: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    metrics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _decode_strings(values: Iterable[Any]) -> List[str]:
    decoded = []
    for value in np.ravel(values):
        if isinstance(value, bytes):
            decoded.append(value.decode("utf-8", errors="replace"))
        else:
            decoded.append(str(value))
    return decoded


def _attr_text(value: Any) -> str:
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="replace")
    return str(value)


def _check_finite_dataset(
    h5: h5py.File,
    path: str,
    reasons: List[str],
) -> None:
    if path not in h5:
        reasons.append(f"missing_dataset:{path}")
        return
    dataset = h5[path]
    if not isinstance(dataset, h5py.Dataset) or dataset.size == 0:
        reasons.append(f"empty_dataset:{path}")
        return
    if not np.all(np.isfinite(dataset[()])):
        reasons.append(f"nonfinite_dataset:{path}")


def _sampling_rate(h5: h5py.File, path: str) -> Optional[float]:
    if path not in h5:
        return None
    timestamps = np.asarray(h5[path][()], dtype=np.float64).reshape(-1)
    if timestamps.size < 2:
        return None
    intervals = np.diff(timestamps)
    if not np.all(intervals > 0.0):
        return None
    median_interval = float(np.median(intervals))
    return 1.0 / median_interval if median_interval > 0.0 else None


def evaluate_episode_quality(
    episode_path: Union[Path, str],
    *,
    execution_success: bool,
    config: Optional[Mapping[str, Any]] = None,
    expected_camera_names: Sequence[str] = (),
    online_reasons: Sequence[str] = (),
) -> EpisodeQualityDecision:
    """Evaluate one closed episode without mutating it.

    The gate deliberately uses simulation ground truth and recorder health. It
    does not attempt an opaque visual judgement. Thresholds can be calibrated
    through ``scripted_controller.auto_quality``.
    """

    cfg = dict(config or {})
    reasons = [str(reason) for reason in online_reasons if str(reason)]
    warnings: List[str] = []
    metrics: Dict[str, Any] = {}
    path = Path(episode_path).expanduser().resolve()

    if not execution_success:
        reasons.append("execution_failed")
    if not path.is_file():
        reasons.append("episode_file_missing")
        return EpisodeQualityDecision(False, reasons, warnings, metrics)

    required_numeric = cfg.get(
        "required_numeric_datasets",
        [
            "observations/joint_pos",
            "observations/joint_vel",
            "observations/joint_torque",
            "observations/ee_pose",
            "observations/ft_wrench",
            "action",
            "timestamps/state_episode",
            "timestamps/force_episode",
        ],
    )
    min_state_samples = int(cfg.get("min_state_samples", 10))
    min_force_samples = int(cfg.get("min_force_samples", 50))
    min_image_samples = int(cfg.get("min_image_samples", 3))
    max_dropped_numeric = int(cfg.get("max_dropped_numeric_samples", 0))
    max_dropped_images = int(cfg.get("max_dropped_image_samples", 0))
    image_std_min = float(cfg.get("image_std_min", 3.0))
    rate_tolerance = float(cfg.get("sampling_rate_tolerance", 0.25))

    try:
        with h5py.File(path, "r") as h5:
            for dataset_path in required_numeric:
                _check_finite_dataset(h5, str(dataset_path), reasons)

            events = (
                _decode_strings(h5["events/names"][()])
                if "events/names" in h5
                else []
            )
            metrics["events"] = events
            if bool(cfg.get("require_task_success_event", True)) and (
                "task_success_site_reached" not in events
            ):
                reasons.append("task_success_event_missing")
            if bool(cfg.get("require_terminal_hold_event", True)) and (
                "terminal_hold_start" not in events
            ):
                reasons.append("terminal_hold_event_missing")

            rejected_events = set(
                cfg.get(
                    "reject_events",
                    [
                        "joint_torque_over_limit",
                        "ft_wrench_over_limit",
                        "scripted_error",
                        "async_capture_error",
                    ],
                )
            )
            for event in sorted(rejected_events.intersection(events)):
                reasons.append(f"rejected_event:{event}")

            metadata = h5.get("episode_metadata")
            if metadata is None:
                reasons.append("episode_metadata_missing")
                attrs: Mapping[str, Any] = {}
            else:
                attrs = metadata.attrs

            async_error = _attr_text(attrs.get("async_error", "")).strip()
            dropped_numeric = int(attrs.get("dropped_numeric_samples", 0))
            dropped_images = int(attrs.get("dropped_image_samples", 0))
            metrics.update(
                {
                    "status": _attr_text(attrs.get("status", "unknown")),
                    "async_error": async_error,
                    "dropped_numeric_samples": dropped_numeric,
                    "dropped_image_samples": dropped_images,
                    "n_state": int(attrs.get("n_state", 0)),
                    "n_force": int(attrs.get("n_force", 0)),
                    "n_image": int(attrs.get("n_image", 0)),
                }
            )
            if async_error:
                reasons.append(f"recorder_async_error:{async_error}")
            if dropped_numeric > max_dropped_numeric:
                reasons.append(f"dropped_numeric_samples:{dropped_numeric}")
            if dropped_images > max_dropped_images:
                reasons.append(f"dropped_image_samples:{dropped_images}")
            if metrics["n_state"] < min_state_samples:
                reasons.append(f"too_few_state_samples:{metrics['n_state']}")
            if metrics["n_force"] < min_force_samples:
                reasons.append(f"too_few_force_samples:{metrics['n_force']}")

            record_images = bool(cfg.get("require_images", True))
            if record_images:
                if metrics["n_image"] < min_image_samples:
                    reasons.append(f"too_few_image_samples:{metrics['n_image']}")
                for camera_name in expected_camera_names:
                    camera_path = f"observations/images/{camera_name}"
                    if camera_path not in h5:
                        reasons.append(f"missing_camera:{camera_name}")
                        continue
                    images = h5[camera_path]
                    if images.shape[0] < min_image_samples:
                        reasons.append(
                            f"too_few_camera_samples:{camera_name}:{images.shape[0]}"
                        )
                        continue
                    sample_indices = sorted(
                        {0, int(images.shape[0] // 2), int(images.shape[0] - 1)}
                    )
                    std_values = [
                        float(np.std(images[index])) for index in sample_indices
                    ]
                    metrics[f"camera_std:{camera_name}"] = std_values
                    if min(std_values) < image_std_min:
                        reasons.append(f"blank_camera_frame:{camera_name}")

            expected_rates = {
                "state": (
                    "timestamps/state_episode",
                    float(cfg.get("expected_state_hz", 30.0)),
                ),
                "force": (
                    "timestamps/force_episode",
                    float(cfg.get("expected_force_hz", 500.0)),
                ),
            }
            if record_images:
                expected_rates["image"] = (
                    "timestamps/image_episode",
                    float(cfg.get("expected_image_hz", 30.0)),
                )
            for name, (timestamp_path, expected_rate) in expected_rates.items():
                actual_rate = _sampling_rate(h5, timestamp_path)
                metrics[f"{name}_hz"] = actual_rate
                if actual_rate is None:
                    warnings.append(f"invalid_{name}_timestamps")
                elif abs(actual_rate - expected_rate) > expected_rate * rate_tolerance:
                    warnings.append(
                        f"{name}_rate_out_of_range:{actual_rate:.3f}"
                    )

            # Physical-quality diagnostics are warnings by default. Set
            # reject_on_warning=true after thresholds have been calibrated to
            # turn these into strict automatic rejection criteria.
            if all(
                name in h5
                for name in (
                    "action",
                    "observations/joint_pos",
                    "observations/joint_vel",
                    "observations/ft_wrench",
                )
            ):
                action = np.asarray(h5["action"][()], dtype=np.float64)
                qpos = np.asarray(
                    h5["observations/joint_pos"][()], dtype=np.float64
                )
                qvel = np.asarray(
                    h5["observations/joint_vel"][()], dtype=np.float64
                )
                wrench = np.asarray(
                    h5["observations/ft_wrench"][()], dtype=np.float64
                )
                shared_rows = min(action.shape[0], qpos.shape[0])
                if shared_rows > 0:
                    tracking_norm = np.linalg.norm(
                        action[:shared_rows, :7] - qpos[:shared_rows, :7],
                        axis=1,
                    )
                    tracking_p95 = float(np.percentile(tracking_norm, 95))
                    tracking_max = float(np.max(tracking_norm))
                    metrics["tracking_norm_p95"] = tracking_p95
                    metrics["tracking_norm_max"] = tracking_max
                    if tracking_p95 > float(
                        cfg.get("tracking_norm_p95_warn", 0.03)
                    ):
                        warnings.append(f"tracking_p95_high:{tracking_p95:.6f}")
                    if tracking_max > float(
                        cfg.get("tracking_norm_max_warn", 0.06)
                    ):
                        warnings.append(f"tracking_max_high:{tracking_max:.6f}")

                force_norm = np.linalg.norm(wrench[:, :3], axis=1)
                force_p95 = float(np.percentile(force_norm, 95))
                force_max = float(np.max(force_norm))
                metrics["force_norm_p95"] = force_p95
                metrics["force_norm_max"] = force_max
                if force_p95 > float(cfg.get("force_p95_warn", 40.0)):
                    warnings.append(f"force_p95_high:{force_p95:.6f}")
                if force_max > float(cfg.get("force_max_warn", 60.0)):
                    warnings.append(f"force_max_high:{force_max:.6f}")

                terminal_seconds = max(
                    0.0,
                    float(cfg.get("terminal_check_seconds", 0.8)),
                )
                state_tail = max(
                    3,
                    int(round(terminal_seconds * expected_rates["state"][1])),
                )
                force_tail = max(
                    3,
                    int(round(terminal_seconds * expected_rates["force"][1])),
                )
                qvel_tail = qvel[-state_tail:, :7]
                force_tail_values = force_norm[-force_tail:]
                terminal_qvel_max = float(
                    np.max(np.linalg.norm(qvel_tail, axis=1))
                )
                terminal_force_max = float(np.max(force_tail_values))
                metrics["terminal_qvel_norm_max"] = terminal_qvel_max
                metrics["terminal_force_norm_max"] = terminal_force_max
                if terminal_qvel_max > float(
                    cfg.get("terminal_qvel_max_warn", 0.05)
                ):
                    warnings.append(
                        f"terminal_qvel_high:{terminal_qvel_max:.6f}"
                    )
                if terminal_force_max > float(
                    cfg.get("terminal_force_max_warn", 15.0)
                ):
                    warnings.append(
                        f"terminal_force_high:{terminal_force_max:.6f}"
                    )
    except (OSError, ValueError, TypeError) as exc:
        reasons.append(f"hdf5_read_error:{exc}")

    if bool(cfg.get("reject_on_warning", False)):
        reasons.extend(f"warning:{warning}" for warning in warnings)

    # Preserve order while preventing duplicate reasons from obscuring reports.
    reasons = list(dict.fromkeys(reasons))
    warnings = list(dict.fromkeys(warnings))
    return EpisodeQualityDecision(not reasons, reasons, warnings, metrics)
