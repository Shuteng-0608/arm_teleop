"""Loading, normalizing, and validating the MuJoCo teleoperation config."""

from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence

import yaml


class MujocoConfigError(ValueError):
    """Raised when a MuJoCo teleoperation config is invalid."""


class UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(loader, node, deep=False):
    mapping = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in mapping:
            raise MujocoConfigError(
                f"Duplicate YAML key {key!r} at line {key_node.start_mark.line + 1}"
            )
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _resolve_path(value: Any, config_dir: Path) -> str:
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = config_dir / path
    return str(path.resolve())


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MujocoConfigError(f"{name} must be a mapping")
    return value


def _require_finite_number(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise MujocoConfigError(f"{name} must be a number") from exc
    if number != number or number in (float("inf"), float("-inf")):
        raise MujocoConfigError(f"{name} must be finite")
    return number


def _require_integer(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise MujocoConfigError(f"{name} must be an integer")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise MujocoConfigError(f"{name} must be an integer") from exc
    if isinstance(value, float) and not value.is_integer():
        raise MujocoConfigError(f"{name} must be an integer")
    return number


def _require_positive(value: Any, name: str, *, allow_zero: bool = False) -> float:
    number = _require_finite_number(value, name)
    invalid = number < 0.0 if allow_zero else number <= 0.0
    if invalid:
        operator = "non-negative" if allow_zero else "positive"
        raise MujocoConfigError(f"{name} must be {operator}")
    return number


def _require_vector(value: Any, name: str, length: int) -> Sequence[Any]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise MujocoConfigError(f"{name} must contain exactly {length} values")
    for index, item in enumerate(value):
        _require_finite_number(item, f"{name}[{index}]")
    return value


def _require_range(value: Any, name: str) -> Sequence[Any]:
    pair = _require_vector(value, name, 2)
    if float(pair[0]) >= float(pair[1]):
        raise MujocoConfigError(f"{name} must be ordered as [min, max]")
    return pair


def load_mujoco_config(config_path: str) -> Dict[str, Any]:
    """Load a config with strict duplicate detection and normalized paths."""
    path = Path(config_path).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = yaml.load(config_file, Loader=UniqueKeySafeLoader)
    except MujocoConfigError:
        raise
    except Exception as exc:
        raise MujocoConfigError(f"Unable to load config {path}: {exc}") from exc

    if not isinstance(config, dict):
        raise MujocoConfigError(f"Config root must be a mapping: {path}")

    normalized = deepcopy(config)
    for key in ("mujoco_model_path", "record_dir", "hdf5_record_dir"):
        if normalized.get(key):
            normalized[key] = _resolve_path(normalized[key], path.parent)

    normalized["config_path"] = str(path)
    return normalized


def apply_runtime_overrides(
    config: Mapping[str, Any],
    *,
    vp_ip: Optional[str] = None,
    end_effector: Optional[str] = None,
    process_name: Optional[str] = None,
) -> Dict[str, Any]:
    """Apply ROS/CLI overrides without mutating the loaded YAML mapping."""
    result = deepcopy(dict(config))
    if vp_ip is not None:
        result["vp_ip"] = vp_ip
    if end_effector is not None:
        result["end_effector"] = str(end_effector).strip().lower()
    if process_name:
        logging_config = dict(result.get("logging") or {})
        logging_config["log_prefix"] = str(process_name)
        result["logging"] = logging_config
    return result


def validate_mujoco_config(config: Mapping[str, Any]) -> None:
    """Validate settings whose failure would otherwise be late or silent."""
    _require_mapping(config, "config")
    config_version = _require_integer(config.get("config_version", 1), "config_version")
    if config_version != 1:
        raise MujocoConfigError(
            f"Unsupported config_version: {config.get('config_version')!r}"
        )
    if not str(config.get("vp_ip", "")).strip():
        raise MujocoConfigError("vp_ip is required")

    model_path = str(config.get("mujoco_model_path", "")).strip()
    if not model_path:
        raise MujocoConfigError("mujoco_model_path is required")
    if not Path(model_path).is_file():
        raise MujocoConfigError(f"MuJoCo model does not exist: {model_path}")

    default_end_effector = "hand" if config.get("enable_hand", False) else "peg"
    end_effector = str(
        config.get("end_effector", default_end_effector)
    ).strip().lower()
    if end_effector not in {"peg", "hand", "none"}:
        raise MujocoConfigError(
            "end_effector must be one of: peg, hand, none"
        )

    arm_config = _require_mapping(config.get("arm_config", {}), "arm_config")
    service_ns = str(
        arm_config.get("teleop_service_ns", "/arm_teleop_mujoco")
    ).rstrip("/")
    if not service_ns.startswith("/"):
        raise MujocoConfigError("arm_config.teleop_service_ns must be absolute")
    if config.get("teleop_controlled_by_recording", True) and not arm_config.get(
        "enable_episode_services", True
    ):
        raise MujocoConfigError(
            "teleop_controlled_by_recording requires arm_config.enable_episode_services"
        )
    if not config.get("teleop_controlled_by_recording", True) and not config.get(
        "accept_teleop_when_not_recording", False
    ):
        raise MujocoConfigError(
            "teleop_controlled_by_recording=false requires "
            "accept_teleop_when_not_recording=true"
        )
    if config.get("hdf5_auto_start", False) and config.get(
        "teleop_controlled_by_recording", True
    ):
        raise MujocoConfigError(
            "hdf5_auto_start is incompatible with teleop_controlled_by_recording; "
            "start episodes through the recording service instead"
        )

    if config.get("enable_recording_service", True) and not config.get(
        "record_hdf5", True
    ):
        raise MujocoConfigError(
            "enable_recording_service requires record_hdf5"
        )

    _require_vector(config.get("initial_arm_joints"), "initial_arm_joints", 7)
    _require_vector(
        arm_config.get("initial_robot_pose"),
        "arm_config.initial_robot_pose",
        6,
    )
    for key in (
        "update_frequency",
        "scaling_factor",
        "smoothing_factor",
        "pose_filter_min_cutoff",
        "joints_filter_min_cutoff",
    ):
        _require_positive(arm_config.get(key), f"arm_config.{key}")
    for key in ("pose_filter_beta", "joints_filter_beta"):
        _require_positive(
            arm_config.get(key), f"arm_config.{key}", allow_zero=True
        )
    _require_positive(config.get("vp_ready_timeout", 2.0), "vp_ready_timeout")
    _require_positive(
        config.get("reset_ignore_teleop_duration", 0.5),
        "reset_ignore_teleop_duration",
        allow_zero=True,
    )

    for key in (
        "hdf5_force_hz",
        "hdf5_state_hz",
        "hdf5_image_hz",
        "max_joint_velocity",
    ):
        _require_positive(config.get(key), key)

    _require_positive(
        config.get("hdf5_async_stop_timeout", 10.0),
        "hdf5_async_stop_timeout",
    )
    _require_positive(
        config.get("render_start_timeout", 10.0),
        "render_start_timeout",
    )
    for key, default in (
        ("hdf5_async_queue_size", 1024),
        ("hdf5_append_block_image", 16),
        ("render_queue_size", 4),
    ):
        value = _require_integer(config.get(key, default), key)
        if value <= 0:
            raise MujocoConfigError(f"{key} must be positive")

    if config.get("visionpro_video_enabled", False):
        for key in ("cctv_window_width", "cctv_window_height"):
            value = _require_integer(config.get(key), key)
            if value <= 0:
                raise MujocoConfigError(f"{key} must be positive")
        _require_positive(
            config.get("camera_stream_fps", 15.0),
            "camera_stream_fps",
        )
        video_port = _require_integer(
            config.get("visionpro_video_port", 9999),
            "visionpro_video_port",
        )
        if not 1 <= video_port <= 65535:
            raise MujocoConfigError(
                "visionpro_video_port must be between 1 and 65535"
            )

    torque_limits = config.get("joint_torque_limits")
    if isinstance(torque_limits, (list, tuple)):
        _require_vector(torque_limits, "joint_torque_limits", 7)
        for index, value in enumerate(torque_limits):
            _require_positive(value, f"joint_torque_limits[{index}]")
    else:
        _require_positive(torque_limits, "joint_torque_limits")

    sampling_mode = str(config.get("hole_sampling_mode", "fixed")).lower()
    if sampling_mode not in {"grid", "uniform_random", "fixed"}:
        raise MujocoConfigError(
            "hole_sampling_mode must be one of: grid, uniform_random, fixed"
        )
    if sampling_mode == "grid":
        rows = _require_integer(config.get("hole_grid_rows", 5), "hole_grid_rows")
        cols = _require_integer(config.get("hole_grid_cols", 5), "hole_grid_cols")
        if rows <= 0 or cols <= 0:
            raise MujocoConfigError("hole_grid_rows and hole_grid_cols must be positive")
        _require_range(config.get("hole_grid_x_range"), "hole_grid_x_range")
        _require_range(config.get("hole_grid_z_range"), "hole_grid_z_range")
        _require_finite_number(
            config.get("hole_grid_y_offset", 0.0), "hole_grid_y_offset"
        )
        start_cycle = _require_integer(
            config.get("hole_grid_start_cycle", 0), "hole_grid_start_cycle"
        )
        start_index = _require_integer(
            config.get("hole_grid_start_index", 0), "hole_grid_start_index"
        )
        if start_cycle < 0:
            raise MujocoConfigError("hole_grid_start_cycle must be non-negative")
        if not 0 <= start_index < rows * cols:
            raise MujocoConfigError(
                "hole_grid_start_index must be within the configured grid"
            )

    force_thresholds = [
        _require_positive(config.get("force_feedback_low_threshold"), "force_feedback_low_threshold"),
        _require_positive(config.get("force_feedback_medium_threshold"), "force_feedback_medium_threshold"),
        _require_positive(config.get("force_feedback_high_threshold"), "force_feedback_high_threshold"),
        _require_positive(config.get("force_feedback_excessive_threshold"), "force_feedback_excessive_threshold"),
    ]
    if force_thresholds != sorted(force_thresholds) or len(set(force_thresholds)) != 4:
        raise MujocoConfigError(
            "force feedback thresholds must be strictly increasing"
        )


def build_arm_teleop_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    """Build the arm teleoperation config with shared initial-state values."""
    arm_config = deepcopy(dict(config.get("arm_config") or {}))
    arm_config["initial_arm_joints"] = list(config["initial_arm_joints"])
    arm_config["teleop_service_ns"] = str(
        arm_config.get("teleop_service_ns", "/arm_teleop_mujoco")
    ).rstrip("/")
    return arm_config


def build_controller_config(config: Mapping[str, Any]) -> Dict[str, Any]:
    """Forward every root setting and add controller-specific defaults."""
    controller_config = deepcopy(dict(config))
    arm_config = dict(config.get("arm_config") or {})
    service_ns = str(
        arm_config.get("teleop_service_ns", "/arm_teleop_mujoco")
    ).rstrip("/")

    controller_config.update(
        {
            "control_mode": config.get("control_mode", "actuator"),
            "launch_viewer": config.get("launch_viewer", True),
            "auto_start": config.get("auto_start", True),
            "viewer_rate": config.get("viewer_rate", 60.0),
            "realtime": config.get("realtime", True),
            "max_joint_velocity": config.get("max_joint_velocity", 0.1),
            "arm_joints": config.get(
                "arm_joints", [f"joint_{index}" for index in range(1, 8)]
            ),
            "arm_sign": config.get("arm_sign", [-1, 1, 1, -1, 1, 1, 1]),
            "viewer_start_wait": config.get("viewer_start_wait", 1.0),
            "enable_visual_guides": config.get("enable_visual_guides", False),
            "hole_axis_world": config.get("hole_axis_world", [0.0, 1.0, 0.0]),
            "hole_entrance_offset": config.get("hole_entrance_offset", 0.026),
            "hole_axis_arrow_length": config.get("hole_axis_arrow_length", 0.20),
            "guide_arrow_width": config.get("guide_arrow_width", 0.002),
            "guide_green_threshold": config.get("guide_green_threshold", 0.010),
            "guide_yellow_threshold": config.get("guide_yellow_threshold", 0.020),
            "show_camera_streams": config.get("show_camera_streams", True),
            "camera_stream_width": config.get("camera_stream_width", 640),
            "camera_stream_height": config.get("camera_stream_height", 480),
            "camera_stream_fps": config.get("camera_stream_fps", 15.0),
            "teleop_controlled_by_recording": config.get(
                "teleop_controlled_by_recording", True
            ),
            "accept_teleop_when_not_recording": config.get(
                "accept_teleop_when_not_recording", False
            ),
            "teleop_stop_service_name": config.get(
                "teleop_stop_service_name", f"{service_ns}/stop"
            ),
            "teleop_recalibrate_service_name": config.get(
                "teleop_recalibrate_service_name", f"{service_ns}/recalibrate"
            ),
            "teleop_start_service_name": config.get(
                "teleop_start_service_name", f"{service_ns}/start"
            ),
            "reset_arm_on_record_stop": config.get(
                "reset_arm_on_record_stop", True
            ),
            # TeleopSystemMujoco activates the controller only after ArmTeleop
            # has registered its episode services.
            "defer_runtime_activation": True,
        }
    )
    return controller_config
