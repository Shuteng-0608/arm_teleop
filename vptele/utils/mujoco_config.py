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
    review_mode: Optional[str] = None,
    target_episodes: Optional[int] = None,
    max_attempts: Optional[int] = None,
    reject_action: Optional[str] = None,
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
    if any(
        value is not None
        for value in (review_mode, target_episodes, max_attempts, reject_action)
    ):
        scripted_config = dict(result.get("scripted_controller") or {})
        if review_mode is not None:
            scripted_config["review_mode"] = str(review_mode).strip().lower()
        if target_episodes is not None:
            scripted_config["target_episodes"] = int(target_episodes)
        if max_attempts is not None:
            scripted_config["max_attempts"] = int(max_attempts)
        if reject_action is not None:
            scripted_config["reject_action"] = str(reject_action).strip().lower()
        result["scripted_controller"] = scripted_config
    return result


def validate_mujoco_config(config: Mapping[str, Any]) -> None:
    """Validate settings whose failure would otherwise be late or silent."""
    _require_mapping(config, "config")
    config_version = _require_integer(config.get("config_version", 1), "config_version")
    if config_version != 1:
        raise MujocoConfigError(
            f"Unsupported config_version: {config.get('config_version')!r}"
        )
    if config.get("vp_enabled", True) and not str(config.get("vp_ip", "")).strip():
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
    enable_ros_interfaces = bool(config.get("enable_ros_interfaces", True))
    if (
        enable_ros_interfaces
        and config.get("teleop_controlled_by_recording", True)
        and not arm_config.get("enable_episode_services", True)
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

    if (
        enable_ros_interfaces
        and config.get("enable_recording_service", True)
        and not config.get("record_hdf5", True)
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

    terminal_hold_max_time = _require_positive(
        config.get("task_success_terminal_hold_time", 1.0),
        "task_success_terminal_hold_time",
        allow_zero=True,
    )
    terminal_hold_min_time = _require_positive(
        config.get("task_success_terminal_hold_min_time", terminal_hold_max_time),
        "task_success_terminal_hold_min_time",
        allow_zero=True,
    )
    _require_positive(
        config.get("task_success_terminal_hold_stable_dwell_time", 0.0),
        "task_success_terminal_hold_stable_dwell_time",
        allow_zero=True,
    )
    if terminal_hold_min_time > terminal_hold_max_time:
        raise MujocoConfigError(
            "task success terminal hold times must satisfy min_time <= max_time"
        )

    success_force_axis = _require_vector(
        config.get("task_success_force_axis_world", [0.0, 1.0, 0.0]),
        "task_success_force_axis_world",
        3,
    )
    if sum(float(value) ** 2 for value in success_force_axis) <= 1e-18:
        raise MujocoConfigError("task_success_force_axis_world must be non-zero")

    for key in (
        "task_success_max_force_norm",
        "task_success_max_lateral_force",
        "task_success_max_arm_speed",
        "terminal_hold_max_force_norm",
        "terminal_hold_max_lateral_force",
        "terminal_hold_force_abort_dwell_time",
    ):
        if key in config:
            _require_positive(config[key], key)

    success_force = config.get("task_success_max_force_norm")
    hold_force = config.get("terminal_hold_max_force_norm")
    if (
        success_force is not None
        and hold_force is not None
        and float(hold_force) < float(success_force)
    ):
        raise MujocoConfigError(
            "terminal_hold_max_force_norm must be >= task_success_max_force_norm"
        )
    success_lateral = config.get("task_success_max_lateral_force")
    hold_lateral = config.get("terminal_hold_max_lateral_force")
    if (
        success_lateral is not None
        and hold_lateral is not None
        and float(hold_lateral) < float(success_lateral)
    ):
        raise MujocoConfigError(
            "terminal_hold_max_lateral_force must be >= "
            "task_success_max_lateral_force"
        )

    scripted_config = _require_mapping(
        config.get("scripted_controller", {}),
        "scripted_controller",
    )
    review_mode = str(scripted_config.get("review_mode", "manual")).lower()
    if review_mode not in {"manual", "auto"}:
        raise MujocoConfigError(
            "scripted_controller.review_mode must be manual or auto"
        )
    target_episodes = _require_integer(
        scripted_config.get("target_episodes", 0),
        "scripted_controller.target_episodes",
    )
    max_attempts = _require_integer(
        scripted_config.get("max_attempts", 0),
        "scripted_controller.max_attempts",
    )
    max_consecutive_rejections = _require_integer(
        scripted_config.get("max_consecutive_rejections", 10),
        "scripted_controller.max_consecutive_rejections",
    )
    if target_episodes < 0:
        raise MujocoConfigError(
            "scripted_controller.target_episodes must be non-negative"
        )
    if max_attempts < 0:
        raise MujocoConfigError(
            "scripted_controller.max_attempts must be non-negative"
        )
    if max_consecutive_rejections < 0:
        raise MujocoConfigError(
            "scripted_controller.max_consecutive_rejections must be non-negative"
        )
    if review_mode == "auto" and target_episodes <= 0:
        raise MujocoConfigError(
            "auto review mode requires scripted_controller.target_episodes > 0"
        )
    if review_mode == "auto" and not bool(scripted_config.get("enabled", False)):
        raise MujocoConfigError(
            "auto review mode requires scripted_controller.enabled=true"
        )
    if max_attempts and max_attempts < target_episodes:
        raise MujocoConfigError(
            "scripted_controller.max_attempts must be zero or at least target_episodes"
        )
    reject_action = str(
        scripted_config.get("reject_action", "quarantine")
    ).lower()
    if reject_action not in {"quarantine", "delete"}:
        raise MujocoConfigError(
            "scripted_controller.reject_action must be quarantine or delete"
        )

    scenario = str(scripted_config.get("scenario", "collision")).lower()
    if scenario not in {"collision", "clean"}:
        raise MujocoConfigError(
            "scripted_controller.scenario must be collision or clean"
        )

    coverage_mode = str(
        scripted_config.get("error_coverage_mode", "random_fixed_radius")
    ).lower()
    if coverage_mode not in {"random_fixed_radius", "stratified_radius_angle"}:
        raise MujocoConfigError(
            "scripted_controller.error_coverage_mode must be "
            "random_fixed_radius or stratified_radius_angle"
        )
    if coverage_mode == "stratified_radius_angle":
        radii = scripted_config.get("rim_contact_radii_mm")
        if not isinstance(radii, (list, tuple)) or not radii:
            raise MujocoConfigError(
                "scripted_controller.rim_contact_radii_mm must be a non-empty list"
            )
        for index, value in enumerate(radii):
            _require_positive(
                value,
                f"scripted_controller.rim_contact_radii_mm[{index}]",
            )
        if len(set(float(value) for value in radii)) != len(radii):
            raise MujocoConfigError(
                "scripted_controller.rim_contact_radii_mm must not contain duplicates"
            )
        angle_bins = _require_integer(
            scripted_config.get("rim_contact_angle_bins", 24),
            "scripted_controller.rim_contact_angle_bins",
        )
        if angle_bins <= 0:
            raise MujocoConfigError(
                "scripted_controller.rim_contact_angle_bins must be positive"
            )
        angle_jitter = _require_positive(
            scripted_config.get("rim_contact_angle_jitter_deg", 0.0),
            "scripted_controller.rim_contact_angle_jitter_deg",
            allow_zero=True,
        )
        if float(angle_jitter) > 180.0 / angle_bins:
            raise MujocoConfigError(
                "scripted_controller.rim_contact_angle_jitter_deg must not "
                "exceed half an angle bin"
            )
        coverage_order = str(
            scripted_config.get("error_coverage_order", "shuffled")
        ).lower()
        if coverage_order not in {"row_major", "shuffled"}:
            raise MujocoConfigError(
                "scripted_controller.error_coverage_order must be "
                "row_major or shuffled"
            )
        start_cycle = _require_integer(
            scripted_config.get("error_coverage_start_cycle", 0),
            "scripted_controller.error_coverage_start_cycle",
        )
        start_index = _require_integer(
            scripted_config.get("error_coverage_start_index", 0),
            "scripted_controller.error_coverage_start_index",
        )
        coverage_size = len(radii) * angle_bins
        coverage_seed = _require_integer(
            scripted_config.get("error_coverage_seed", 42),
            "scripted_controller.error_coverage_seed",
        )
        if coverage_seed < 0:
            raise MujocoConfigError(
                "scripted_controller.error_coverage_seed must be non-negative"
            )
        if start_cycle < 0 or not 0 <= start_index < coverage_size:
            raise MujocoConfigError(
                "scripted_controller error coverage start cursor is out of range"
            )

    for key, default in (
        ("approach_control_period_s", 0.03),
        ("approach_force_poll_period_s", 0.005),
        ("approach_free_speed_m_s", 0.050),
        ("approach_far_speed_m_s", 0.025),
        ("approach_near_speed_m_s", 0.008),
        ("approach_probe_speed_m_s", 0.002),
        ("approach_far_distance_m", 0.010),
        ("approach_near_distance_m", 0.003),
        ("approach_arrival_tolerance_m", 0.0005),
        ("approach_probe_activation_distance_m", 0.002),
        ("wall_contact_detect_force_n", 3.0),
        ("wall_contact_detect_dwell_s", 0.015),
        ("wall_contact_target_dwell_s", 0.005),
        ("wall_contact_max_push_m", 0.002),
        ("wall_contact_settle_s", 0.30),
    ):
        _require_positive(
            scripted_config.get(key, default), f"scripted_controller.{key}"
        )
    near_distance = float(scripted_config.get("approach_near_distance_m", 0.003))
    far_distance = float(scripted_config.get("approach_far_distance_m", 0.010))
    if not near_distance < far_distance:
        raise MujocoConfigError(
            "scripted_controller approach distances must satisfy near < far"
        )
    detect_force = float(scripted_config.get("wall_contact_detect_force_n", 3.0))
    contact_min = float(scripted_config.get("wall_contact_force_min", 15.0))
    contact_max = float(scripted_config.get("wall_contact_force_max", 28.0))
    if not 0.0 < detect_force <= contact_min <= contact_max < 40.0:
        raise MujocoConfigError(
            "scripted_controller contact forces must satisfy "
            "0 < detect <= min <= max < 40 N"
        )

    if bool(scripted_config.get("in_hole_correction_enabled", False)):
        if str(config.get("hdf5_ft_compensation_mode", "gravity")).lower() != "gravity":
            raise MujocoConfigError(
                "in-hole correction requires hdf5_ft_compensation_mode=gravity"
            )
        gravity_bodies = config.get("hdf5_ft_gravity_tool_body_names")
        if not isinstance(gravity_bodies, (list, tuple)) or not gravity_bodies:
            raise MujocoConfigError(
                "in-hole correction requires hdf5_ft_gravity_tool_body_names"
            )

        depths = scripted_config.get("in_hole_disturbance_depth_fractions")
        if not isinstance(depths, (list, tuple)) or not depths:
            raise MujocoConfigError(
                "scripted_controller.in_hole_disturbance_depth_fractions "
                "must be a non-empty list"
            )
        parsed_depths = []
        for index, value in enumerate(depths):
            parsed = _require_positive(
                value,
                "scripted_controller.in_hole_disturbance_depth_fractions"
                f"[{index}]",
            )
            if float(parsed) >= 1.0:
                raise MujocoConfigError(
                    "in-hole disturbance depth fractions must be in (0, 1)"
                )
            parsed_depths.append(float(parsed))
        if len(set(parsed_depths)) != len(parsed_depths):
            raise MujocoConfigError(
                "in-hole disturbance depth fractions must not contain duplicates"
            )

        amplitudes = scripted_config.get("in_hole_disturbance_amplitudes_mm")
        if not isinstance(amplitudes, (list, tuple)) or not amplitudes:
            raise MujocoConfigError(
                "scripted_controller.in_hole_disturbance_amplitudes_mm "
                "must be a non-empty list"
            )
        parsed_amplitudes = [
            float(
                _require_positive(
                    value,
                    "scripted_controller.in_hole_disturbance_amplitudes_mm"
                    f"[{index}]",
                )
            )
            for index, value in enumerate(amplitudes)
        ]
        if len(set(parsed_amplitudes)) != len(parsed_amplitudes):
            raise MujocoConfigError(
                "in-hole disturbance amplitudes must not contain duplicates"
            )
        clearance_mm = float(
            _require_positive(
                scripted_config.get("in_hole_nominal_clearance_mm", 3.0),
                "scripted_controller.in_hole_nominal_clearance_mm",
            )
        )
        if min(parsed_amplitudes) <= clearance_mm:
            raise MujocoConfigError(
                "in-hole disturbance amplitudes must exceed nominal clearance"
            )

        direction_bins = _require_integer(
            scripted_config.get("in_hole_disturbance_direction_bins", 8),
            "scripted_controller.in_hole_disturbance_direction_bins",
        )
        if direction_bins <= 0:
            raise MujocoConfigError(
                "in-hole disturbance direction bins must be positive"
            )
        coverage_order = str(
            scripted_config.get(
                "in_hole_disturbance_coverage_order", "shuffled"
            )
        ).lower()
        if coverage_order not in {"row_major", "shuffled"}:
            raise MujocoConfigError(
                "in-hole disturbance coverage order must be row_major or shuffled"
            )
        coverage_seed = _require_integer(
            scripted_config.get("in_hole_disturbance_coverage_seed", 73),
            "scripted_controller.in_hole_disturbance_coverage_seed",
        )
        start_cycle = _require_integer(
            scripted_config.get("in_hole_disturbance_start_cycle", 0),
            "scripted_controller.in_hole_disturbance_start_cycle",
        )
        start_index = _require_integer(
            scripted_config.get("in_hole_disturbance_start_index", 0),
            "scripted_controller.in_hole_disturbance_start_index",
        )
        coverage_size = len(depths) * direction_bins * len(amplitudes)
        if (
            coverage_seed < 0
            or start_cycle < 0
            or not 0 <= start_index < coverage_size
        ):
            raise MujocoConfigError(
                "in-hole disturbance coverage seed/cursor is out of range"
            )

        for key, default in (
            ("in_hole_control_period_s", 0.03),
            ("in_hole_contact_detect_force_n", 2.0),
            ("in_hole_contact_detect_dwell_s", 0.03),
            ("in_hole_contact_release_force_n", 1.5),
            ("in_hole_contact_release_dwell_s", 0.09),
            ("in_hole_contact_target_force_n", 5.0),
            ("in_hole_contact_target_dwell_s", 0.03),
            ("in_hole_force_limit_n", 25.0),
            ("in_hole_correction_gain_m_per_n", 0.00008),
            ("in_hole_correction_max_step_m", 0.00025),
            ("in_hole_correction_max_travel_m", 0.006),
            ("in_hole_correction_timeout_s", 3.0),
        ):
            _require_positive(
                scripted_config.get(key, default),
                f"scripted_controller.{key}",
            )
        filter_alpha = float(
            _require_positive(
                scripted_config.get("in_hole_force_filter_alpha", 0.30),
                "scripted_controller.in_hole_force_filter_alpha",
            )
        )
        if filter_alpha > 1.0:
            raise MujocoConfigError(
                "scripted_controller.in_hole_force_filter_alpha must be <= 1"
            )
        detection_min_fraction = float(
            _require_positive(
                scripted_config.get(
                    "in_hole_contact_detection_min_fraction", 0.70
                ),
                "scripted_controller.in_hole_contact_detection_min_fraction",
            )
        )
        if detection_min_fraction > 1.0:
            raise MujocoConfigError(
                "scripted_controller.in_hole_contact_detection_min_fraction "
                "must be <= 1"
            )
        release_force = float(
            scripted_config.get("in_hole_contact_release_force_n", 1.5)
        )
        detect_force = float(
            scripted_config.get("in_hole_contact_detect_force_n", 2.0)
        )
        target_force = float(
            scripted_config.get("in_hole_contact_target_force_n", 5.0)
        )
        in_hole_limit = float(
            scripted_config.get("in_hole_force_limit_n", 25.0)
        )
        if not (
            0.0
            < release_force
            < detect_force
            <= target_force
            < in_hole_limit
            < 40.0
        ):
            raise MujocoConfigError(
                "in-hole forces must satisfy "
                "0 < release < detect <= target < limit < 40 N"
            )
        correction_sign = float(
            scripted_config.get("in_hole_force_correction_sign", -1.0)
        )
        if correction_sign not in {-1.0, 1.0}:
            raise MujocoConfigError(
                "scripted_controller.in_hole_force_correction_sign must be -1 or 1"
            )
        for key, default, allow_zero in (
            ("in_hole_disturbance_ramp_steps", 18, False),
            ("in_hole_disturbance_hold_steps", 4, True),
            ("in_hole_return_center_steps", 10, False),
        ):
            value = _require_integer(
                scripted_config.get(key, default),
                f"scripted_controller.{key}",
            )
            if value < 0 or (not allow_zero and value == 0):
                raise MujocoConfigError(
                    f"scripted_controller.{key} has an invalid value"
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
