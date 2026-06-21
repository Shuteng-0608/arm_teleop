import time
from pathlib import Path

import rospy

from arm_teleop.msg import MH6Command
from end_effectors.end_effector_base import EndEffectorBase
from end_effectors.mh6.mh6_backend import create_backend
from end_effectors.mh6.mh6_controller import (
    CommandRateLimiter,
    LowDimHandCommand,
    TeleopCalibration,
    low_dim_to_actuator_command,
)
from end_effectors.mh6.mh6_mapping import MH6HandMapper, MappingCalibration
from end_effectors.mh6.mh6_safety import (
    evaluate_hardware_safety,
    validate_actuator_command,
)
from end_effectors.mh6.visionpro_adapter import extract_canonical_hand_data
from utils.logger import get_logger

logger = get_logger()


class MH6HandTeleopROS(EndEffectorBase):
    """Dry-run MH6 hand adapter for validating Vision Pro hand data."""

    def __init__(self, vp_streamer, robot_controller, config=None):
        config = config or {}
        super().__init__(vp_streamer, robot_controller, config)

        self.hand = str(self.config.get('hand', 'left')).strip().lower()
        if self.hand not in ('left', 'right'):
            raise ValueError("mh6_config.hand must be 'left' or 'right'")

        self.prefer_full_skeleton = bool(self.config.get('prefer_full_skeleton', True))
        self.allow_legacy_25_fingers = bool(self.config.get('allow_legacy_25_fingers', True))
        self.print_debug = bool(self.config.get('print_debug', True))
        self.actuator_debug = bool(self.config.get('actuator_debug', True))
        self.rate_limit_enabled = bool(self.config.get('rate_limit_enabled', True))
        self.max_delta_per_update = int(self.config.get('max_delta_per_update', 80))
        self.palm_times = self.config.get('palm_times', None)
        self.command_topic = self.config.get('command_topic', '/arm_teleop/mh6_command')
        self.publish_debug_topic = bool(self.config.get('publish_debug_topic', True))
        self.requested_backend = str(self.config.get('backend', 'dry_run')).strip().lower()
        self.requested_hardware = bool(self.config.get('enable_hardware', False))

        # This milestone is intentionally dry-run only. Hardware output is blocked
        # even when a YAML file accidentally enables it.
        self.dry_run = True
        self.enable_hardware = False

        self.latest_points = None
        self.latest_mapping_result = None
        self.latest_actuator_command = None
        self.latest_source_type = "unknown"
        self.latest_joint_count = 0
        self.valid_frame_count = 0
        self.last_debug_time = 0.0
        self.command_publisher = None
        self.hardware_safety_decision = evaluate_hardware_safety(self.config)
        self.backend = create_backend(self.config, self.hardware_safety_decision)

        self.calibration = self._load_calibration()
        self.mapper = MH6HandMapper(self._make_mapping_calibration(self.calibration))
        self.rate_limiter = CommandRateLimiter(
            self.calibration,
            enabled=self.rate_limit_enabled,
            max_delta_per_update=self.max_delta_per_update,
        )

        requested_dry_run = self.config.get('dry_run', True)
        if requested_dry_run is not True or self.requested_hardware:
            rospy.logwarn(
                "MH6 milestone is dry-run only; forcing dry_run=True and "
                "enable_hardware=False"
            )
        if self.hardware_safety_decision.requested:
            rospy.logwarn(
                "MH6 hardware safety decision: requested=%s eligible=%s reason=%s",
                self.hardware_safety_decision.requested,
                self.hardware_safety_decision.eligible,
                self.hardware_safety_decision.reason,
            )
            if self.hardware_safety_decision.eligible:
                rospy.logwarn(
                    "Hardware configuration is eligible, but ModbusHardwareBackend "
                    "is not implemented in this milestone. Commands will be suppressed."
                )

        if self.publish_debug_topic:
            self.command_publisher = rospy.Publisher(
                self.command_topic,
                MH6Command,
                queue_size=100,
            )

        self.backend.start()
        self.initialize()

    def initialize(self):
        rospy.loginfo(
            "MH6 dry-run end-effector initialized: hand=%s, "
            "backend=%s (%s), dry_run=%s, "
            "update_frequency=%.3fs, prefer_full_skeleton=%s, "
            "allow_legacy_25_fingers=%s, rate_limit_enabled=%s, "
            "hardware_enabled=%s, command_topic=%s, hardware_requested=%s, "
            "hardware_eligible=%s",
            self.hand,
            self.requested_backend,
            self.backend.__class__.__name__,
            self.dry_run,
            self.update_frequency,
            self.prefer_full_skeleton,
            self.allow_legacy_25_fingers,
            self.rate_limit_enabled,
            self.enable_hardware,
            self.command_topic,
            self.hardware_safety_decision.requested,
            self.hardware_safety_decision.eligible,
        )

    def process_vp_data(self, vp_data):
        return extract_canonical_hand_data(
            vp_data,
            self.hand,
            prefer_full_skeleton=self.prefer_full_skeleton,
            allow_legacy_25_fingers=self.allow_legacy_25_fingers,
        )

    def update(self):
        canonical = self.process_vp_data(self.vp_streamer.latest)
        if canonical is None:
            return
        points = canonical.points

        try:
            mapping_result = self.mapper.step(points)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, "MH6 dry-run: mapping skipped: %s", exc)
            return

        low_dim_command = LowDimHandCommand.from_mapping_result(mapping_result["low_dim"])
        actuator_command = low_dim_to_actuator_command(
            low_dim_command,
            self.calibration,
            palm_times=self.palm_times,
        )
        limited_command = self.rate_limiter.apply(actuator_command)
        valid_command, validation_reason = validate_actuator_command(
            limited_command,
            self.calibration,
        )
        if not valid_command:
            rospy.logwarn_throttle(
                5.0,
                "MH6 dry-run: dropping invalid actuator command: %s",
                validation_reason,
            )
            return

        self.latest_points = points
        self.latest_mapping_result = mapping_result
        self.latest_actuator_command = limited_command
        self.latest_source_type = canonical.source_type
        self.latest_joint_count = points.shape[0]
        self.valid_frame_count += 1

        self.backend.send(
            limited_command,
            context={
                "hand": self.hand,
                "source": canonical.source_type,
                "frame": self.valid_frame_count,
                "dry_run": self.dry_run,
                "hardware_enabled": self.enable_hardware,
            },
        )

        if self.command_publisher is not None:
            self.command_publisher.publish(
                self._build_command_msg(
                    mapping_result,
                    limited_command,
                    canonical.source_type,
                )
            )

        if self.print_debug or self.actuator_debug:
            now = time.time()
            if now - self.last_debug_time >= 2.0:
                wrist = points[0]
                low_dim = mapping_result["low_dim"]
                palm = mapping_result["palm"]
                intent = mapping_result["intent"]
                rospy.loginfo(
                    "MH6 dry-run: hand=%s source=%s frame=%d joints=%d wrist=[%.3f, %.3f, %.3f] "
                    "low_dim(T=%.2f I=%.2f M=%.2f R=%.2f L=%.2f h=%.2f v=%.2f) "
                    "palm(thumb=%.2f little=%.2f) intent(P_opp=%.2f g=%.2f t=%.2f) "
                    "actuator(fingers=%s palms=%s palm_times=%s rate_limit=%s hardware_enabled=%s)",
                    self.hand,
                    canonical.source_type,
                    self.valid_frame_count,
                    self.latest_joint_count,
                    wrist[0],
                    wrist[1],
                    wrist[2],
                    low_dim["u_thumb"],
                    low_dim["u_index"],
                    low_dim["u_middle"],
                    low_dim["u_ring"],
                    low_dim["u_little"],
                    low_dim["u_h"],
                    low_dim["u_v"],
                    palm["thumbSide"],
                    palm["littleSide"],
                    intent["P_opp"],
                    intent["g"],
                    intent["t"],
                    limited_command.finger_positions,
                    limited_command.palm_positions,
                    limited_command.palm_times,
                    self.rate_limit_enabled,
                    self.enable_hardware,
                )
                self.last_debug_time = now

    def _load_calibration(self):
        calibration_path = self.config.get('calibration_path')
        if not calibration_path:
            rospy.logwarn("MH6 dry-run: no calibration_path set; using built-in calibration")
            return TeleopCalibration()

        resolved_path = self._resolve_repo_path(calibration_path)
        try:
            calibration = TeleopCalibration.load_json(resolved_path)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to load MH6 calibration from {resolved_path}: {exc}"
            ) from exc

        rospy.loginfo("MH6 dry-run: loaded calibration from %s", resolved_path)
        return calibration

    @staticmethod
    def _make_mapping_calibration(calibration):
        return MappingCalibration(
            curl_open=dict(calibration.curl_open),
            curl_closed=dict(calibration.curl_closed),
            opposition_open_dist=dict(calibration.opposition_open_dist),
            opposition_closed_dist=dict(calibration.opposition_closed_dist),
            opposition_threshold=calibration.opposition_threshold,
        )

    @staticmethod
    def _resolve_repo_path(path_value):
        path = Path(path_value).expanduser()
        if path.is_absolute():
            return path
        repo_root = Path(__file__).resolve().parents[3]
        return repo_root / path

    def _build_command_msg(self, mapping_result, actuator_command, source_type):
        low_dim = mapping_result["low_dim"]
        palm = mapping_result["palm"]
        intent = mapping_result["intent"]

        msg = MH6Command()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "mh6_dry_run"
        msg.hand = self.hand
        msg.source = source_type
        msg.dry_run = True
        msg.hardware_enabled = False
        msg.rate_limit_enabled = self.rate_limit_enabled

        msg.low_dim = [
            float(low_dim["u_thumb"]),
            float(low_dim["u_index"]),
            float(low_dim["u_middle"]),
            float(low_dim["u_ring"]),
            float(low_dim["u_little"]),
            float(low_dim["u_h"]),
            float(low_dim["u_v"]),
        ]
        msg.palm_norm = [
            float(palm["thumbSide"]),
            float(palm["littleSide"]),
            float(low_dim["u_h"]),
        ]
        msg.intent = [
            float(intent["P_opp"]),
            float(intent["g"]),
            float(intent["t"]),
        ]
        msg.finger_positions = self._fixed_int_list(actuator_command.finger_positions, 5, 0)
        msg.palm_positions = self._fixed_int_list(actuator_command.palm_positions, 3, 0)
        msg.palm_times = self._fixed_int_list(actuator_command.palm_times, 3, 0)
        return msg

    @staticmethod
    def _fixed_int_list(values, size, default):
        fixed = [int(value) for value in values[:size]]
        while len(fixed) < size:
            fixed.append(default)
        return fixed

    def stop(self):
        super().stop()
        self.backend.stop()
