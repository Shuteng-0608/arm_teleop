from __future__ import annotations

import copy
import math
import time
from numbers import Real
from typing import Dict, List, Optional

import rospy

from arm_teleop.msg import MH6NormalizedCommand
from end_effectors.end_effector_base import EndEffectorBase
from end_effectors.mh6.mh6_mapping import MH6HandMapper
from end_effectors.mh6.visionpro_adapter import extract_mh6_points
from utils.logger import get_logger

logger = get_logger()


class CommandLowPassFilter:
    """Time-aware first-order low-pass filter for normalized command sections."""

    def __init__(self, tau: float) -> None:
        self.tau = float(tau)
        self.previous: Dict[tuple, float] = {}
        self.previous_timestamp: Optional[float] = None

    def reset(self) -> None:
        self.previous.clear()
        self.previous_timestamp = None

    def apply(
        self,
        result: Dict[str, Dict[str, float]],
        now: float,
    ) -> Dict[str, Dict[str, float]]:
        filtered_result = copy.deepcopy(result)

        if self.previous_timestamp is None:
            for section_name in ("low_dim", "palm"):
                for key, value in result.get(section_name, {}).items():
                    if isinstance(value, Real) and not isinstance(value, bool):
                        self.previous[(section_name, key)] = float(value)
            self.previous_timestamp = now
            return filtered_result

        dt = now - self.previous_timestamp
        alpha = 0.0 if dt <= 0.0 else 1.0 - math.exp(-dt / self.tau)
        alpha = min(max(alpha, 0.0), 1.0)

        for section_name in ("low_dim", "palm"):
            raw_section = result.get(section_name, {})
            filtered_section = filtered_result.get(section_name, {})
            for key, value in raw_section.items():
                if not isinstance(value, Real) or isinstance(value, bool):
                    continue

                state_key = (section_name, key)
                current = float(value)
                previous = self.previous.get(state_key, current)
                filtered = previous + alpha * (current - previous)
                filtered_section[key] = filtered
                self.previous[state_key] = filtered

        if dt > 0.0:
            self.previous_timestamp = now
        return filtered_result


class MH6HandTeleopROS(EndEffectorBase):
    """Publish MH6 normalized teleoperation commands from the existing Vision Pro stream."""

    def __init__(self, vp_streamer, robot_controller, config=None):
        super().__init__(vp_streamer, robot_controller, config)
        self.hand = self.config.get("hand", "left")
        if self.hand not in ("left", "right"):
            raise ValueError("mh6_config.hand must be 'left' or 'right'")

        self.topic = self.config.get("topic", "/arm_teleop/mh6_normalized_cmd")
        self.update_frequency = float(self.config.get("update_frequency", 0.05))
        self.calibrate_seconds = float(self.config.get("calibrate_seconds", 2.0))
        self.filter_tau = float(self.config.get("filter_tau", 0.24))
        self.palm_times = self._parse_palm_times(self.config.get("palm_times", [50, 50, 50]))
        self.allow_legacy_25_fingers = bool(self.config.get("allow_legacy_25_fingers", True))
        queue_size = int(self.config.get("queue_size", 1))

        self.mapper = MH6HandMapper()
        self.command_filter = CommandLowPassFilter(self.filter_tau) if self.filter_tau > 0.0 else None
        self.calibrated = False
        self.publisher = rospy.Publisher(self.topic, MH6NormalizedCommand, queue_size=queue_size)

        self.initialize()

    @staticmethod
    def _parse_palm_times(values) -> List[int]:
        palm_times = [int(value) for value in values]
        if len(palm_times) != 3:
            raise ValueError("mh6_config.palm_times must contain exactly 3 values")
        return palm_times

    def initialize(self):
        logger.info("Initializing MH6 normalized topic bridge...")
        if self.calibrate_seconds <= 0.0:
            rospy.logwarn("MH6 open-hand calibration disabled because calibrate_seconds <= 0.")
            return

        samples = self._collect_open_hand_samples(self.calibrate_seconds)
        if not samples:
            rospy.logwarn(
                "MH6 open-hand calibration failed: no valid Vision Pro hand samples collected. "
                "Commands will not be published until calibration succeeds."
            )
            return

        try:
            self.mapper.calibrate_open(samples)
        except ValueError as exc:
            rospy.logwarn(f"MH6 open-hand calibration failed: {exc}")
            return

        self.calibrated = True
        logger.info(f"MH6 open-hand calibration complete with {len(samples)} samples")

    def _collect_open_hand_samples(self, duration: float) -> List:
        period = max(self.update_frequency, 0.001)
        deadline = time.time() + duration
        samples = []

        while time.time() < deadline and not rospy.is_shutdown():
            loop_start = time.time()
            vp_data = self.vp_streamer.latest
            points = extract_mh6_points(vp_data, self.hand, self.allow_legacy_25_fingers)
            if points is not None:
                samples.append(points)

            sleep_time = period - (time.time() - loop_start)
            if sleep_time > 0.0:
                time.sleep(sleep_time)

        return samples

    def process_vp_data(self, vp_data):
        return extract_mh6_points(vp_data, self.hand, self.allow_legacy_25_fingers)

    def update(self):
        vp_data = self.vp_streamer.latest
        points = self.process_vp_data(vp_data)
        if points is None:
            rospy.logwarn_throttle(5.0, "MH6 topic bridge skipped invalid or missing Vision Pro hand data.")
            return

        if not self.calibrated:
            rospy.logwarn_throttle(5.0, "MH6 topic bridge is not calibrated; skipping command publish.")
            return

        try:
            raw_result = self.mapper.step(points)
        except ValueError as exc:
            rospy.logwarn_throttle(5.0, f"MH6 mapping rejected Vision Pro points: {exc}")
            return

        now = time.time()
        filtered = self.command_filter is not None
        result = self.command_filter.apply(raw_result, now) if filtered else raw_result

        low_dim = result["low_dim"]
        intent = result.get("intent", {})
        finger_values = [
            low_dim["u_thumb"],
            low_dim["u_index"],
            low_dim["u_middle"],
            low_dim["u_ring"],
            low_dim["u_little"],
        ]
        palm_values = [low_dim["u_h"], low_dim["u_h"], low_dim["u_h"]]

        msg = MH6NormalizedCommand()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "vision_pro"
        msg.hand = self.hand
        msg.calibrated = True
        msg.filtered = filtered
        msg.finger_values = [float(value) for value in finger_values]
        msg.palm_values = [float(value) for value in palm_values]
        msg.palm_times = self.palm_times
        msg.low_dim = [
            float(low_dim["u_thumb"]),
            float(low_dim["u_index"]),
            float(low_dim["u_middle"]),
            float(low_dim["u_ring"]),
            float(low_dim["u_little"]),
            float(low_dim["u_h"]),
            float(low_dim["u_v"]),
        ]
        msg.intent = [
            float(intent.get("P_opp", 0.0)),
            float(intent.get("g", 0.0)),
            float(intent.get("t", 0.0)),
        ]

        self.publisher.publish(msg)
