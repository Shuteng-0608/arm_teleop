#!/usr/bin/env python3
"""ROS wrapper for MH6 normalized commands and DexHandControl."""

from __future__ import annotations

import math
import threading
from typing import Iterable, List, Optional, Tuple

import rospy

from arm_teleop.msg import MH6NormalizedCommand
from mh6_modbus_dev import DexHandControl


class MH6ModbusROS:
    def __init__(self) -> None:
        self.command_topic = rospy.get_param(
            "~command_topic", "/arm_teleop/mh6_normalized_cmd"
        )
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = int(rospy.get_param("~baudrate", 115200))
        self.wait_status = bool(rospy.get_param("~wait_status", False))
        self.free_on_start = bool(rospy.get_param("~free_on_start", True))
        self.free_on_shutdown = bool(rospy.get_param("~free_on_shutdown", False))
        self.command_timeout_sec = float(rospy.get_param("~command_timeout_sec", 0.5))
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))
        self.require_calibrated = bool(rospy.get_param("~require_calibrated", True))

        self._lock = threading.Lock()
        self._latest_msg: Optional[MH6NormalizedCommand] = None
        self._latest_receive_time: Optional[rospy.Time] = None

        self.hand = DexHandControl(port=self.port, baudrate=self.baudrate)
        if not self.hand.start_persistent_connection():
            raise RuntimeError("failed to start MH6 persistent Modbus connection")

        if self.free_on_start:
            self._free_all("startup")

        self._subscriber = rospy.Subscriber(
            self.command_topic,
            MH6NormalizedCommand,
            self._command_callback,
            queue_size=1,
        )
        rospy.loginfo(
            "MH6 Modbus ROS node subscribed to %s using port %s at %d baud.",
            self.command_topic,
            self.port,
            self.baudrate,
        )

    def _command_callback(self, msg: MH6NormalizedCommand) -> None:
        with self._lock:
            self._latest_msg = msg
            self._latest_receive_time = rospy.Time.now()

    def spin(self) -> None:
        rate_hz = self.send_rate_hz if self.send_rate_hz > 0.0 else 20.0
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            self._process_latest_command()
            rate.sleep()

    def _process_latest_command(self) -> None:
        msg, receive_time = self._get_latest_command()
        if msg is None or receive_time is None:
            return

        age = (rospy.Time.now() - receive_time).to_sec()
        if self.command_timeout_sec > 0.0 and age > self.command_timeout_sec:
            rospy.logwarn_throttle(
                2.0,
                "Skipping stale MH6 command: age %.3fs exceeds timeout %.3fs.",
                age,
                self.command_timeout_sec,
            )
            return

        if self.require_calibrated and not msg.calibrated:
            rospy.logwarn_throttle(
                2.0,
                "Skipping MH6 command because message is not calibrated.",
            )
            return

        command = self._validate_command(msg)
        if command is None:
            return
        finger_values, palm_values, palm_times = command

        try:
            self.hand.move_hand_normalized(
                finger_values=finger_values,
                palm_values=palm_values,
                palm_times=palm_times,
                wait_status=self.wait_status,
            )
        except Exception as exc:
            rospy.logerr_throttle(
                2.0,
                "MH6 move_hand_normalized failed: %s",
                exc,
            )

    def _get_latest_command(
        self,
    ) -> Tuple[Optional[MH6NormalizedCommand], Optional[rospy.Time]]:
        with self._lock:
            return self._latest_msg, self._latest_receive_time

    def _validate_command(
        self, msg: MH6NormalizedCommand
    ) -> Optional[Tuple[List[float], List[float], List[int]]]:
        if len(msg.finger_values) != 5:
            rospy.logwarn_throttle(
                2.0,
                "Invalid MH6 command: expected 5 finger values, got %d.",
                len(msg.finger_values),
            )
            return None
        if len(msg.palm_values) != 3:
            rospy.logwarn_throttle(
                2.0,
                "Invalid MH6 command: expected 3 palm values, got %d.",
                len(msg.palm_values),
            )
            return None
        if len(msg.palm_times) != 3:
            rospy.logwarn_throttle(
                2.0,
                "Invalid MH6 command: expected 3 palm times, got %d.",
                len(msg.palm_times),
            )
            return None

        finger_values = self._normalized_values("finger_values", msg.finger_values)
        palm_values = self._normalized_values("palm_values", msg.palm_values)
        palm_times = self._palm_times(msg.palm_times)
        if finger_values is None or palm_values is None or palm_times is None:
            return None
        return finger_values, palm_values, palm_times

    def _normalized_values(
        self, label: str, values: Iterable[float]
    ) -> Optional[List[float]]:
        clipped = []
        did_clip = False
        for value in values:
            try:
                value = float(value)
            except (TypeError, ValueError):
                rospy.logwarn_throttle(
                    2.0,
                    "Invalid MH6 command: %s contains non-numeric value.",
                    label,
                )
                return None
            if not math.isfinite(value):
                rospy.logwarn_throttle(
                    2.0,
                    "Invalid MH6 command: %s contains non-finite value.",
                    label,
                )
                return None
            clipped_value = min(max(value, 0.0), 1.0)
            did_clip = did_clip or clipped_value != value
            clipped.append(clipped_value)

        if did_clip:
            rospy.logwarn_throttle(
                2.0,
                "MH6 command %s had values outside [0, 1]; clipped before send.",
                label,
            )
        return clipped

    def _palm_times(self, values: Iterable[int]) -> Optional[List[int]]:
        clipped = []
        did_clip = False
        for value in values:
            try:
                value = int(value)
            except (TypeError, ValueError):
                rospy.logwarn_throttle(
                    2.0,
                    "Invalid MH6 command: palm_times contains non-integer value.",
                )
                return None
            clipped_value = min(max(value, 1), 1000)
            did_clip = did_clip or clipped_value != value
            clipped.append(clipped_value)

        if did_clip:
            rospy.logwarn_throttle(
                2.0,
                "MH6 command palm_times had values outside [1, 1000]; clipped before send.",
            )
        return clipped

    def shutdown(self) -> None:
        if self.free_on_shutdown:
            self._free_all("shutdown")
        self.hand.stop_persistent_connection()
        rospy.loginfo("MH6 Modbus ROS node shutdown complete.")

    def _free_all(self, reason: str) -> None:
        try:
            if self.hand.free_all():
                rospy.loginfo("MH6 free_all succeeded during %s.", reason)
            else:
                rospy.logwarn("MH6 free_all failed during %s.", reason)
        except Exception as exc:
            rospy.logwarn("MH6 free_all raised during %s: %s", reason, exc)


def main() -> None:
    rospy.init_node("mh6_modbus_ros")
    node = MH6ModbusROS()
    rospy.on_shutdown(node.shutdown)
    node.spin()


if __name__ == "__main__":
    main()
