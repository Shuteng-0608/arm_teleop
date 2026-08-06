#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
VPTELE_ROOT = REPO_ROOT / "vptele"
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
if str(VPTELE_ROOT) not in sys.path:
    sys.path.insert(0, str(VPTELE_ROOT))

from arm_control.robot_controller_mujoco_peg_tool_contact import (  # noqa: E402
    RobotControllerMuJoCoPegTool,
)
from core.teleop_system_mujoco import TeleopSystemMujoco  # noqa: E402
from core.vp_streamer_avp import VPStreamer  # noqa: E402


class VPStreamerVideoTest(unittest.TestCase):
    @patch("core.vp_streamer_avp.AVPStreamer")
    def test_video_api_configures_mono_webrtc_and_forwards_bgr_frame(
        self,
        avp_streamer_cls,
    ):
        underlying = avp_streamer_cls.return_value
        underlying.start_webrtc.return_value = True
        underlying.is_connected.return_value = True
        streamer = VPStreamer("192.0.2.1", record=False)

        self.assertTrue(
            streamer.start_video_stream(1280, 720, 15.0, port=9999)
        )
        underlying.configure_video.assert_called_once_with(
            device=None,
            format=None,
            size="1280x720",
            fps=15,
            stereo=False,
        )
        underlying.start_webrtc.assert_called_once_with(
            port=9999,
            blocking=False,
        )

        frame_bgr = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.assertTrue(streamer.update_video_frame(frame_bgr))
        underlying.update_frame.assert_called_once_with(frame_bgr)
        self.assertTrue(streamer.is_video_connected())

        streamer.close()
        streamer.close()
        underlying.cleanup.assert_called_once_with()

    @patch("core.vp_streamer_avp.AVPStreamer")
    def test_failed_webrtc_start_leaves_frame_updates_disabled(
        self,
        avp_streamer_cls,
    ):
        underlying = avp_streamer_cls.return_value
        underlying.start_webrtc.return_value = False
        streamer = VPStreamer("192.0.2.1", record=False)

        self.assertFalse(streamer.start_video_stream(1280, 720, 15, 9999))
        self.assertFalse(
            streamer.update_video_frame(
                np.zeros((720, 1280, 3), dtype=np.uint8)
            )
        )
        underlying.update_frame.assert_not_called()


class ControllerCCTVFrameSinkTest(unittest.TestCase):
    @staticmethod
    def _controller(show_camera_streams, sink):
        controller = RobotControllerMuJoCoPegTool.__new__(
            RobotControllerMuJoCoPegTool
        )
        controller.show_camera_streams = show_camera_streams
        controller.cctv_frame_sink = sink
        controller.cctv_frame_sink_error_count = 0
        controller._last_cctv_frame_sink_error_log_time = 0.0
        controller.cctv_camera = "cctv_cam"
        controller.separate_cctv_window = True
        controller.cctv_window_name = "CCTV Camera"
        controller.show_cctv_in_combined_panel = False
        controller.monitor_camera_names = ["cctv_cam"]
        controller.force_feedback_config = SimpleNamespace(
            enabled=False,
            display_mode="off",
        )
        controller.hdf5_recorder = None
        controller._get_force_feedback_snapshot = lambda render_data: None
        controller._ensure_cctv_window = lambda: None
        return controller

    def test_local_window_and_video_sink_receive_same_composed_frame(self):
        received = []
        controller = self._controller(True, received.append)
        composed = np.full((720, 1280, 3), 7, dtype=np.uint8)
        controller._render_cctv_window_bgr = Mock(return_value=composed)

        with patch(
            "arm_control.robot_controller_mujoco_peg_tool_contact.cv2.imshow"
        ) as imshow, patch(
            "arm_control.robot_controller_mujoco_peg_tool_contact.cv2.waitKey"
        ):
            controller.update_camera_stream_windows({}, render_data=None)

        self.assertIs(received[0], composed)
        self.assertIs(imshow.call_args.args[1], composed)

    def test_video_sink_still_runs_when_local_windows_are_disabled(self):
        received = []
        controller = self._controller(False, received.append)
        composed = np.full((720, 1280, 3), 9, dtype=np.uint8)
        controller._render_cctv_window_bgr = Mock(return_value=composed)

        with patch(
            "arm_control.robot_controller_mujoco_peg_tool_contact.cv2.imshow"
        ) as imshow, patch(
            "arm_control.robot_controller_mujoco_peg_tool_contact.cv2.waitKey"
        ) as wait_key:
            controller.update_camera_stream_windows({}, render_data=None)

        self.assertEqual(received, [composed])
        imshow.assert_not_called()
        wait_key.assert_not_called()
        self.assertTrue(controller._render_pipeline_enabled())

    def test_video_sink_exception_is_isolated(self):
        def failing_sink(frame):
            raise RuntimeError("transport down")

        controller = self._controller(False, failing_sink)
        controller._publish_cctv_frame(
            np.zeros((720, 1280, 3), dtype=np.uint8)
        )
        self.assertEqual(controller.cctv_frame_sink_error_count, 1)


class TeleopSystemVideoLifecycleTest(unittest.TestCase):
    def test_failed_optional_video_start_detaches_sink(self):
        system = TeleopSystemMujoco(
            {
                "visionpro_video_enabled": True,
                "visionpro_video_port": 9999,
                "cctv_window_width": 1280,
                "cctv_window_height": 720,
                "camera_stream_fps": 15.0,
            }
        )
        system.vp_streamer = Mock()
        system.vp_streamer.start_video_stream.return_value = False
        system.robot_controller = Mock()
        system.robot_controller.set_cctv_frame_sink.return_value = True

        system._initialize_visionpro_video()

        sink_calls = system.robot_controller.set_cctv_frame_sink.call_args_list
        self.assertEqual(len(sink_calls), 2)
        self.assertIsNone(sink_calls[-1].args[0])

    def test_stop_closes_visionpro_after_rendering_stops(self):
        events = []
        system = TeleopSystemMujoco({})
        system.end_effector = SimpleNamespace(stop=lambda: events.append("hand"))
        system.arm_teleop = SimpleNamespace(stop=lambda: events.append("arm"))
        system.robot_controller = SimpleNamespace(
            disconnect=lambda: events.append("simulation")
        )
        system.vp_streamer = SimpleNamespace(close=lambda: events.append("vp"))

        system.stop()

        self.assertEqual(events, ["hand", "arm", "simulation", "vp"])


if __name__ == "__main__":
    unittest.main()
