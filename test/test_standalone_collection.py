#!/usr/bin/env python3

import argparse
import subprocess
import sys
import tempfile
import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from vptele.arm_control.scripted_collection import (
    EpisodeRunOutcome,
    ScriptedInsertionRunner,
)
from vptele.main_scripted import DEFAULT_CONFIG, build_standalone_config


REPO_ROOT = Path(__file__).resolve().parents[1]
MOVING_HOLE_CONFIG = (
    REPO_ROOT / "vptele" / "config" / "config_arm_right_moving_hole.yaml"
)


class StandaloneCollectionTest(unittest.TestCase):
    def test_standalone_overrides_disable_ros_and_operator_interfaces(self):
        options = argparse.Namespace(
            config=str(DEFAULT_CONFIG),
            target_episodes=3,
            max_attempts=8,
            reject_action="quarantine",
            show_ui=False,
        )
        config = build_standalone_config(options)

        self.assertFalse(config["enable_ros_interfaces"])
        self.assertFalse(config["enable_recording_service"])
        self.assertFalse(config["vp_enabled"])
        self.assertFalse(config["visionpro_video_enabled"])
        self.assertFalse(config["launch_viewer"])
        self.assertFalse(config["show_camera_streams"])
        self.assertTrue(config["scripted_controller"]["enabled"])
        self.assertEqual(config["scripted_controller"]["review_mode"], "auto")
        self.assertEqual(config["scripted_controller"]["target_episodes"], 3)

    def test_scripted_entry_enables_safely_disabled_moving_hole_profile(self):
        options = argparse.Namespace(
            config=str(MOVING_HOLE_CONFIG),
            target_episodes=2,
            max_attempts=4,
            reject_action="quarantine",
            show_ui=True,
            log_level=None,
        )
        config = build_standalone_config(options)

        self.assertTrue(config["scripted_controller"]["enabled"])
        self.assertEqual(config["scripted_controller"]["review_mode"], "auto")
        self.assertEqual(config["scripted_controller"]["target_episodes"], 2)
        self.assertFalse(config["enable_ros_interfaces"])
        self.assertFalse(config["enable_recording_service"])
        self.assertFalse(config["vp_enabled"])
        self.assertFalse(config["visionpro_video_enabled"])
        self.assertTrue(config["launch_viewer"])
        self.assertTrue(config["show_camera_streams"])

    def test_automatic_runner_finishes_without_ros_shutdown_state(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            robot_controller = SimpleNamespace(
                arm_sign=[1] * 7,
                hdf5_recorder=SimpleNamespace(output_dir=temp_dir),
                task_success_auto_stop_recording=True,
            )
            runner = ScriptedInsertionRunner(
                robot_controller=robot_controller,
                config={
                    "review_mode": "auto",
                    "target_episodes": 2,
                    "max_attempts": 2,
                    "max_consecutive_rejections": 2,
                    "hdf5_record_dir": temp_dir,
                },
            )
            runner._run_one_episode = lambda: EpisodeRunOutcome(
                success=True,
                kept=True,
                message="accepted",
            )

            self.assertTrue(runner.start_automatic_batch())
            self.assertTrue(runner.batch_finished_event.wait(timeout=2.0))
            self.assertTrue(runner.batch_completed)
            self.assertEqual(
                runner.batch_stats,
                {"attempted": 2, "kept": 2, "rejected": 0},
            )

    def test_episode_metadata_uses_current_wall_threshold(self):
        class FakeRecorder:
            def __init__(self, output_dir):
                self.output_dir = output_dir
                self.hdf5_path = Path(output_dir) / "episode.hdf5"
                self.episode_metadata = None

            def start_episode(self, label, episode_metadata):
                del label
                self.episode_metadata = dict(episode_metadata)
                return self.hdf5_path

            def add_event(self, *_args, **_kwargs):
                return None

            def stop_episode(self, status):
                del status
                return self.hdf5_path

        class FakeScriptedController:
            def __init__(self):
                self.wall_threshold = 0.0
                self.run_wall_threshold = None

            def _init_dofs(self):
                return None

            def _sample_error(self):
                return None

            def _sample_wall_threshold(self):
                self.wall_threshold = 23.5

            def get_last_error_info(self):
                return {
                    "scripted_error_xy_mm": 10.0,
                    "scripted_error_angle_deg": 45.0,
                    "scripted_wall_threshold_n": self.wall_threshold,
                }

            def run_episode(
                self,
                error_xy_mm,
                error_angle_deg,
                wall_threshold_n,
            ):
                del error_xy_mm, error_angle_deg
                self.run_wall_threshold = wall_threshold_n
                return SimpleNamespace(
                    success=True,
                    outcome="success",
                    retry_count=0,
                    duration_s=1.0,
                )

        with tempfile.TemporaryDirectory() as temp_dir:
            recorder = FakeRecorder(temp_dir)
            robot_controller = SimpleNamespace(
                arm_sign=[1] * 7,
                hdf5_recorder=recorder,
                task_success_auto_stop_recording=True,
                task_success_triggered=True,
                task_success_terminal_hold_time=0.0,
                accept_teleop_commands=False,
                reset_arm_to_initial_pose=lambda: None,
            )
            runner = ScriptedInsertionRunner(
                robot_controller=robot_controller,
                config={
                    "review_mode": "manual",
                    "hdf5_record_dir": temp_dir,
                    "collection_mode": "direct",
                    "manual_review_after_episode": False,
                    "reset_arm_on_stop": False,
                    "reset_ignore_teleop_duration": 0.0,
                },
            )
            fake_controller = FakeScriptedController()
            runner.controller = fake_controller
            runner._set_fixed_hole_center = lambda: None

            with patch("vptele.arm_control.scripted_collection.time.sleep"):
                outcome = runner._run_one_episode()

            self.assertTrue(outcome.success)
            self.assertEqual(
                recorder.episode_metadata["scripted_wall_threshold_n"],
                23.5,
            )
            self.assertEqual(fake_controller.run_wall_threshold, 23.5)

    def test_standalone_import_chain_does_not_require_rospy(self):
        code = """
import sys
sys.modules['rospy'] = None
import vptele.main_scripted
import vptele.arm_control.robot_controller_mujoco_peg_tool_contact
import vptele.arm_control.scripted_collection
import vptele.core.mujoco_collection_system
print('ok')
"""
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True,
            timeout=20,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "ok")

    def test_legacy_collect_module_reexports_new_entry(self):
        from vptele import collect_mujoco, main_scripted

        self.assertIs(collect_mujoco.main, main_scripted.main)
        self.assertIs(
            collect_mujoco.build_standalone_config,
            main_scripted.build_standalone_config,
        )

    def test_two_stage_replay_never_overwrites_terminal_hold(self):
        class FakeRecorder:
            def __init__(self):
                self.events = []

            def add_event(self, name, metadata=None):
                self.events.append((name, metadata))

        applied = []
        recorder = FakeRecorder()
        robot_controller = SimpleNamespace(
            arm_sign=[1] * 7,
            hdf5_recorder=recorder,
            lock=threading.RLock(),
            task_success_triggered=True,
            terminal_hold_active=True,
            target_joints=[0.0] * 7,
            command_joints=[0.0] * 7,
            _apply_actuator_targets=lambda command: applied.append(list(command)),
        )
        runner = ScriptedInsertionRunner(
            robot_controller=robot_controller,
            config={"hdf5_record_dir": tempfile.gettempdir()},
        )

        replay_ok = runner._replay_command_trace(
            np.ones((3, 7)),
            replay_hz=1000.0,
        )

        self.assertTrue(replay_ok)
        self.assertEqual(applied, [])
        self.assertEqual(
            recorder.events,
            [("scripted_replay_stopped_on_task_success", {"trace_index": 0})],
        )


if __name__ == "__main__":
    unittest.main()
