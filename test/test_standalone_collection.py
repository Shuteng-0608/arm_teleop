#!/usr/bin/env python3

import argparse
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

from vptele.arm_control.scripted_collection import (
    EpisodeRunOutcome,
    ScriptedInsertionRunner,
)
from vptele.collect_mujoco import DEFAULT_CONFIG, build_standalone_config


REPO_ROOT = Path(__file__).resolve().parents[1]


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
        self.assertEqual(config["scripted_controller"]["review_mode"], "auto")
        self.assertEqual(config["scripted_controller"]["target_episodes"], 3)

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

    def test_standalone_import_chain_does_not_require_rospy(self):
        code = """
import sys
sys.modules['rospy'] = None
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


if __name__ == "__main__":
    unittest.main()
