#!/usr/bin/env python3
"""Command-line entry point for ROS-free automatic MuJoCo collection."""

from __future__ import annotations

import argparse
import signal
import threading
from pathlib import Path
from typing import Optional, Sequence

from vptele.utils.logger import get_logger, setup_logger
from vptele.utils.mujoco_config import (
    MujocoConfigError,
    apply_runtime_overrides,
    load_mujoco_config,
    validate_mujoco_config,
)


DEFAULT_CONFIG = Path(__file__).resolve().parent / "config" / "config_arm_right_peg.yaml"


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ROS-free automatic MuJoCo peg-in-hole data collection",
    )
    parser.add_argument(
        "--target-episodes",
        type=int,
        required=True,
        help="number of accepted episodes to retain before exiting",
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG),
        help="MuJoCo YAML config path",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=None,
        help="attempt safety limit; omit or use 0 for target-episodes * 5",
    )
    parser.add_argument(
        "--reject-action",
        choices=("quarantine", "delete"),
        default=None,
        help="handling for episodes rejected by the quality gate",
    )
    parser.add_argument(
        "--show-ui",
        action="store_true",
        help="show the passive viewer and camera windows (headless by default)",
    )
    parser.add_argument(
        "--log-level",
        choices=("debug", "info", "warning", "error", "critical"),
        default=None,
    )
    return parser


def build_standalone_config(options: argparse.Namespace):
    config = load_mujoco_config(options.config)
    config = apply_runtime_overrides(
        config,
        review_mode="auto",
        target_episodes=options.target_episodes,
        max_attempts=options.max_attempts,
        reject_action=options.reject_action,
    )
    config.update(
        {
            "enable_ros_interfaces": False,
            "enable_recording_service": False,
            "vp_enabled": False,
            "visionpro_video_enabled": False,
            "launch_viewer": bool(options.show_ui),
            "show_camera_streams": bool(options.show_ui),
        }
    )
    validate_mujoco_config(config)
    return config


def main(argv: Optional[Sequence[str]] = None) -> int:
    options = build_cli_parser().parse_args(argv)
    try:
        config = build_standalone_config(options)
    except MujocoConfigError as exc:
        build_cli_parser().error(str(exc))

    log_config = config.get("logging", {})
    logger = setup_logger(
        options.log_level or log_config.get("console_level", "info"),
        log_config.get("file_level", "debug"),
        log_config.get("max_file_size", 100 * 1024 * 1024),
        log_config.get("backup_count", 5),
        prefix=log_config.get("log_prefix", "mujoco_auto"),
    )

    stop_event = threading.Event()

    def request_stop(signum, _frame):
        logger.warning("Received signal %s; stopping after current operation", signum)
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    # Importing the standalone system never imports rospy. Keep this import
    # after argument/config handling so --help remains fast and side-effect free.
    from vptele.core.mujoco_collection_system import MujocoAutomaticCollectionSystem

    system = MujocoAutomaticCollectionSystem(config, stop_event=stop_event)
    completed = False
    try:
        system.initialize()
        completed = system.run()
    finally:
        system.stop()

    logger = get_logger()
    logger.info(
        "Automatic collection finished: completed=%s stats=%s manifest=%s",
        completed,
        system.batch_stats,
        system.manifest_path,
    )
    return 0 if completed else 2


if __name__ == "__main__":
    raise SystemExit(main())
