#!/usr/bin/env python3
"""ROS-free entry point for automatic scripted MuJoCo data collection.

This module deliberately owns only the scripted simulation workflow.  It does
not initialize ROS, expose ROS services, connect to Vision Pro, or import the
teleoperation system.  The existing controller, recorder, randomization,
two-stage replay, quality gate, quarantine/delete policy, batch manifest, and
optional local visualization are reused unchanged.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
from pathlib import Path
from typing import Optional, Sequence


# Support both of the intended launch forms:
#   python -m vptele.main_scripted
#   python vptele/main_scripted.py
if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vptele.utils.logger import get_logger, setup_logger
from vptele.utils.mujoco_config import (
    MujocoConfigError,
    apply_runtime_overrides,
    load_mujoco_config,
    validate_mujoco_config,
)


DEFAULT_CONFIG = (
    Path(__file__).resolve().parent / "config" / "config_arm_right_peg.yaml"
)


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ROS-free automatic scripted MuJoCo data collection",
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
        "--scenario",
        choices=("collision", "clean"),
        default=None,
        help=(
            "collection scenario; collision keeps the configured rim/in-hole "
            "contacts, clean uses zero offset and disables deliberate contact"
        ),
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
    """Load one profile and enforce the ROS-free automatic runtime contract."""
    config = load_mujoco_config(options.config)
    config = apply_runtime_overrides(
        config,
        review_mode="auto",
        target_episodes=options.target_episodes,
        max_attempts=options.max_attempts,
        reject_action=options.reject_action,
    )

    # These values are entry-point invariants, rather than user-selectable
    # options.  In particular, enabling the scripted controller here allows a
    # profile to remain safely disabled for interactive main_mujoco.py runs.
    scripted_config = dict(config.get("scripted_controller") or {})
    scripted_config["enabled"] = True
    scripted_config.setdefault("scenario", "collision")
    scenario = getattr(options, "scenario", None)
    if scenario is not None:
        scripted_config["scenario"] = str(scenario).strip().lower()
    config["scripted_controller"] = scripted_config
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


def run_scripted_collection(
    options: argparse.Namespace,
    *,
    stop_event: Optional[threading.Event] = None,
) -> bool:
    """Run one retained-count batch and always release simulation resources."""
    config = build_standalone_config(options)
    log_config = config.get("logging", {})
    logger = setup_logger(
        getattr(options, "log_level", None)
        or log_config.get("console_level", "info"),
        log_config.get("file_level", "debug"),
        log_config.get("max_file_size", 100 * 1024 * 1024),
        log_config.get("backup_count", 5),
        prefix=log_config.get("log_prefix", "mujoco_scripted"),
    )

    from vptele.core.mujoco_collection_system import MujocoAutomaticCollectionSystem

    system = MujocoAutomaticCollectionSystem(
        config,
        stop_event=stop_event or threading.Event(),
    )
    completed = False
    try:
        system.initialize()
        completed = system.run()
    finally:
        system.stop()

    logger = get_logger()
    logger.info(
        "Automatic scripted collection finished: completed=%s stats=%s manifest=%s",
        completed,
        system.batch_stats,
        system.manifest_path,
    )
    return completed


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_cli_parser()
    options = parser.parse_args(argv)
    stop_event = threading.Event()

    def request_stop(signum, _frame):
        get_logger().warning(
            "Received signal %s; stopping the scripted batch safely", signum
        )
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        completed = run_scripted_collection(options, stop_event=stop_event)
    except MujocoConfigError as exc:
        parser.error(str(exc))
    except Exception:
        get_logger().exception("Automatic scripted collection failed")
        return 1
    return 0 if completed else 2


if __name__ == "__main__":
    raise SystemExit(main())
