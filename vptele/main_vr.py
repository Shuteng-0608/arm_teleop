#!/usr/bin/env python3
"""Pure-Python Vision Pro teleoperation and manual HDF5 collection entry."""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path
from typing import Optional, Sequence


if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from vptele.utils.logger import get_logger, setup_logger
from vptele.utils.mujoco_config import (
    MujocoConfigError,
    load_mujoco_config,
    validate_mujoco_config,
)


DEFAULT_CONFIG = (
    Path(__file__).resolve().parent
    / "config"
    / "config_arm_right_moving_hole.yaml"
)


def build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="ROS-free Vision Pro -> MuJoCo teleoperation and recording"
    )
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--vp-ip", default=None, help="override Vision Pro IP")
    parser.add_argument(
        "--synthetic",
        action="store_true",
        help="use an official-shape synthetic tracking stream instead of hardware",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="disable the MuJoCo viewer and OpenCV camera windows",
    )
    parser.add_argument("--label", default="teleop", help="episode label")
    parser.add_argument(
        "--log-level",
        choices=("debug", "info", "warning", "error", "critical"),
        default=None,
    )
    return parser


def build_vr_config(options: argparse.Namespace):
    """Load a profile and enforce a ROS-free human teleoperation runtime."""
    config = load_mujoco_config(options.config)
    if options.vp_ip:
        config["vp_ip"] = options.vp_ip
    config.update(
        {
            "vp_enabled": True,
            "enable_ros_interfaces": False,
            "enable_recording_service": False,
            "hdf5_auto_start": False,
            "launch_viewer": not bool(options.headless),
            "show_camera_streams": not bool(options.headless),
        }
    )
    if options.synthetic:
        config["visionpro_video_enabled"] = False
    arm_config = dict(config.get("arm_config") or {})
    arm_config["enable_episode_services"] = False
    config["arm_config"] = arm_config
    validate_mujoco_config(config)
    return config


def _print_controls() -> None:
    print(
        "\nCommands:\n"
        "  r / Enter  start a calibrated recording episode\n"
        "  k          stop and keep the episode (or accept auto-completed)\n"
        "  d          stop and discard the episode\n"
        "  c          recalibrate the current wrist pose\n"
        "  q          quit safely\n"
    )


def run_interactive(options: argparse.Namespace) -> int:
    config = build_vr_config(options)
    log_config = config.get("logging", {})
    setup_logger(
        options.log_level or log_config.get("console_level", "info"),
        log_config.get("file_level", "debug"),
        log_config.get("max_file_size", 100 * 1024 * 1024),
        log_config.get("backup_count", 5),
        prefix="mujoco_vr",
    )

    streamer = None
    if options.synthetic:
        from vptele.core.synthetic_vp_streamer import SyntheticVPStreamer

        streamer = SyntheticVPStreamer()
        get_logger().warning(
            "Synthetic Vision Pro tracking is active; use it only for commissioning"
        )

    from vptele.core.mujoco_vr_teleop_system import MujocoVRTeleopSystem

    system = MujocoVRTeleopSystem(config, vp_streamer=streamer)
    stopping = False

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        system.initialize()
        _print_controls()
        while not stopping:
            try:
                command = input("vr-teleop> ").strip().lower()
            except EOFError:
                break
            if command in {"q", "quit", "exit"}:
                break
            if command in {"", "r", "record", "start"}:
                outcome = system.start_episode(options.label)
                print(outcome.message)
            elif command in {"k", "keep"}:
                outcome = system.stop_episode(keep=True)
                print(outcome.message)
            elif command in {"d", "discard"}:
                outcome = system.stop_episode(keep=False)
                print(outcome.message)
            elif command in {"c", "calibrate"}:
                print("Calibration succeeded" if system.recalibrate() else "Calibration failed")
            else:
                _print_controls()
    finally:
        system.stop()
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_cli_parser()
    options = parser.parse_args(argv)
    try:
        return run_interactive(options)
    except MujocoConfigError as exc:
        parser.error(str(exc))
    except Exception:
        get_logger().exception("ROS-free Vision Pro teleoperation failed")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
