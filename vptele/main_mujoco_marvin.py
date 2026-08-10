#!/usr/bin/env python3
"""Python-only Vision Pro teleoperation entry for the Marvin M6 MuJoCo model."""

from __future__ import annotations

import argparse
from copy import deepcopy
import importlib.util
from pathlib import Path
import sys
import time
from typing import Optional, Sequence


VPTELE_DIR = Path(__file__).resolve().parent
REPO_ROOT = VPTELE_DIR.parent
for import_root in (REPO_ROOT, VPTELE_DIR):
    if str(import_root) not in sys.path:
        sys.path.insert(0, str(import_root))

from arm_control.marvin_ik_solver import preflight_marvin_ik_assets
from utils.logger import get_logger, setup_logger
from utils.mujoco_config import (
    apply_runtime_overrides,
    build_arm_teleop_config,
    build_controller_config,
    load_mujoco_config,
    validate_mujoco_config,
)


DEFAULT_CONFIG_PATH = VPTELE_DIR / "config" / "config_arm_right_peg_marvin.yaml"


def select_marvin_ik_library_path(arm_config, platform_name=None):
    """Select the vendor kinematics library for the active operating system."""
    platform_name = sys.platform if platform_name is None else str(platform_name)
    platform_key = (
        "marvin_ik_library_path_windows"
        if platform_name == "win32"
        else "marvin_ik_library_path_linux"
    )
    selected = arm_config.get(platform_key) or arm_config.get(
        "marvin_ik_library_path"
    )
    if not selected:
        raise ValueError(
            f"arm_config.{platform_key} or arm_config.marvin_ik_library_path "
            "must be configured"
        )
    return str(selected)


def preflight_python_dependencies() -> None:
    """Report all non-ROS runtime packages missing from the active Python."""
    required_modules = {
        "mujoco": "mujoco",
        "cv2": "opencv-python",
        "h5py": "h5py",
        "avp_stream": "avp-stream",
    }
    missing = [
        package
        for module, package in required_modules.items()
        if importlib.util.find_spec(module) is None
    ]
    if missing:
        raise RuntimeError(
            "Missing Python runtime dependencies / 当前 Python 缺少运行依赖: "
            + ", ".join(missing)
        )


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run Marvin M6 Vision Pro teleoperation with Python, without "
            "roscore or ROS IK services."
        )
    )
    parser.add_argument(
        "--config",
        default=str(DEFAULT_CONFIG_PATH),
        help="Path to the Marvin MuJoCo YAML config.",
    )
    parser.add_argument(
        "--vp-ip",
        default=None,
        help="Override the Vision Pro IP from YAML.",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Automatically record one HDF5 episode for the whole run.",
    )
    parser.add_argument(
        "--label",
        default="marvin_python",
        help="HDF5 episode label used with --record.",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Open the interactive MuJoCo viewer in addition to camera windows.",
    )
    parser.add_argument(
        "--no-camera-windows",
        action="store_true",
        help="Disable local OpenCV camera windows.",
    )
    parser.add_argument(
        "--no-video-return",
        action="store_true",
        help="Disable the CCTV WebRTC return stream to Vision Pro.",
    )
    parser.add_argument(
        "--wait-timeout",
        type=float,
        default=None,
        help="Seconds to wait for a valid right-wrist transform.",
    )
    parser.add_argument(
        "--record-host",
        default="127.0.0.1",
        help="Host for the pure-Python recording control server.",
    )
    parser.add_argument(
        "--record-port",
        type=int,
        default=8765,
        help="Port for the pure-Python recording control server.",
    )
    parser.add_argument(
        "--no-record-server",
        action="store_true",
        help="Disable the pure-Python recording control server.",
    )
    return parser.parse_args(argv)


def build_python_only_config(
    config,
    *,
    vp_ip=None,
    record=False,
    label="marvin_python",
    viewer=False,
    camera_windows=True,
    video_return=True,
):
    """Return a standalone profile with every ROS service dependency disabled."""
    result = apply_runtime_overrides(deepcopy(config), vp_ip=vp_ip)
    result["teleop_controlled_by_recording"] = False
    result["accept_teleop_when_not_recording"] = True
    result["enable_recording_service"] = False

    arm_config = dict(result.get("arm_config") or {})
    arm_config["ik_backend"] = "marvin_local"
    arm_config["enable_episode_services"] = False
    arm_config["marvin_ik_library_path"] = select_marvin_ik_library_path(
        arm_config
    )
    result["arm_config"] = arm_config

    if viewer:
        result["launch_viewer"] = True
    if not camera_windows:
        result["show_camera_streams"] = False
    if not video_return:
        result["visionpro_video_enabled"] = False

    if record:
        result["record_hdf5"] = True
        result["hdf5_auto_start"] = True
        result["hdf5_episode_label"] = str(label).strip() or "marvin_python"
    return result


def _wait_for_right_wrist(streamer, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = streamer.get_hand_position(hand="right")
        if value is not None:
            try:
                shape = tuple(value.shape)
            except AttributeError:
                try:
                    import numpy as np

                    shape = tuple(np.asarray(value).shape)
                except Exception:
                    shape = ()
            if shape == (4, 4) or (
                len(shape) == 3 and shape[0] > 0 and shape[1:] == (4, 4)
            ):
                return
        time.sleep(0.05)
    raise TimeoutError(
        f"等待 Vision Pro 右手腕数据超时（{timeout:.1f} 秒）"
    )


def _start_video_return(config, streamer, controller, logger) -> None:
    if not bool(config.get("visionpro_video_enabled", False)):
        return
    if not controller.set_cctv_frame_sink(streamer.update_video_frame):
        logger.warning("未找到 CCTV 相机，Vision Pro 视频回传已跳过")
        return
    started = streamer.start_video_stream(
        width=config.get("cctv_window_width", 1280),
        height=config.get("cctv_window_height", 720),
        fps=config.get("camera_stream_fps", 15.0),
        port=config.get("visionpro_video_port", 9999),
    )
    if not started:
        controller.set_cctv_frame_sink(None)
        logger.warning("Vision Pro 视频回传启动失败；遥操和本地仿真继续运行")


def run_python_only(args: argparse.Namespace) -> None:
    config_path = Path(args.config).expanduser().resolve()
    config = load_mujoco_config(str(config_path))
    config = build_python_only_config(
        config,
        vp_ip=args.vp_ip,
        record=args.record,
        label=args.label,
        viewer=args.viewer,
        camera_windows=not args.no_camera_windows,
        video_return=not args.no_video_return,
    )
    validate_mujoco_config(config)

    arm_config = config["arm_config"]
    preflight_marvin_ik_assets(
        arm_config["marvin_ik_module_path"],
        arm_config["marvin_ik_library_path"],
        arm_config["marvin_ik_config_path"],
    )
    preflight_python_dependencies()

    log_config = config.get("logging", {})
    setup_logger(
        log_config.get("console_level", "info"),
        log_config.get("file_level", "debug"),
        log_config.get("max_file_size", 100 * 1024 * 1024),
        log_config.get("backup_count", 5),
        prefix=log_config.get("log_prefix", "mujoco_marvin_python"),
    )
    logger = get_logger()

    # Delayed imports keep configuration and IK preflight errors independent
    # from MuJoCo/OpenGL/avp-stream availability.
    from arm_control.arm_teleop_mujoco_marvin import ArmTeleopMujocoMarvin
    from arm_control.robot_controller_mujoco_peg_tool_contact import (
        RobotControllerMuJoCoPegTool,
    )
    from core.vp_streamer_avp import VPStreamer

    streamer = None
    controller = None
    teleop = None
    recording_server = None
    try:
        logger.info("连接 Vision Pro: %s", config["vp_ip"])
        streamer = VPStreamer(
            ip=config["vp_ip"],
            record=bool(config.get("vp_record", False)),
        )
        wait_timeout = (
            float(args.wait_timeout)
            if args.wait_timeout is not None
            else float(config.get("vp_ready_timeout", 10.0))
        )
        if wait_timeout <= 0.0:
            raise ValueError("--wait-timeout 必须大于 0")
        _wait_for_right_wrist(streamer, wait_timeout)

        controller = RobotControllerMuJoCoPegTool(
            config["mujoco_model_path"],
            build_controller_config(config),
        )
        teleop = ArmTeleopMujocoMarvin(
            streamer,
            controller,
            build_arm_teleop_config(config),
        )
        _start_video_return(config, streamer, controller, logger)

        # Prepare the first hole and start physics/rendering before exposing
        # episode controls, so every accepted start request has valid context.
        controller.activate_runtime()

        if not args.no_record_server:
            from utils.mujoco_hdf5_recorder_python import (
                PurePythonEpisodeManager,
                PurePythonRecordingServer,
            )

            episode_manager = PurePythonEpisodeManager(controller, teleop)
            recording_server = PurePythonRecordingServer(
                episode_manager,
                host=args.record_host,
                port=args.record_port,
            )
            recording_server.start()
            logger.info(
                "Pure-Python recording server ready at %s:%s",
                args.record_host,
                args.record_port,
            )

        teleop.start()
        logger.info(
            "Python-only Marvin 遥操已启动；无需 roscore/IK service，按 Ctrl+C 退出"
        )
        while controller.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("收到 Ctrl+C，正在关闭 Python-only Marvin 仿真")
    finally:
        if recording_server is not None:
            recording_server.close()
        if teleop is not None:
            teleop.stop()
        if controller is not None:
            controller.disconnect()
        if streamer is not None:
            streamer.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        run_python_only(args)
        return 0
    except Exception as exc:
        print(f"Marvin Python-only 遥操启动失败: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
