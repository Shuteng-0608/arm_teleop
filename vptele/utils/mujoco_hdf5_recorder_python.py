#!/usr/bin/env python3
"""Pure-Python episode control for the compact MuJoCo HDF5 recorder.

The compact :mod:`mujoco_hdf5_recorder` implementation is already independent
of ROS.  This module replaces the ROS ``SetRecording`` service with a small
localhost JSON/TCP service and coordinates the controller and teleoperation
lifecycle around every episode.
"""

from __future__ import annotations

import json
import shutil
import socket
import socketserver
import threading
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Optional

from vptele.utils.mujoco_hdf5_recorder import (
    ImageCaptureRequest,
    MujocoHDF5Recorder,
)


DEFAULT_RECORDING_HOST = "127.0.0.1"
DEFAULT_RECORDING_PORT = 8765


@dataclass(frozen=True)
class RecordingResult:
    success: bool
    active: bool
    pending_review: bool
    message: str
    episode_path: str = ""
    hole_cell: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class PurePythonEpisodeManager:
    """Coordinate reset, calibration, teleoperation and HDF5 episodes.

    This mirrors the established ROS recording-service ordering while calling
    Python objects directly.  The controller remains the owner of physics,
    rendering, task-success detection and hole scheduling.  Its existing
    ``MujocoHDF5Recorder`` remains the owner of HDF5 I/O.
    """

    def __init__(self, controller, teleop) -> None:
        self.controller = controller
        self.teleop = teleop
        self.recorder: Optional[MujocoHDF5Recorder] = getattr(
            controller, "hdf5_recorder", None
        )
        if self.recorder is None:
            raise RuntimeError(
                "HDF5 recorder is unavailable; set record_hdf5=true"
            )
        self._operation_lock = threading.RLock()

    def _hole_cell(self, context=None) -> str:
        sample = context or getattr(
            self.controller, "current_hole_sample", {}
        )
        return str(sample.get("hole_grid_cell_label", ""))

    def status(self) -> RecordingResult:
        with self._operation_lock:
            active = bool(getattr(self.recorder, "active", False))
            pending = bool(
                getattr(
                    self.controller,
                    "pending_auto_completed_review",
                    False,
                )
            )
            path = ""
            if pending:
                path = str(
                    getattr(
                        self.controller,
                        "pending_auto_completed_episode_path",
                        "",
                    )
                )
            elif getattr(self.recorder, "hdf5_path", None) is not None:
                path = str(self.recorder.hdf5_path)
            async_error = getattr(self.recorder, "async_error", None)
            message = "recording" if active else "idle"
            if pending:
                message = "episode completed; waiting for keep/discard review"
            if async_error:
                message += f"; recorder error: {async_error}"
            return RecordingResult(
                success=async_error is None,
                active=active,
                pending_review=pending,
                message=message,
                episode_path=path,
                hole_cell=self._hole_cell(),
            )

    def start_episode(self, label: str = "teleop") -> RecordingResult:
        with self._operation_lock:
            if bool(getattr(self.recorder, "active", False)):
                result = self.status()
                return RecordingResult(
                    False,
                    True,
                    result.pending_review,
                    "Recording is already active.",
                    result.episode_path,
                    result.hole_cell,
                )
            if bool(
                getattr(
                    self.controller,
                    "pending_auto_completed_review",
                    False,
                )
            ):
                result = self.status()
                return RecordingResult(
                    False,
                    False,
                    True,
                    "Review the auto-completed episode before starting another.",
                    result.episode_path,
                    result.hole_cell,
                )

            safe_label = str(label).strip() or "teleop"
            self.controller.accept_teleop_commands = False
            hdf5_path = None
            try:
                # Stop first so no stale IK command can race with reset.
                self.teleop.stop()

                if getattr(
                    self.controller, "reset_arm_on_record_start", True
                ):
                    self.controller.reset_arm_to_initial_pose()
                if getattr(
                    self.controller,
                    "reset_joint_torque_alarm_on_record_start",
                    True,
                ):
                    self.controller.reset_joint_torque_alarm()
                if getattr(
                    self.controller,
                    "reset_ft_wrench_alarm_on_record_start",
                    True,
                ):
                    self.controller.reset_ft_wrench_alarm()
                with self.controller.lock:
                    self.controller._reset_task_success_state_locked()

                # Calibration must observe the post-reset MuJoCo flange pose.
                self.teleop.recalibrate_for_new_episode()
                with self.controller.lock:
                    hdf5_path = self.recorder.start_episode(
                        label=safe_label,
                        episode_metadata=self.controller.current_hole_sample,
                    )

                self.controller.accept_teleop_commands = True
                self.teleop.start()
                return RecordingResult(
                    True,
                    True,
                    False,
                    f"Started HDF5 recording and teleoperation: {safe_label}",
                    str(hdf5_path) if hdf5_path is not None else "",
                    self._hole_cell(),
                )
            except Exception as exc:
                self.controller.accept_teleop_commands = False
                try:
                    self.teleop.stop()
                except Exception:
                    pass
                if bool(getattr(self.recorder, "active", False)):
                    with self.controller.lock:
                        self.recorder.stop_episode(status="teleop_start_failed")
                return RecordingResult(
                    False,
                    False,
                    False,
                    f"Failed to start episode: {exc}",
                    str(hdf5_path) if hdf5_path is not None else "",
                    self._hole_cell(),
                )

    def _remove_episode_dir(self, episode_path: str) -> None:
        if not episode_path:
            return
        hdf5_path = Path(episode_path).expanduser().resolve()
        directory = hdf5_path.parent
        output_root = Path(self.recorder.output_dir).expanduser().resolve()
        if hdf5_path.name != "episode.hdf5" or directory.parent != output_root:
            raise RuntimeError(
                "Refusing to delete an episode outside the configured output "
                f"directory: {directory}"
            )
        if directory.exists():
            shutil.rmtree(directory)

    def _finish_hole(self, keep: bool):
        return self.controller.finalize_hole_after_episode(keep=bool(keep))

    def stop_episode(self, keep: bool = True) -> RecordingResult:
        with self._operation_lock:
            pending = bool(
                getattr(
                    self.controller,
                    "pending_auto_completed_review",
                    False,
                )
            )
            if pending:
                episode_path = str(
                    getattr(
                        self.controller,
                        "pending_auto_completed_episode_path",
                        "",
                    )
                )
                try:
                    self.controller.accept_teleop_commands = False
                    self.teleop.stop()
                    self.controller._reset_after_episode_stop()
                    next_hole = self._finish_hole(keep)
                    if not keep:
                        self._remove_episode_dir(episode_path)
                    action = "kept" if keep else "discarded"
                    return RecordingResult(
                        True,
                        False,
                        False,
                        f"Auto-completed episode {action} after review.",
                        episode_path,
                        self._hole_cell(next_hole),
                    )
                except Exception as exc:
                    return RecordingResult(
                        False,
                        False,
                        True,
                        f"Failed to review auto-completed episode: {exc}",
                        episode_path,
                        self._hole_cell(),
                    )

            if not bool(getattr(self.recorder, "active", False)):
                result = self.status()
                return RecordingResult(
                    False,
                    False,
                    False,
                    "No active recording episode.",
                    result.episode_path,
                    result.hole_cell,
                )

            self.controller.accept_teleop_commands = False
            self.teleop.stop()
            before_stop = getattr(self.recorder, "hdf5_path", None)
            try:
                with self.controller.lock:
                    stopped_path = self.recorder.stop_episode(
                        status="manual_keep" if keep else "manual_discard"
                    )
                hdf5_path = stopped_path or before_stop
                episode_path = str(hdf5_path) if hdf5_path is not None else ""

                # Reset only after HDF5 is inactive so reset is never recorded.
                self.controller._reset_after_episode_stop()
                next_hole = self._finish_hole(keep)
                if not keep:
                    self._remove_episode_dir(episode_path)
                action = "kept" if keep else "discarded"
                return RecordingResult(
                    True,
                    False,
                    False,
                    f"Stopped recording; episode {action}.",
                    episode_path,
                    self._hole_cell(next_hole),
                )
            except Exception as exc:
                return RecordingResult(
                    False,
                    bool(getattr(self.recorder, "active", False)),
                    False,
                    f"Failed to stop episode: {exc}",
                    str(before_stop) if before_stop is not None else "",
                    self._hole_cell(),
                )

    def handle_command(self, request: Dict[str, Any]) -> RecordingResult:
        command = str(request.get("command", "status")).strip().lower()
        if command == "status":
            return self.status()
        if command == "start":
            return self.start_episode(str(request.get("label", "teleop")))
        if command == "stop":
            return self.stop_episode(bool(request.get("keep", True)))
        return RecordingResult(
            False,
            bool(getattr(self.recorder, "active", False)),
            bool(
                getattr(
                    self.controller,
                    "pending_auto_completed_review",
                    False,
                )
            ),
            f"Unknown command: {command}",
            hole_cell=self._hole_cell(),
        )


class _RecordingRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        try:
            line = self.rfile.readline(1024 * 1024)
            if not line:
                return
            request = json.loads(line.decode("utf-8"))
            if not isinstance(request, dict):
                raise ValueError("request must be a JSON object")
            result = self.server.manager.handle_command(request)
            payload = result.to_dict()
        except Exception as exc:
            payload = RecordingResult(
                False,
                False,
                False,
                f"Invalid recording request: {exc}",
            ).to_dict()
        self.wfile.write(
            (json.dumps(payload, ensure_ascii=False) + "\n").encode("utf-8")
        )


class _ThreadingRecordingServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True


class PurePythonRecordingServer:
    """Expose ``PurePythonEpisodeManager`` on a local JSON/TCP endpoint."""

    def __init__(
        self,
        manager: PurePythonEpisodeManager,
        host: str = DEFAULT_RECORDING_HOST,
        port: int = DEFAULT_RECORDING_PORT,
    ) -> None:
        if not 0 <= int(port) <= 65535:
            raise ValueError(
                "recording server port must be between 0 and 65535"
            )
        self.manager = manager
        self.server = _ThreadingRecordingServer(
            (str(host), int(port)), _RecordingRequestHandler
        )
        self.server.manager = manager
        self.thread: Optional[threading.Thread] = None

    @property
    def address(self):
        return self.server.server_address

    def start(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            return
        self.thread = threading.Thread(
            target=self.server.serve_forever,
            name="PurePythonRecordingServer",
            daemon=True,
        )
        self.thread.start()

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        if self.thread is not None:
            self.thread.join(timeout=2.0)
        self.thread = None


def send_recording_command(
    request: Dict[str, Any],
    host: str = DEFAULT_RECORDING_HOST,
    port: int = DEFAULT_RECORDING_PORT,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    """Send one request to the pure-Python recording server."""
    payload = (json.dumps(request, ensure_ascii=False) + "\n").encode("utf-8")
    with socket.create_connection((str(host), int(port)), timeout=timeout) as sock:
        sock.sendall(payload)
        response = sock.makefile("rb").readline(1024 * 1024)
    if not response:
        raise ConnectionError("recording server closed without a response")
    result = json.loads(response.decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("recording server returned a non-object response")
    return result


__all__ = [
    "DEFAULT_RECORDING_HOST",
    "DEFAULT_RECORDING_PORT",
    "ImageCaptureRequest",
    "MujocoHDF5Recorder",
    "PurePythonEpisodeManager",
    "PurePythonRecordingServer",
    "RecordingResult",
    "send_recording_command",
]
