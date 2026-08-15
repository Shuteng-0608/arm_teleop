#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Compact MuJoCo HDF5 recorder for peg-in-hole teleoperation.

Records only core data:
  state stream: ee_pose, joint_pos, joint_vel, joint_torque
  force stream: ft_wrench
  image stream: RGB frame tensors stored directly inside HDF5
  episode_metadata: rates, names, initial/final poses and task setup information

The legacy episode API remains compatible. ``record_if_needed`` now returns an
optional ImageCaptureRequest; a render worker fulfills it with
``enqueue_image_sample`` while numeric samples are written asynchronously.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional

import h5py
import mujoco
import numpy as np


@dataclass(frozen=True)
class ImageCaptureRequest:
    """A timestamped request for an asynchronously rendered image sample."""

    generation: int
    t_sim: float
    t_episode: float


@dataclass(frozen=True)
class _WritePacket:
    kind: str
    generation: int
    payload: Any


class MujocoHDF5Recorder:
    # Small in-memory batches remove per-row h5py overhead while keeping the
    # public API, sampling cadence, dataset layout and dtypes unchanged.
    _FORCE_WRITE_BATCH_SIZE = 64
    _STATE_WRITE_BATCH_SIZE = 16

    def __init__(
        self,
        model,
        data,
        output_dir: str,
        model_path: str = "",
        force_hz: float = 500.0,
        state_hz: float = 30.0,
        image_hz: float = 30.0,
        record_images: bool = True,
        camera_names: Optional[List[str]] = None,
        image_width: int = 640,
        image_height: int = 480,
        image_format: str = "hdf5_rgb",   # compatibility only; images are stored in HDF5
        jpg_quality: int = 90,            # compatibility only; unused
        max_buffer_rows: int = 500000,

        enable_ft_tare: bool = True,
        # record_ft_wrench_raw: bool = True,

        ft_compensation_mode: str = "gravity",
        record_ft_wrench_raw: bool = True,
        record_ft_wrench_gravity: bool = True,

        record_actions: bool = True,
        record_action_alias: bool = True,

        ft_gravity_tool_body_names: Optional[List[str]] = None,
        ft_gravity_world: Optional[List[float]] = None,
        ft_gravity_sensor_sign: float = -1.0,

        joint_names: Optional[List[str]] = None,
        actuator_names: Optional[List[str]] = None,
        ee_body_name: str = "peg_tool",
        task_name: str = "wall_peg_in_hole",
        ft_force_sensor_name: str = "peg_ft_force",
        ft_torque_sensor_name: str = "peg_ft_torque",
        peg_geom_name: str = "cylindrical_peg",
        peg_tip_site_name: str = "peg_tip_site",
        hole_center_site_name: str = "hole_goal_site",
        hole_goal_site_name: Optional[str] = None,
        sampled_target_site_name: Optional[str] = None,
        hole_ring_geom_prefix: str = "wall_hole_ring_",
        hole_axis_body: Optional[List[float]] = None,
        image_compression: Optional[str] = "lzf",   # "lzf", "gzip", or None
        image_compression_level: int = 1,
        numeric_compression: Optional[str] = None,
        chunk_size_state: int = 256,
        chunk_size_force: int = 2048,
        chunk_size_image: int = 1,
        append_block_image: int = 16,
        async_queue_size: int = 1024,
        async_stop_timeout: float = 10.0,
    ):
        self.model = model
        self.data = data
        self.model_path = model_path
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.force_hz = float(force_hz)
        self.state_hz = float(state_hz)
        self.image_hz = float(image_hz)
        self.force_period = 1.0 / max(self.force_hz, 1e-9)
        self.state_period = 1.0 / max(self.state_hz, 1e-9)
        self.image_period = 1.0 / max(self.image_hz, 1e-9)

        self.record_images = bool(record_images)
        self.camera_names = list(camera_names or ["ee_cam", "base_top_cam"])
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.max_buffer_rows = int(max_buffer_rows)

        self.enable_ft_tare = bool(enable_ft_tare)
        # self.record_ft_wrench_raw = bool(record_ft_wrench_raw)
        self.ft_wrench_bias_raw = np.zeros(6, dtype=np.float64)

        self.ft_compensation_mode = str(ft_compensation_mode).lower()
        self.record_ft_wrench_raw = bool(record_ft_wrench_raw)
        self.record_ft_wrench_gravity = bool(record_ft_wrench_gravity)

        self.ft_gravity_tool_body_names = list(
            ft_gravity_tool_body_names or ["peg_tool"]
        )

        self.ft_gravity_world = np.asarray(
            ft_gravity_world if ft_gravity_world is not None else [0.0, 0.0, -9.81],
            dtype=np.float64,
        )

        self.ft_gravity_sensor_sign = float(ft_gravity_sensor_sign)

        self.record_actions = bool(record_actions)
        self.record_action_alias = bool(record_action_alias)


        self.joint_names = list(joint_names or [f"joint_{i}" for i in range(1, 8)])
        self.actuator_names = list(actuator_names or [f"motor_joint_{i}" for i in range(1, 8)])
        self.ee_body_name = ee_body_name
        self.task_name = str(task_name)
        self.ft_force_sensor_name = ft_force_sensor_name
        self.ft_torque_sensor_name = ft_torque_sensor_name
        self.peg_geom_name = peg_geom_name
        self.peg_tip_site_name = peg_tip_site_name
        self.hole_goal_site_name = hole_goal_site_name or hole_center_site_name
        self.sampled_target_site_name = (
            sampled_target_site_name or self.hole_goal_site_name
        )
        # Compatibility alias for existing HDF5 field names and callers.
        self.hole_center_site_name = self.hole_goal_site_name
        self.hole_ring_geom_prefix = str(hole_ring_geom_prefix)
        self.hole_axis_body = np.asarray(
            hole_axis_body if hole_axis_body is not None else [0.0, -1.0, 0.0],
            dtype=np.float64,
        )

        self.image_compression = image_compression
        self.image_compression_level = int(image_compression_level)
        self.numeric_compression = numeric_compression
        self.chunk_size_state = int(chunk_size_state)
        self.chunk_size_force = int(chunk_size_force)
        self.chunk_size_image = int(chunk_size_image)
        self.append_block_image = max(1, int(append_block_image))
        self.async_queue_size = max(1, int(async_queue_size))
        self.async_stop_timeout = max(0.1, float(async_stop_timeout))

        self.active = False
        self._state_lock = threading.RLock()
        self._pending_image_condition = threading.Condition(self._state_lock)
        self._io_lock = threading.RLock()
        self._generation = 0
        self._pending_image_requests = 0
        self._dropped_numeric_samples = 0
        self._dropped_image_samples = 0
        self._async_error: Optional[str] = None
        self._write_queue: queue.Queue = queue.Queue(
            maxsize=self.async_queue_size
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="MujocoHDF5Writer",
            daemon=True,
        )
        self.session_dir: Optional[Path] = None
        self.hdf5_path: Optional[Path] = None
        self.h5: Optional[h5py.File] = None

        self.episode_label = ""
        self.episode_context: Dict[str, Any] = {}
        self.episode_start_sim_time = 0.0
        self.episode_start_wall_time = 0.0
        self.next_state_t = 0.0
        self.next_force_t = 0.0
        self.next_image_t = 0.0
        self.n_state = 0
        self.n_force = 0
        self.n_image = 0
        self.event_rows: List[Dict[str, Any]] = []

        self.joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ]
        self.actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.actuator_names
        ]
        self.ee_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, self.ee_body_name)

        self.ft_force_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.ft_force_sensor_name)
        self.ft_torque_sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, self.ft_torque_sensor_name)
        # The force sensor object is the site used by the sensor.
        self.ft_sensor_site_id = -1

        if self.ft_force_sensor_id != -1:
            self.ft_sensor_site_id = int(
                self.model.sensor_objid[self.ft_force_sensor_id]
            )
        elif self.ft_torque_sensor_id != -1:
            self.ft_sensor_site_id = int(
                self.model.sensor_objid[self.ft_torque_sensor_id]
            )

        self.ft_gravity_tool_body_ids = []
        for body_name in self.ft_gravity_tool_body_names:
            bid = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_name,
            )

            if bid == -1:
                print(
                    f"[CompactHDF5Recorder] Warning: gravity compensation body "
                    f"not found: {body_name}"
                )
            else:
                self.ft_gravity_tool_body_ids.append(bid)

        if self.ft_compensation_mode == "gravity":
            print("[CompactHDF5Recorder] FT compensation mode: gravity")
            print(f"  tool bodies = {self.ft_gravity_tool_body_names}")
            print(f"  gravity     = {self.ft_gravity_world.tolist()}")
            print(f"  sign        = {self.ft_gravity_sensor_sign}")


        self.peg_geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            self.peg_geom_name,
        )
        self.peg_tip_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.peg_tip_site_name)
        self.hole_goal_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            self.hole_goal_site_name,
        )
        self.hole_center_site_id = self.hole_goal_site_id
        self.sampled_target_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            self.sampled_target_site_name,
        )
        self.task_geometry_metadata = self._infer_task_geometry_metadata()

        self.camera_ids: Dict[str, int] = {}
        if self.record_images:
            for cam_name in self.camera_names:
                cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, cam_name)
                if cam_id == -1:
                    print(f"[CompactHDF5Recorder] Warning: camera not found: {cam_name}")
                else:
                    self.camera_ids[cam_name] = cam_id
            if not self.camera_ids:
                self.record_images = False
                print("[CompactHDF5Recorder] No valid camera found; image recording disabled.")

        print("[CompactHDF5Recorder] Ready.")
        print(f"  output_dir    = {self.output_dir}")
        print(f"  state_hz      = {self.state_hz}")
        print(f"  force_hz      = {self.force_hz}")
        print(f"  image_hz      = {self.image_hz}")
        print(f"  record_images = {self.record_images}")
        if self.record_images:
            print(f"  cameras       = {list(self.camera_ids.keys())}")
        self._writer_thread.start()

    @property
    def async_error(self) -> Optional[str]:
        with self._state_lock:
            return self._async_error

    def _writer_loop(self) -> None:
        """Own all steady-state HDF5 appends outside the physics thread."""
        force_samples: List[Dict[str, Any]] = []
        state_samples: List[Dict[str, Any]] = []

        while True:
            packet = self._write_queue.get()

            if packet.kind == "close":
                try:
                    self._flush_numeric_batches(
                        force_samples,
                        state_samples,
                        packet.generation,
                    )
                except Exception as exc:
                    self._record_async_error(exc)
                finally:
                    self._write_queue.task_done()
                return

            if packet.kind == "barrier":
                try:
                    self._flush_numeric_batches(
                        force_samples,
                        state_samples,
                        packet.generation,
                    )
                except Exception as exc:
                    self._record_async_error(exc)
                finally:
                    packet.payload.set()
                    self._write_queue.task_done()
                continue

            try:
                with self._io_lock:
                    packet_is_current = bool(
                        self.h5 is not None
                        and packet.generation == self._generation
                    )
                if not packet_is_current:
                    continue

                if packet.kind == "force":
                    force_samples.append(packet.payload)
                elif packet.kind == "state":
                    state_samples.append(packet.payload)
                elif packet.kind == "image":
                    with self._io_lock:
                        if (
                            self.h5 is not None
                            and packet.generation == self._generation
                        ):
                            self._append_image_packet(packet.payload)
                else:
                    raise ValueError(f"Unknown recorder packet: {packet.kind}")

                if (
                    len(force_samples) >= self._FORCE_WRITE_BATCH_SIZE
                    or len(state_samples) >= self._STATE_WRITE_BATCH_SIZE
                ):
                    self._flush_numeric_batches(
                        force_samples,
                        state_samples,
                        packet.generation,
                    )
            except Exception as exc:
                self._record_async_error(exc)
            finally:
                self._write_queue.task_done()

    def _record_async_error(self, exc: Exception) -> None:
        message = f"{type(exc).__name__}: {exc}"
        with self._state_lock:
            first_error = self._async_error is None
            if first_error:
                self._async_error = message
        if first_error:
            print(f"[CompactHDF5Recorder] Async writer failed: {message}")

    def _flush_numeric_batches(
        self,
        force_buffer: List[Dict[str, Any]],
        state_buffer: List[Dict[str, Any]],
        generation: int,
    ) -> None:
        """Write buffered numeric rows with one slice assignment per dataset."""
        if not force_buffer and not state_buffer:
            return

        force_samples = list(force_buffer)
        state_samples = list(state_buffer)
        force_buffer.clear()
        state_buffer.clear()

        with self._io_lock:
            if self.h5 is None or generation != self._generation:
                return
            if force_samples:
                self._append_force_batch(force_samples)
            if state_samples:
                self._append_state_batch(state_samples)

    def _enqueue_packet_locked(self, packet: _WritePacket) -> bool:
        """Non-blocking enqueue; caller holds ``_state_lock``."""
        try:
            self._write_queue.put_nowait(packet)
            return True
        except queue.Full:
            if packet.kind == "image":
                self._dropped_image_samples += 1
            else:
                self._dropped_numeric_samples += 1
            if self._async_error is None:
                self._async_error = (
                    f"async write queue full (capacity={self.async_queue_size})"
                )
                print(f"[CompactHDF5Recorder] {self._async_error}")
            return False

    def _flush_writer(self, generation: int) -> bool:
        barrier = threading.Event()
        try:
            self._write_queue.put(
                _WritePacket("barrier", generation, barrier),
                timeout=self.async_stop_timeout,
            )
        except queue.Full:
            return False
        return barrier.wait(timeout=self.async_stop_timeout)

    def _infer_task_geometry_metadata(self) -> Dict[str, Any]:
        """Read peg/hole dimensions from the compiled MuJoCo model.

        The hole is represented by box segments rather than a cylinder. For
        each segment, project its half extents onto the radial and insertion
        directions. The median inner radius and depth are recorded so metadata
        follows the actual XML geometry instead of duplicated constants.
        """
        if self.peg_geom_id == -1:
            raise ValueError(f"Peg geom not found: {self.peg_geom_name}")
        if int(self.model.geom_type[self.peg_geom_id]) != int(
            mujoco.mjtGeom.mjGEOM_CYLINDER
        ):
            raise ValueError(f"Peg geom must be a cylinder: {self.peg_geom_name}")
        if self.peg_tip_site_id == -1:
            raise ValueError(f"Peg tip site not found: {self.peg_tip_site_name}")
        if self.hole_goal_site_id == -1:
            raise ValueError(f"Hole goal site not found: {self.hole_goal_site_name}")
        if self.sampled_target_site_id == -1:
            raise ValueError(
                f"Sampled target site not found: {self.sampled_target_site_name}"
            )

        axis = np.asarray(self.hole_axis_body, dtype=np.float64)
        if axis.shape != (3,) or not np.all(np.isfinite(axis)):
            raise ValueError("hole_axis_body must be a finite 3-vector")
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm <= 1e-12:
            raise ValueError("hole_axis_body must be non-zero")
        axis = axis / axis_norm
        self.hole_axis_body = axis

        ring_ids = []
        ring_names = []
        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                geom_id,
            )
            if geom_name and geom_name.startswith(self.hole_ring_geom_prefix):
                ring_ids.append(geom_id)
                ring_names.append(geom_name)

        if not ring_ids:
            raise ValueError(
                f"No hole ring geoms match prefix: {self.hole_ring_geom_prefix}"
            )

        radii = []
        depths = []
        for geom_id in ring_ids:
            if int(self.model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
                raise ValueError("Hole ring geoms must be boxes")

            center = np.asarray(self.model.geom_pos[geom_id], dtype=np.float64)
            radial_center = center - float(np.dot(center, axis)) * axis
            center_radius = float(np.linalg.norm(radial_center))
            if center_radius <= 1e-12:
                raise ValueError("Hole ring geom center cannot lie on the hole axis")
            radial_direction = radial_center / center_radius

            rotation_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(rotation_flat, self.model.geom_quat[geom_id])
            rotation = rotation_flat.reshape(3, 3)
            half_sizes = np.asarray(self.model.geom_size[geom_id], dtype=np.float64)

            half_radial_extent = float(
                np.sum(np.abs(rotation.T @ radial_direction) * half_sizes)
            )
            half_depth_extent = float(
                np.sum(np.abs(rotation.T @ axis) * half_sizes)
            )
            radii.append(center_radius - half_radial_extent)
            depths.append(2.0 * half_depth_extent)

        hole_radius = float(np.median(radii))
        hole_depth = float(np.median(depths))
        if hole_radius <= 0.0 or hole_depth <= 0.0:
            raise ValueError(
                f"Invalid inferred hole geometry: radius={hole_radius}, depth={hole_depth}"
            )

        return {
            "task_geometry_length_unit": "m",
            "task_position_frame": "world",
            "peg_geom_name": self.peg_geom_name,
            "peg_radius_m": float(self.model.geom_size[self.peg_geom_id, 0]),
            "peg_length_m": float(2.0 * self.model.geom_size[self.peg_geom_id, 1]),
            "peg_tip_site_name": self.peg_tip_site_name,
            "hole_goal_site_name": self.hole_goal_site_name,
            "sampled_target_site_name": self.sampled_target_site_name,
            "hole_ring_geom_prefix": self.hole_ring_geom_prefix,
            "hole_ring_segment_count": len(ring_ids),
            "hole_ring_geom_names": ring_names,
            "hole_axis_body": axis.tolist(),
            "hole_radius_m": hole_radius,
            "hole_depth_m": hole_depth,
            "hole_radius_segment_std_m": float(np.std(radii)),
            "hole_depth_segment_std_m": float(np.std(depths)),
        }

    # ------------------------------------------------------------------
    # Public lifecycle API
    # ------------------------------------------------------------------
    def start_episode(
        self,
        label: str = "teleop",
        episode_metadata: Optional[Dict[str, Any]] = None,
    ) -> Optional[Path]:
        with self._state_lock:
            if self.active:
                print("[CompactHDF5Recorder] Episode already active.")
                return self.hdf5_path

        with self._io_lock:
            if self.h5 is not None:
                raise RuntimeError("Previous HDF5 episode is still open")

            safe_label = self._safe_name(label)
            timestamp = time.strftime("%Y%m%d_%H%M%S")

            self.session_dir = self.output_dir / f"{timestamp}_{safe_label}"
            self.session_dir.mkdir(parents=True, exist_ok=True)
            self.hdf5_path = self.session_dir / "episode.hdf5"

            self.episode_label = safe_label
            self.episode_context = dict(self.task_geometry_metadata)
            self.episode_context.update(episode_metadata or {})

            peg_tip_world = self._site_pos(self.peg_tip_site_id)
            hole_goal_world = self._site_pos(self.hole_goal_site_id)
            sampled_target_world = self._site_pos(self.sampled_target_site_id)
            hole_offset = np.asarray(
                self.episode_context.get(
                    "target_offset_from_nominal_xyz",
                    self.episode_context.get(
                        "hole_offset_from_nominal_xyz",
                        self.episode_context.get(
                            "hole_actual_offset_xyz",
                            [0.0, 0.0, 0.0],
                        ),
                    ),
                ),
                dtype=np.float64,
            )
            if hole_offset.shape != (3,) or not np.all(np.isfinite(hole_offset)):
                raise ValueError(f"Invalid episode hole offset: {hole_offset}")
            self.episode_context.update(
                {
                    "peg_tip_site_initial_pos_world": peg_tip_world.tolist(),
                    "hole_goal_site_initial_pos_world": hole_goal_world.tolist(),
                    "hole_goal_site_nominal_pos_world": (
                        hole_goal_world - hole_offset
                        if self.sampled_target_site_id == self.hole_goal_site_id
                        else hole_goal_world
                    ).tolist(),
                    "sampled_target_site_initial_pos_world": (
                        sampled_target_world.tolist()
                    ),
                    "sampled_target_site_nominal_pos_world": (
                        sampled_target_world - hole_offset
                    ).tolist(),
                    "hole_actual_offset_xyz": hole_offset.tolist(),
                    "hole_offset_from_nominal_xyz": hole_offset.tolist(),
                }
            )
            self.episode_start_sim_time = float(self.data.time)
            self.episode_start_wall_time = time.time()

            self.next_state_t = float(self.data.time)
            self.next_force_t = float(self.data.time)
            self.next_image_t = float(self.data.time)

            self.n_state = 0
            self.n_force = 0
            self.n_image = 0
            self.event_rows = []

            # self.h5 = h5py.File(self.hdf5_path, "w")
            # self._create_file_structure()
            # self._write_initial_metadata()

            # self.active = True
            # self.add_event("record_start")
            # 每条 episode 开始时，读取当前六维力作为 tare bias。
            # 注意：这里发生在 reset arm / randomize hole 之后、teleop 开始之前。
            if self.enable_ft_tare:
                self.ft_wrench_bias_raw = self._ft_wrench_raw()
            else:
                self.ft_wrench_bias_raw = np.zeros(6, dtype=np.float64)

            self.h5 = h5py.File(self.hdf5_path, "w")
            self._create_file_structure()
            self._write_initial_metadata()

            with self._state_lock:
                self._generation += 1
                self._pending_image_requests = 0
                self._dropped_numeric_samples = 0
                self._dropped_image_samples = 0
                self._async_error = None
                self.active = True

        self.add_event("record_start")

        print(f"[CompactHDF5Recorder] Started: {self.session_dir}")
        return self.hdf5_path

    def stop_episode(self, status: str = "manual_stop") -> Optional[Path]:
        with self._pending_image_condition:
            if not self.active:
                print("[CompactHDF5Recorder] No active episode.")
                return self.hdf5_path

            self.add_event(status)
            self.active = False
            generation = self._generation

            deadline = time.monotonic() + self.async_stop_timeout
            while self._pending_image_requests > 0:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self._dropped_image_samples += self._pending_image_requests
                    self._pending_image_requests = 0
                    if self._async_error is None:
                        self._async_error = "timed out waiting for rendered images"
                    break
                self._pending_image_condition.wait(timeout=remaining)

        if not self._flush_writer(generation):
            with self._state_lock:
                if self._async_error is None:
                    self._async_error = "timed out flushing async writer"

        with self._io_lock:
            self._write_final_metadata(status=status)
            self._write_events()

            if self.h5 is not None:
                self.h5.flush()
                self.h5.close()
                self.h5 = None

            self._write_sidecar_json(status=status)

            print(f"[CompactHDF5Recorder] Saved: {self.hdf5_path}")
            return self.hdf5_path

    def close(self) -> None:
        if self.active:
            self.stop_episode(status="controller_shutdown")
        elif self.h5 is not None:
            # Recover a file left open after an asynchronous capture failure.
            self._flush_writer(self._generation)
            with self._io_lock:
                self._write_final_metadata(status="async_capture_error")
                self._write_events()
                self.h5.flush()
                self.h5.close()
                self.h5 = None
                self._write_sidecar_json(status="async_capture_error")

        if self._writer_thread.is_alive():
            try:
                self._write_queue.put(
                    _WritePacket("close", self._generation, None),
                    timeout=self.async_stop_timeout,
                )
            except queue.Full:
                pass
            self._writer_thread.join(timeout=self.async_stop_timeout)
            if self._writer_thread.is_alive():
                raise RuntimeError("HDF5 writer thread did not stop cleanly")

    def add_event(self, event: str, extra: Optional[Dict[str, Any]] = None) -> None:
        with self._state_lock:
            if not self.active and event != "record_start":
                print(f"[CompactHDF5Recorder] Ignored event without active episode: {event}")
                return

            row: Dict[str, Any] = {
                "event": str(event),
                "t_sim": float(self.data.time),
                "t_episode": float(self.data.time - self.episode_start_sim_time),
                "t_wall": float(time.time()),
                "t_wall_from_start": float(time.time() - self.episode_start_wall_time),
            }

            if extra:
                row.update(extra)

            self.event_rows.append(row)

            print(f"[CompactHDF5Recorder] Event: {event} @ {row['t_episode']:.3f}s")

    def record_if_needed(
        self,
        controller,
    ) -> Optional[ImageCaptureRequest]:
        """
        Snapshot due numeric streams immediately after ``mujoco.mj_step``.

        The caller owns the live ``MjData`` while this method runs. Only copied
        NumPy arrays are queued for the writer. Image rendering is requested
        separately so OpenGL and disk I/O never run on the physics thread.
        """
        del controller  # retained for public API compatibility
        with self._state_lock:
            if not self.active:
                return None

            t = float(self.data.time)
            generation = self._generation

            if t + 1e-12 >= self.next_force_t:
                self._enqueue_packet_locked(
                    _WritePacket(
                        "force",
                        generation,
                        self._capture_force_sample(t),
                    )
                )
                while self.next_force_t <= t + 1e-12:
                    self.next_force_t += self.force_period

            if t + 1e-12 >= self.next_state_t:
                self._enqueue_packet_locked(
                    _WritePacket(
                        "state",
                        generation,
                        self._capture_state_sample(t),
                    )
                )
                while self.next_state_t <= t + 1e-12:
                    self.next_state_t += self.state_period

            if self.record_images and t + 1e-12 >= self.next_image_t:
                while self.next_image_t <= t + 1e-12:
                    self.next_image_t += self.image_period
                self._pending_image_requests += 1
                return ImageCaptureRequest(
                    generation=generation,
                    t_sim=t,
                    t_episode=float(t - self.episode_start_sim_time),
                )

            return None

    def enqueue_image_sample(
        self,
        request: ImageCaptureRequest,
        frames: Dict[str, np.ndarray],
    ) -> bool:
        """Queue rendered RGB frames and release the pending request."""
        copied_frames = {
            name: np.ascontiguousarray(frame, dtype=np.uint8)
            for name, frame in frames.items()
        }
        with self._pending_image_condition:
            accepted = self._enqueue_packet_locked(
                _WritePacket(
                    "image",
                    request.generation,
                    {
                        "t_sim": request.t_sim,
                        "t_episode": request.t_episode,
                        "frames": copied_frames,
                    },
                )
            )
            self._pending_image_requests = max(
                0, self._pending_image_requests - 1
            )
            self._pending_image_condition.notify_all()
            return accepted

    def drop_image_request(
        self,
        request: ImageCaptureRequest,
        reason: str,
    ) -> None:
        """Release a render request that could not be fulfilled."""
        with self._pending_image_condition:
            self._dropped_image_samples += 1
            self._pending_image_requests = max(
                0, self._pending_image_requests - 1
            )
            self.event_rows.append(
                {
                    "event": "image_dropped",
                    "t_sim": request.t_sim,
                    "t_episode": request.t_episode,
                    "t_wall": float(time.time()),
                    "t_wall_from_start": float(
                        time.time() - self.episode_start_wall_time
                    ),
                    "reason": str(reason),
                }
            )
            self._pending_image_condition.notify_all()

    # ------------------------------------------------------------------
    # File structure
    # ------------------------------------------------------------------
    def _create_file_structure(self) -> None:
        assert self.h5 is not None
        f = self.h5

        f.attrs["schema_version"] = "compact_mujoco_hdf5_v1"
        f.attrs["alignment_clock"] = "mujoco data.time"
        f.attrs["image_storage"] = "hdf5_uint8_rgb_frame_tensor"
        f.attrs["episode_label"] = self.episode_label
        f.attrs["model_path"] = self.model_path
        f.attrs["mujoco_timestep"] = float(self.model.opt.timestep)
        f.attrs["state_hz"] = self.state_hz
        f.attrs["force_hz"] = self.force_hz
        f.attrs["image_hz"] = self.image_hz
        f.attrs["created_wall_time"] = time.strftime("%Y-%m-%d %H:%M:%S")
        f.attrs["schema_json"] = json.dumps(
            {
                "observations/ee_pose": "[N_state,7] x y z qw qx qy qz",
                "observations/joint_pos": "[N_state,7] joint angles",
                "observations/joint_vel": "[N_state,7] joint velocities",
                "observations/joint_torque": "[N_state,7] qfrc_actuator at joint dofs",
                # "observations/ft_wrench": "[N_force,6] Fx Fy Fz Tx Ty Tz",

                # "observations/ft_wrench": "[N_force,6] tare-compensated Fx Fy Fz Tx Ty Tz",
                # "observations/ft_wrench_raw": "[N_force,6] raw Fx Fy Fz Tx Ty Tz",

                "observations/ft_wrench": "[N_force,6] compensated Fx Fy Fz Tx Ty Tz",
                "observations/ft_wrench_raw": "[N_force,6] raw MuJoCo FT sensor reading",
                "observations/ft_wrench_gravity": "[N_force,6] predicted gravity wrench in sensor frame",

                "observations/images/<camera>": "[N_image,H,W,3] uint8 RGB",

                "actions/joint_pos_command": "[N_state,7] actual actuator position command from data.ctrl[actuator_ids]",
                "action": "[N_state,7] ACT-compatible alias of actions/joint_pos_command",

                "timebase": "MuJoCo data.time",
            },
            ensure_ascii=False,
        )

        f.require_group("episode_metadata")
        f.require_group("timestamps")
        f.require_group("observations")
        f["observations"].require_group("images")
        f.require_group("events")
        if self.record_actions:
            f.require_group("actions")

        self._create_resizable_1d("timestamps/state", dtype=np.float64, chunk=self.chunk_size_state)
        self._create_resizable_1d("timestamps/state_episode", dtype=np.float64, chunk=self.chunk_size_state)
        self._create_resizable_1d("timestamps/force", dtype=np.float64, chunk=self.chunk_size_force)
        self._create_resizable_1d("timestamps/force_episode", dtype=np.float64, chunk=self.chunk_size_force)
        self._create_resizable_1d("timestamps/image", dtype=np.float64, chunk=max(1, self.chunk_size_image))
        self._create_resizable_1d("timestamps/image_episode", dtype=np.float64, chunk=max(1, self.chunk_size_image))

        self._create_resizable_2d("observations/ee_pose", width=7, chunk=self.chunk_size_state)
        self._create_resizable_2d("observations/joint_pos", width=len(self.joint_names), chunk=self.chunk_size_state)
        self._create_resizable_2d("observations/joint_vel", width=len(self.joint_names), chunk=self.chunk_size_state)
        self._create_resizable_2d("observations/joint_torque", width=len(self.joint_names), chunk=self.chunk_size_state)

        if self.record_actions:
            self._create_resizable_2d(
                "actions/joint_pos_command",
                width=len(self.actuator_names),
                chunk=self.chunk_size_state,
            )

            if self.record_action_alias:
                self._create_resizable_2d(
                    "action",
                    width=len(self.actuator_names),
                    chunk=self.chunk_size_state,
                )
        # self._create_resizable_2d("observations/ft_wrench", width=6, chunk=self.chunk_size_force)
        # compensated wrench, used by default for learning
        # self._create_resizable_2d(
        #     "observations/ft_wrench",
        #     width=6,
        #     chunk=self.chunk_size_force,
        # )

        # # raw wrench, optional but strongly recommended for debugging
        # if self.record_ft_wrench_raw:
        #     self._create_resizable_2d(
        #         "observations/ft_wrench_raw",
        #         width=6,
        #         chunk=self.chunk_size_force,
        #     )
        # Default compensated wrench used for learning.
        self._create_resizable_2d(
            "observations/ft_wrench",
            width=6,
            chunk=self.chunk_size_force,
        )

        # Raw MuJoCo FT sensor reading.
        if self.record_ft_wrench_raw:
            self._create_resizable_2d(
                "observations/ft_wrench_raw",
                width=6,
                chunk=self.chunk_size_force,
            )

        # Predicted gravity wrench.
        if self.record_ft_wrench_gravity:
            self._create_resizable_2d(
                "observations/ft_wrench_gravity",
                width=6,
                chunk=self.chunk_size_force,
            )

        if self.record_images:
            str_dtype = h5py.string_dtype(encoding="utf-8")
            camera_names = np.asarray(list(self.camera_ids.keys()), dtype=object)
            f["observations/images"].create_dataset("camera_names", data=camera_names, dtype=str_dtype)
            for cam_name in self.camera_ids.keys():
                self._create_image_dataset(f"observations/images/{cam_name}")

    def _create_resizable_1d(self, path: str, dtype, chunk: int) -> None:
        assert self.h5 is not None
        self.h5.create_dataset(
            path,
            shape=(0,),
            maxshape=(None,),
            dtype=dtype,
            chunks=(max(1, int(chunk)),),
            compression=self.numeric_compression,
        )

    def _create_resizable_2d(self, path: str, width: int, chunk: int) -> None:
        assert self.h5 is not None
        self.h5.create_dataset(
            path,
            shape=(0, width),
            maxshape=(None, width),
            dtype=np.float64,
            chunks=(max(1, int(chunk)), width),
            compression=self.numeric_compression,
        )

    def _create_image_dataset(self, path: str) -> None:
        assert self.h5 is not None
        kwargs = {}
        if self.image_compression == "gzip":
            kwargs["compression"] = "gzip"
            kwargs["compression_opts"] = self.image_compression_level
        elif self.image_compression == "lzf":
            kwargs["compression"] = "lzf"
        elif self.image_compression is None:
            pass
        else:
            raise ValueError("image_compression must be 'lzf', 'gzip', or None")

        self.h5.create_dataset(
            path,
            shape=(0, self.image_height, self.image_width, 3),
            maxshape=(None, self.image_height, self.image_width, 3),
            dtype=np.uint8,
            chunks=(max(1, self.chunk_size_image), self.image_height, self.image_width, 3),
            **kwargs,
        )

    # ------------------------------------------------------------------
    # Append samples
    # ------------------------------------------------------------------
    def _capture_force_sample(self, t_sim: float) -> Dict[str, Any]:
        raw = self._ft_wrench_raw()
        gravity = self._ft_gravity_wrench()
        compensated = self._compensate_ft_wrench(raw, gravity)
        return {
            "t_sim": float(t_sim),
            "t_episode": float(t_sim - self.episode_start_sim_time),
            "raw": raw.copy(),
            "gravity": gravity.copy(),
            "compensated": compensated.copy(),
        }

    def _append_force_packet(self, sample: Dict[str, Any]) -> None:
        """Compatibility wrapper for callers that append one force row."""
        self._append_force_batch([sample])

    def _append_force_batch(self, samples: List[Dict[str, Any]]) -> None:
        if not samples:
            return

        start = self.n_force
        self._append_1d_batch(
            "timestamps/force",
            start,
            [sample["t_sim"] for sample in samples],
        )
        self._append_1d_batch(
            "timestamps/force_episode",
            start,
            [sample["t_episode"] for sample in samples],
        )

        if self.record_ft_wrench_raw:
            self._append_2d_batch(
                "observations/ft_wrench_raw",
                start,
                [sample["raw"] for sample in samples],
            )

        if self.record_ft_wrench_gravity:
            self._append_2d_batch(
                "observations/ft_wrench_gravity",
                start,
                [sample["gravity"] for sample in samples],
            )

        self._append_2d_batch(
            "observations/ft_wrench",
            start,
            [sample["compensated"] for sample in samples],
        )
        self.n_force += len(samples)

    def _capture_state_sample(self, t_sim: float) -> Dict[str, Any]:
        sample = {
            "t_sim": float(t_sim),
            "t_episode": float(t_sim - self.episode_start_sim_time),
            "ee_pose": self._ee_pose().copy(),
            "joint_pos": self._joint_pos().copy(),
            "joint_vel": self._joint_vel().copy(),
            "joint_torque": self._joint_torque().copy(),
        }
        if self.record_actions:
            sample["joint_pos_command"] = self._joint_pos_command().copy()
        return sample

    def _append_state_packet(self, sample: Dict[str, Any]) -> None:
        """Compatibility wrapper for callers that append one state row."""
        self._append_state_batch([sample])

    def _append_state_batch(self, samples: List[Dict[str, Any]]) -> None:
        if not samples:
            return

        start = self.n_state
        self._append_1d_batch(
            "timestamps/state",
            start,
            [sample["t_sim"] for sample in samples],
        )
        self._append_1d_batch(
            "timestamps/state_episode",
            start,
            [sample["t_episode"] for sample in samples],
        )

        for path, field in (
            ("observations/ee_pose", "ee_pose"),
            ("observations/joint_pos", "joint_pos"),
            ("observations/joint_vel", "joint_vel"),
            ("observations/joint_torque", "joint_torque"),
        ):
            self._append_2d_batch(
                path,
                start,
                [sample[field] for sample in samples],
            )

        if self.record_actions:
            commands = [sample["joint_pos_command"] for sample in samples]
            self._append_2d_batch(
                "actions/joint_pos_command",
                start,
                commands,
            )
            if self.record_action_alias:
                self._append_2d_batch("action", start, commands)

        self.n_state += len(samples)

    def _append_image_packet(self, sample: Dict[str, Any]) -> None:
        i = self.n_image
        self._append_1d("timestamps/image", i, sample["t_sim"])
        self._append_1d("timestamps/image_episode", i, sample["t_episode"])

        frames = sample["frames"]
        missing = set(self.camera_ids) - set(frames)
        if missing:
            raise ValueError(f"Missing rendered cameras: {sorted(missing)}")
        for cam_name in self.camera_ids:
            frame = frames[cam_name]
            expected_shape = (self.image_height, self.image_width, 3)
            if frame.shape != expected_shape:
                raise ValueError(
                    f"Camera {cam_name} shape {frame.shape}, expected {expected_shape}"
                )
            self._append_image(f"observations/images/{cam_name}", i, frame)

        self.n_image += 1

    def _append_1d(self, path: str, i: int, value: float) -> None:
        self._append_1d_batch(path, i, [value])

    def _append_1d_batch(
        self,
        path: str,
        start: int,
        values,
    ) -> None:
        assert self.h5 is not None
        d = self.h5[path]
        values_array = np.asarray(values, dtype=d.dtype)
        end = start + len(values_array)
        if end <= start:
            return
        if path in {"timestamps/force", "timestamps/force_episode"}:
            block = self.chunk_size_force
        elif path in {"timestamps/image", "timestamps/image_episode"}:
            block = self.append_block_image
        else:
            block = self.chunk_size_state
        if d.shape[0] < end:
            rows = (((end - 1) // block) + 1) * block
            d.resize((rows,))
        d[start:end] = values_array

    def _append_2d(self, path: str, i: int, row: np.ndarray) -> None:
        self._append_2d_batch(path, i, [row])

    def _append_2d_batch(
        self,
        path: str,
        start: int,
        rows,
    ) -> None:
        assert self.h5 is not None
        d = self.h5[path]
        rows_array = np.asarray(rows, dtype=d.dtype)
        end = start + len(rows_array)
        if end <= start:
            return
        block = (
            self.chunk_size_force
            if path.startswith("observations/ft_wrench")
            else self.chunk_size_state
        )
        if d.shape[0] < end:
            allocated_rows = (((end - 1) // block) + 1) * block
            d.resize((allocated_rows, d.shape[1]))
        d[start:end, :] = rows_array

    def _append_image(self, path: str, i: int, frame: np.ndarray) -> None:
        assert self.h5 is not None
        d = self.h5[path]
        if d.shape[0] <= i:
            rows = ((i // self.append_block_image) + 1) * self.append_block_image
            d.resize((rows, self.image_height, self.image_width, 3))
        d[i, :, :, :] = frame

    def _trim_stream_datasets(self) -> None:
        """Remove unused allocation rows before finalizing an episode."""
        assert self.h5 is not None
        for path in ("timestamps/state", "timestamps/state_episode"):
            self.h5[path].resize((self.n_state,))
        for path in ("timestamps/force", "timestamps/force_episode"):
            self.h5[path].resize((self.n_force,))
        for path in ("timestamps/image", "timestamps/image_episode"):
            self.h5[path].resize((self.n_image,))

        state_paths = [
            "observations/ee_pose",
            "observations/joint_pos",
            "observations/joint_vel",
            "observations/joint_torque",
        ]
        if self.record_actions:
            state_paths.append("actions/joint_pos_command")
            if self.record_action_alias:
                state_paths.append("action")
        for path in state_paths:
            dataset = self.h5[path]
            dataset.resize((self.n_state, dataset.shape[1]))

        force_paths = ["observations/ft_wrench"]
        if self.record_ft_wrench_raw:
            force_paths.append("observations/ft_wrench_raw")
        if self.record_ft_wrench_gravity:
            force_paths.append("observations/ft_wrench_gravity")
        for path in force_paths:
            dataset = self.h5[path]
            dataset.resize((self.n_force, dataset.shape[1]))

        if self.record_images:
            for camera_name in self.camera_ids:
                path = f"observations/images/{camera_name}"
                self.h5[path].resize(
                    (self.n_image, self.image_height, self.image_width, 3)
                )

    # ------------------------------------------------------------------
    # Metadata
    # ------------------------------------------------------------------
    def _write_initial_metadata(self) -> None:
        assert self.h5 is not None
        g = self.h5["episode_metadata"]
        g.attrs["status"] = "recording"
        g.attrs["episode_label"] = self.episode_label
        g.attrs["episode_start_sim_time"] = self.episode_start_sim_time
        g.attrs["episode_start_wall_time"] = self.episode_start_wall_time
        g.attrs["model_path"] = self.model_path
        g.attrs["mujoco_timestep"] = float(self.model.opt.timestep)
        g.attrs["state_hz"] = self.state_hz
        g.attrs["force_hz"] = self.force_hz
        g.attrs["image_hz"] = self.image_hz
        g.attrs["ee_body_name"] = self.ee_body_name
        g.attrs["ft_force_sensor_name"] = self.ft_force_sensor_name
        g.attrs["ft_torque_sensor_name"] = self.ft_torque_sensor_name
        g.attrs["peg_geom_name"] = self.peg_geom_name
        g.attrs["peg_tip_site_name"] = self.peg_tip_site_name
        g.attrs["hole_center_site_name"] = self.hole_center_site_name
        g.attrs["hole_goal_site_name"] = self.hole_goal_site_name
        g.attrs["sampled_target_site_name"] = self.sampled_target_site_name
        g.attrs["task_name"] = self.task_name
        g.attrs["task_success"] = "unknown"
        random_seed = self.episode_context.get(
            "hole_grid_seed",
            self.episode_context.get("hole_random_seed", -1),
        )
        g.attrs["random_seed"] = -1 if random_seed is None else int(random_seed)

        self._write_episode_context(g)

        self._write_string_dataset("episode_metadata/joint_names", self.joint_names)
        self._write_string_dataset("episode_metadata/actuator_names", self.actuator_names)
        self._write_string_dataset("episode_metadata/camera_names", list(self.camera_ids.keys()))
        self._write_string_dataset(
            "episode_metadata/ft_gravity_tool_body_names",
            self.ft_gravity_tool_body_names,
        )

        initial_joint_pos = self._joint_pos()
        initial_joint_vel = self._joint_vel()
        initial_joint_torque = self._joint_torque()
        initial_ee_pose = self._ee_pose()
        initial_joint_pos_command = self._joint_pos_command()

        # initial_ft = self._ft_wrench()

        # initial_ft_raw = self._ft_wrench_raw()
        # initial_ft = self._compensate_ft_wrench(initial_ft_raw)

        initial_ft_raw = self._ft_wrench_raw()
        initial_ft_gravity = self._ft_gravity_wrench()
        initial_ft = self._compensate_ft_wrench(
            initial_ft_raw,
            initial_ft_gravity,
        )

        tool_mass, tool_com_w = self._tool_mass_and_com_world()
        sensor_pos_w, R_ws = self._ft_sensor_pose_world()

        if tool_mass > 1e-12 and not np.any(np.isnan(tool_com_w)):
            tool_com_s = R_ws.T @ (tool_com_w - sensor_pos_w)
        else:
            tool_com_s = np.full(3, np.nan, dtype=np.float64)

        initial_peg_tip = self._site_pos(self.peg_tip_site_id)
        initial_hole_center = self._site_pos(self.hole_center_site_id)
        initial_err_xyz = initial_peg_tip - initial_hole_center

        g.create_dataset("initial_joint_pos", data=initial_joint_pos)
        g.create_dataset("initial_joint_vel", data=initial_joint_vel)
        g.create_dataset("initial_joint_torque", data=initial_joint_torque)
        g.create_dataset("initial_ee_pose", data=initial_ee_pose)
        if self.record_actions:
            g.create_dataset(
                "initial_joint_pos_command",
                data=initial_joint_pos_command,
            )

        # g.create_dataset("initial_ft_wrench", data=initial_ft)
        g.create_dataset("initial_ft_wrench", data=initial_ft)
        # g.create_dataset("initial_ft_wrench_raw", data=initial_ft_raw)
        # g.create_dataset("ft_wrench_bias_raw", data=self.ft_wrench_bias_raw)
        g.create_dataset("initial_ft_wrench_raw", data=initial_ft_raw)
        g.create_dataset("initial_ft_wrench_gravity", data=initial_ft_gravity)

        g.create_dataset("ft_gravity_world", data=self.ft_gravity_world)
        g.create_dataset("ft_gravity_tool_com_world_initial", data=tool_com_w)
        g.create_dataset("ft_gravity_tool_com_sensor_initial", data=tool_com_s)

        g.attrs["ft_compensation_mode"] = self.ft_compensation_mode
        g.attrs["ft_gravity_sensor_sign"] = self.ft_gravity_sensor_sign
        g.attrs["ft_gravity_tool_mass_initial"] = float(tool_mass)

        g.attrs["enable_ft_tare"] = int(self.enable_ft_tare)
        g.attrs["record_ft_wrench_raw"] = int(self.record_ft_wrench_raw)
        # g.attrs["ft_wrench_convention"] = (
        #     "observations/ft_wrench = observations/ft_wrench_raw - episode_metadata/ft_wrench_bias_raw"
        # )
        if self.ft_compensation_mode == "gravity":
            g.attrs["ft_wrench_convention"] = (
                "observations/ft_wrench = observations/ft_wrench_raw - observations/ft_wrench_gravity"
            )
        elif self.ft_compensation_mode == "none":
            g.attrs["ft_wrench_convention"] = (
                "observations/ft_wrench = observations/ft_wrench_raw"
            )
        else:
            g.attrs["ft_wrench_convention"] = (
                f"ft_compensation_mode={self.ft_compensation_mode}"
            )

        g.create_dataset("initial_peg_tip_pos", data=initial_peg_tip)
        g.create_dataset("initial_hole_center_pos", data=initial_hole_center)
        g.create_dataset("initial_hole_goal_pos", data=initial_hole_center)
        g.create_dataset("initial_task_error_xyz", data=initial_err_xyz)
        g.attrs["initial_align_err_xz"] = float(np.linalg.norm([initial_err_xyz[0], initial_err_xyz[2]]))
        g.attrs["initial_insertion_err_y"] = float(initial_err_xyz[1])

    def _write_final_metadata(self, status: str) -> None:
        assert self.h5 is not None
        self._trim_stream_datasets()
        g = self.h5["episode_metadata"]
        g.attrs["status"] = status
        g.attrs["episode_end_sim_time"] = float(self.data.time)
        g.attrs["episode_end_wall_time"] = float(time.time())
        g.attrs["duration_sim"] = float(self.data.time - self.episode_start_sim_time)
        g.attrs["duration_wall"] = float(time.time() - self.episode_start_wall_time)
        g.attrs["n_state"] = int(self.n_state)
        g.attrs["n_force"] = int(self.n_force)
        g.attrs["n_image"] = int(self.n_image)
        g.attrs["dropped_numeric_samples"] = int(
            self._dropped_numeric_samples
        )
        g.attrs["dropped_image_samples"] = int(self._dropped_image_samples)
        g.attrs["async_error"] = self._async_error or ""
        g.attrs["record_actions"] = int(self.record_actions)
        g.attrs["action_convention"] = (
            "action = actions/joint_pos_command = data.ctrl[actuator_ids]"
        )

        final_values = {
            "final_joint_pos": self._joint_pos(),
            "final_joint_vel": self._joint_vel(),
            "final_joint_torque": self._joint_torque(),
            "final_ee_pose": self._ee_pose(),
            "final_joint_pos_command": self._joint_pos_command(),

            # "final_ft_wrench": self._ft_wrench(),
            
            # "final_ft_wrench": self._ft_wrench(),
            # "final_ft_wrench_raw": self._ft_wrench_raw(),

            "final_ft_wrench": self._ft_wrench(),
            "final_ft_wrench_raw": self._ft_wrench_raw(),
            "final_ft_wrench_gravity": self._ft_gravity_wrench(),

            "final_peg_tip_pos": self._site_pos(self.peg_tip_site_id),
            "final_hole_center_pos": self._site_pos(self.hole_center_site_id),
            "final_hole_goal_pos": self._site_pos(self.hole_goal_site_id),
        }
        for name, value in final_values.items():
            if name in g:
                del g[name]
            g.create_dataset(name, data=value)

        final_err_xyz = self._site_pos(self.peg_tip_site_id) - self._site_pos(self.hole_center_site_id)
        g.attrs["final_align_err_xz"] = float(np.linalg.norm([final_err_xyz[0], final_err_xyz[2]]))
        g.attrs["final_insertion_err_y"] = float(final_err_xyz[1])

        self.h5.attrs["status"] = status
        self.h5.attrs["duration_sim"] = float(self.data.time - self.episode_start_sim_time)
        self.h5.attrs["n_state"] = int(self.n_state)
        self.h5.attrs["n_force"] = int(self.n_force)
        self.h5.attrs["n_image"] = int(self.n_image)
        self.h5.attrs["dropped_numeric_samples"] = int(
            self._dropped_numeric_samples
        )
        self.h5.attrs["dropped_image_samples"] = int(
            self._dropped_image_samples
        )

    def _write_events(self) -> None:
        assert self.h5 is not None
        g = self.h5["events"]
        str_dtype = h5py.string_dtype(encoding="utf-8")
        names = np.asarray([str(r.get("event", "")) for r in self.event_rows], dtype=object)
        t_sim = np.asarray([float(r.get("t_sim", np.nan)) for r in self.event_rows], dtype=np.float64)
        t_episode = np.asarray([float(r.get("t_episode", np.nan)) for r in self.event_rows], dtype=np.float64)
        t_wall = np.asarray([float(r.get("t_wall", np.nan)) for r in self.event_rows], dtype=np.float64)
        standard_keys = {
            "event",
            "t_sim",
            "t_episode",
            "t_wall",
            "t_wall_from_start",
        }
        extra_json = np.asarray(
            [
                json.dumps(
                    self._json_safe(
                        {
                            key: value
                            for key, value in row.items()
                            if key not in standard_keys
                        }
                    ),
                    ensure_ascii=False,
                )
                for row in self.event_rows
            ],
            dtype=object,
        )
        for key in ["names", "t_sim", "t_episode", "t_wall", "extra_json"]:
            if key in g:
                del g[key]
        g.create_dataset("names", data=names, dtype=str_dtype)
        g.create_dataset("t_sim", data=t_sim)
        g.create_dataset("t_episode", data=t_episode)
        g.create_dataset("t_wall", data=t_wall)
        g.create_dataset("extra_json", data=extra_json, dtype=str_dtype)

    def _write_string_dataset(self, path: str, values: List[str]) -> None:
        assert self.h5 is not None
        str_dtype = h5py.string_dtype(encoding="utf-8")
        arr = np.asarray([str(v) for v in values], dtype=object)
        if path in self.h5:
            del self.h5[path]
        self.h5.create_dataset(path, data=arr, dtype=str_dtype)

    def _write_sidecar_json(self, status: str) -> None:
        if self.session_dir is None or self.hdf5_path is None:
            return
        sidecar = {
            "hdf5_path": str(self.hdf5_path),
            "status": status,
            "episode_label": self.episode_label,
            "duration_sim": float(self.data.time - self.episode_start_sim_time),
            "n_state": int(self.n_state),
            "n_force": int(self.n_force),
            "n_image": int(self.n_image),
            "dropped_numeric_samples": int(self._dropped_numeric_samples),
            "dropped_image_samples": int(self._dropped_image_samples),
            "async_error": self._async_error or "",
            "camera_names": list(self.camera_ids.keys()),
            "image_storage": "inside_hdf5_uint8_rgb",
            "schema_version": "compact_mujoco_hdf5_v1",
            "episode_context": self._json_safe(self.episode_context),
        }
        with (self.session_dir / "metadata.json").open("w", encoding="utf-8") as f:
            json.dump(sidecar, f, indent=2, ensure_ascii=False)

    def _write_episode_context(self, group: h5py.Group) -> None:
        """Persist controller-provided task setup alongside built-in metadata."""
        for key, value in self.episode_context.items():
            name = str(key)
            if not name or "/" in name:
                raise ValueError(f"Invalid episode metadata key: {name!r}")

            if isinstance(value, (str, bytes, bool, int, float, np.generic)):
                group.attrs[name] = value
                continue

            array = np.asarray(value)
            if array.dtype.kind in {"b", "i", "u", "f"}:
                if name in group:
                    del group[name]
                group.create_dataset(name, data=array)
            else:
                group.attrs[name] = json.dumps(
                    self._json_safe(value),
                    ensure_ascii=False,
                )

    @classmethod
    def _json_safe(cls, value):
        if isinstance(value, dict):
            return {str(k): cls._json_safe(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._json_safe(v) for v in value]
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, Path):
            return str(value)
        return value

    # ------------------------------------------------------------------
    # Low-level getters
    # ------------------------------------------------------------------
    def _t_episode(self) -> float:
        return float(self.data.time - self.episode_start_sim_time)

    def _joint_pos(self) -> np.ndarray:
        out = np.full(len(self.joint_ids), np.nan, dtype=np.float64)
        for i, jid in enumerate(self.joint_ids):
            if jid == -1:
                continue
            qadr = self.model.jnt_qposadr[jid]
            out[i] = float(self.data.qpos[qadr])
        return out

    def _joint_vel(self) -> np.ndarray:
        out = np.full(len(self.joint_ids), np.nan, dtype=np.float64)
        for i, jid in enumerate(self.joint_ids):
            if jid == -1:
                continue
            dadr = self.model.jnt_dofadr[jid]
            out[i] = float(self.data.qvel[dadr])
        return out

    def _joint_torque(self) -> np.ndarray:
        """
        For hinge joints, MuJoCo generalized actuator force at the joint dof
        can be interpreted as the actuator torque for that joint.
        """
        out = np.full(len(self.joint_ids), np.nan, dtype=np.float64)
        for i, jid in enumerate(self.joint_ids):
            if jid == -1:
                continue
            dadr = self.model.jnt_dofadr[jid]
            out[i] = float(self.data.qfrc_actuator[dadr])
        return out

    def _ee_pose(self) -> np.ndarray:
        if self.ee_body_id == -1:
            return np.full(7, np.nan, dtype=np.float64)
        p = self.data.xpos[self.ee_body_id].copy()
        q = self.data.xquat[self.ee_body_id].copy()  # MuJoCo quaternion order: w x y z
        return np.asarray([p[0], p[1], p[2], q[0], q[1], q[2], q[3]], dtype=np.float64)

    def _site_pos(self, site_id: int) -> np.ndarray:
        if site_id == -1:
            return np.full(3, np.nan, dtype=np.float64)
        return self.data.site_xpos[site_id].copy().astype(np.float64)

    def _sensor_vec(self, sensor_id: int, dim_expected: int) -> np.ndarray:
        if sensor_id == -1:
            return np.full(dim_expected, np.nan, dtype=np.float64)
        adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        values = self.data.sensordata[adr:adr + dim].copy().astype(np.float64)
        if len(values) < dim_expected:
            padded = np.full(dim_expected, np.nan, dtype=np.float64)
            padded[:len(values)] = values
            return padded
        return values[:dim_expected]

    def _joint_pos_command(self) -> np.ndarray:
        """
        Actual position command sent to MuJoCo actuators.

        For position actuators, data.ctrl[actuator_id] is the actuator's
        current position target. This is the best ACT-style action label.
        """
        out = np.full(len(self.actuator_ids), np.nan, dtype=np.float64)

        for i, aid in enumerate(self.actuator_ids):
            if aid == -1:
                continue

            if aid < 0 or aid >= self.data.ctrl.shape[0]:
                continue

            out[i] = float(self.data.ctrl[aid])

        return out

    # def _ft_wrench(self) -> np.ndarray:
    #     force = self._sensor_vec(self.ft_force_sensor_id, 3)
    #     torque = self._sensor_vec(self.ft_torque_sensor_id, 3)
    #     return np.concatenate([force, torque]).astype(np.float64)
    # def _ft_wrench_raw(self) -> np.ndarray:
    #     """
    #     Raw 6D force/torque sensor reading from MuJoCo.

    #     Output:
    #         [Fx, Fy, Fz, Tx, Ty, Tz]
    #     """
    #     force = self._sensor_vec(self.ft_force_sensor_id, 3)
    #     torque = self._sensor_vec(self.ft_torque_sensor_id, 3)
    #     return np.concatenate([force, torque]).astype(np.float64)


    # def _compensate_ft_wrench(self, raw: np.ndarray) -> np.ndarray:
    #     """
    #     Episode-start tare compensation.

    #     This is not a full pose-dependent gravity compensation model.
    #     It removes the static bias measured at the beginning of the episode.
    #     """
    #     raw = np.asarray(raw, dtype=np.float64)

    #     if self.enable_ft_tare:
    #         return raw - self.ft_wrench_bias_raw

    #     return raw


    # def _ft_wrench(self) -> np.ndarray:
    #     """
    #     Default wrench used by the dataset.

    #     If enable_ft_tare=True, this returns tare-compensated wrench.
    #     Otherwise, it returns raw wrench.
    #     """
    #     raw = self._ft_wrench_raw()
    #     return self._compensate_ft_wrench(raw)
    def _ft_wrench_raw(self) -> np.ndarray:
        """
        Raw 6D force/torque sensor reading from MuJoCo.

        Output:
            [Fx, Fy, Fz, Tx, Ty, Tz]
        """
        force = self._sensor_vec(self.ft_force_sensor_id, 3)
        torque = self._sensor_vec(self.ft_torque_sensor_id, 3)
        return np.concatenate([force, torque]).astype(np.float64)


    def _tool_mass_and_com_world(self):
        """
        Compute total mass and world COM of the configured tool bodies.

        This uses MuJoCo compiled body masses and body inertial COM positions.
        For your current peg task, the usual body list is ["peg_tool"].
        """
        if not self.ft_gravity_tool_body_ids:
            return 0.0, np.full(3, np.nan, dtype=np.float64)

        total_mass = 0.0
        weighted_com = np.zeros(3, dtype=np.float64)

        for bid in self.ft_gravity_tool_body_ids:
            m = float(self.model.body_mass[bid])

            if m <= 0.0:
                continue

            # data.xipos[bid] is the world position of the body inertial frame.
            com_w = self.data.xipos[bid].copy().astype(np.float64)

            total_mass += m
            weighted_com += m * com_w

        if total_mass <= 1e-12:
            return 0.0, np.full(3, np.nan, dtype=np.float64)

        return total_mass, weighted_com / total_mass


    def _ft_sensor_pose_world(self):
        """
        Return sensor site world position and rotation.

        R_WS maps sensor-frame vectors to world-frame vectors.
        """
        if self.ft_sensor_site_id == -1:
            return (
                np.full(3, np.nan, dtype=np.float64),
                np.eye(3, dtype=np.float64),
            )

        p_ws = self.data.site_xpos[self.ft_sensor_site_id].copy().astype(np.float64)

        R_ws = self.data.site_xmat[self.ft_sensor_site_id].copy().reshape(3, 3)
        R_ws = R_ws.astype(np.float64)

        return p_ws, R_ws


    def _ft_gravity_wrench(self) -> np.ndarray:
        """
        Predict the wrench measured by the FT sensor due to tool gravity.

        The result is expressed in the sensor site frame:
            [Fx, Fy, Fz, Tx, Ty, Tz]

        The sign convention is controlled by self.ft_gravity_sensor_sign.
        For MuJoCo force sensors attached to a child body, -1.0 is often the
        correct default for removing the tool's static load from the sensor output.
        """
        if self.ft_compensation_mode not in {"gravity", "gravity_tare"}:
            return np.zeros(6, dtype=np.float64)

        if self.ft_sensor_site_id == -1:
            return np.zeros(6, dtype=np.float64)

        tool_mass, tool_com_w = self._tool_mass_and_com_world()

        if tool_mass <= 1e-12 or np.any(np.isnan(tool_com_w)):
            return np.zeros(6, dtype=np.float64)

        sensor_pos_w, R_ws = self._ft_sensor_pose_world()

        if np.any(np.isnan(sensor_pos_w)):
            return np.zeros(6, dtype=np.float64)

        # world -> sensor
        R_sw = R_ws.T

        # Force of gravity on the tool, expressed in world frame.
        force_g_w = tool_mass * self.ft_gravity_world

        # Express gravity force in sensor frame.
        force_g_s = R_sw @ force_g_w

        # COM position relative to sensor origin, expressed in sensor frame.
        r_com_s = R_sw @ (tool_com_w - sensor_pos_w)

        # Gravity torque about sensor origin.
        torque_g_s = np.cross(r_com_s, force_g_s)

        gravity_wrench = np.concatenate([force_g_s, torque_g_s]).astype(np.float64)

        # Convert physical gravity wrench to the MuJoCo sensor sign convention.
        return self.ft_gravity_sensor_sign * gravity_wrench


    def _compensate_ft_wrench(
        self,
        raw: np.ndarray,
        gravity: np.ndarray,
    ) -> np.ndarray:
        """
        Apply selected FT compensation mode.
        """
        raw = np.asarray(raw, dtype=np.float64)
        gravity = np.asarray(gravity, dtype=np.float64)

        if self.ft_compensation_mode == "none":
            return raw

        if self.ft_compensation_mode == "gravity":
            return raw - gravity

        raise ValueError(
            f"Unknown ft_compensation_mode: {self.ft_compensation_mode}"
        )


    def _ft_wrench(self) -> np.ndarray:
        """
        Default wrench used by the dataset.
        """
        raw = self._ft_wrench_raw()
        gravity = self._ft_gravity_wrench()
        return self._compensate_ft_wrench(raw, gravity)

    @staticmethod
    def _safe_name(text: str) -> str:
        safe = "".join(c if c.isalnum() or c in {"_", "-"} else "_" for c in str(text))
        return safe.strip("_") or "episode"


MujocoCompactHDF5Recorder = MujocoHDF5Recorder
