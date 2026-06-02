#!/usr/bin/env python3
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import h5py
import imageio.v2 as imageio
import mujoco
import numpy as np


class MujocoHDF5Recorder:
    """
    HDF5 recorder for MuJoCo peg-in-hole teleoperation.

    One episode creates:
        <output_dir>/<timestamp>_<label>/episode.hdf5
        <output_dir>/<timestamp>_<label>/images/<camera_name>/*.jpg   optional
        <output_dir>/<timestamp>_<label>/metadata.json

    Alignment rule:
        Every stream uses MuJoCo simulation time: data.time.
    """

    def __init__(
        self,
        model,
        data,
        output_dir: str,
        model_path: str = "",
        force_hz: float = 500.0,
        state_hz: float = 30.0,
        image_hz: float = 30.0,
        record_images: bool = False,
        camera_names: Optional[List[str]] = None,
        image_width: int = 640,
        image_height: int = 480,
        image_format: str = "jpg",
        jpg_quality: int = 90,
        max_buffer_rows: int = 500000,
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
        self.camera_names = list(camera_names or [])
        self.image_width = int(image_width)
        self.image_height = int(image_height)
        self.image_format = image_format.lower().lstrip(".")
        self.jpg_quality = int(jpg_quality)
        self.max_buffer_rows = int(max_buffer_rows)

        if self.image_format not in {"jpg", "jpeg", "png"}:
            raise ValueError("image_format must be jpg, jpeg, or png")

        self.active = False
        self.session_dir: Optional[Path] = None
        self.hdf5_path: Optional[Path] = None
        self.image_root: Optional[Path] = None
        self.renderer: Optional[mujoco.Renderer] = None

        self.episode_label = ""
        self.episode_start_sim_time = 0.0
        self.episode_start_wall_time = 0.0
        self.next_force_t = 0.0
        self.next_state_t = 0.0
        self.next_image_t = 0.0
        self.image_frame_id = 0

        self.joint_names = [f"joint_{i}" for i in range(1, 8)]
        self.joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.joint_names]
        self.actuator_names = [f"motor_joint_{i}" for i in range(1, 8)]
        self.actuator_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in self.actuator_names]

        self.link7_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_7")
        self.peg_tool_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "peg_tool")
        self.peg_tip_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "peg_tip_site")
        self.hole_center_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "hole_center_site")
        self.ft_force_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_ft_force")
        self.ft_torque_sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_ft_torque")

        self.camera_ids: Dict[str, int] = {}
        if self.record_images:
            for name in self.camera_names:
                cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
                if cid == -1:
                    print(f"[HDF5Recorder] Warning: camera not found: {name}")
                else:
                    self.camera_ids[name] = cid
            if not self.camera_ids:
                print("[HDF5Recorder] No valid cameras; image recording disabled.")
                self.record_images = False

        self._reset_buffers()
        print(f"[HDF5Recorder] Ready. output_dir={self.output_dir}")

    def start_episode(self, label: str = "teleop") -> Optional[Path]:
        if self.active:
            print("[HDF5Recorder] Episode already active.")
            return self.hdf5_path

        safe = self._safe(label)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"{stamp}_{safe}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.hdf5_path = self.session_dir / "episode.hdf5"

        if self.record_images:
            self.image_root = self.session_dir / "images"
            self.image_root.mkdir(parents=True, exist_ok=True)
            for cam in self.camera_ids:
                (self.image_root / cam).mkdir(parents=True, exist_ok=True)

        self.episode_label = safe
        self.episode_start_sim_time = float(self.data.time)
        self.episode_start_wall_time = time.time()
        self.next_force_t = float(self.data.time)
        self.next_state_t = float(self.data.time)
        self.next_image_t = float(self.data.time)
        self.image_frame_id = 0
        self._reset_buffers()
        self.active = True
        self.add_event("record_start")
        print(f"[HDF5Recorder] Started: {self.session_dir}")
        return self.hdf5_path

    def stop_episode(self, status: str = "manual_stop") -> Optional[Path]:
        if not self.active:
            print("[HDF5Recorder] No active episode.")
            return self.hdf5_path
        self.add_event(status)
        self._write_hdf5(status=status)
        self.active = False
        print(f"[HDF5Recorder] Saved: {self.hdf5_path}")
        return self.hdf5_path

    def close(self) -> None:
        if self.active:
            self.stop_episode("controller_shutdown")
        if self.renderer is not None:
            self.renderer.close()
            self.renderer = None

    def add_event(self, event: str, extra: Optional[Dict[str, Any]] = None) -> None:
        if not self.active:
            print(f"[HDF5Recorder] Ignored event without active episode: {event}")
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
        print(f"[HDF5Recorder] Event: {event} @ {row['t_episode']:.3f}s")

    def record_if_needed(self, controller) -> None:
        if not self.active:
            return
        t = float(self.data.time)

        if t + 1e-12 >= self.next_force_t:
            self.force_rows.append(self._force_row(controller))
            while self.next_force_t <= t + 1e-12:
                self.next_force_t += self.force_period

        if t + 1e-12 >= self.next_state_t:
            self.state_rows.append(self._state_row(controller))
            while self.next_state_t <= t + 1e-12:
                self.next_state_t += self.state_period

        if self.record_images and t + 1e-12 >= self.next_image_t:
            self._record_images()
            while self.next_image_t <= t + 1e-12:
                self.next_image_t += self.image_period

        if max(len(self.force_rows), len(self.state_rows), len(self.image_rows)) > self.max_buffer_rows:
            self.stop_episode("buffer_limit")

    def _base(self) -> Dict[str, float]:
        return {
            "t_sim": float(self.data.time),
            "t_episode": float(self.data.time - self.episode_start_sim_time),
            "t_wall": float(time.time()),
            "t_wall_from_start": float(time.time() - self.episode_start_wall_time),
        }

    def _force_row(self, controller) -> Dict[str, Any]:
        r: Dict[str, Any] = {}
        r.update(self._base())
        r.update(self._ft())
        r.update(self._task_error())
        r.update(self._contact())
        r.update(self._targets(controller))
        return r

    def _state_row(self, controller) -> Dict[str, Any]:
        r: Dict[str, Any] = {}
        r.update(self._base())
        r.update(self._joints())
        r.update(self._ctrl())
        r.update(self._targets(controller))
        r.update(self._pose(self.link7_body_id, "link7"))
        r.update(self._pose(self.peg_tool_body_id, "peg_tool"))
        r.update(self._site(self.peg_tip_site_id, "peg_tip"))
        r.update(self._site(self.hole_center_site_id, "hole_center"))
        r.update(self._task_error())
        r.update(self._ft())
        r.update(self._contact())
        return r

    def _record_images(self) -> None:
        if self.session_dir is None:
            return
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=self.image_height, width=self.image_width)

        t = float(self.data.time)
        te = float(self.data.time - self.episode_start_sim_time)
        fid = int(self.image_frame_id)

        for cam_name, cam_id in self.camera_ids.items():
            self.renderer.update_scene(self.data, camera=cam_id)
            rgb = self.renderer.render()
            fname = f"{fid:08d}_{t:.6f}.{self.image_format}"
            rel = Path("images") / cam_name / fname
            abs_path = self.session_dir / rel
            if self.image_format in {"jpg", "jpeg"}:
                imageio.imwrite(abs_path, rgb, quality=self.jpg_quality)
            else:
                imageio.imwrite(abs_path, rgb)
            self.image_rows.append({
                "frame_id": fid,
                "t_sim": t,
                "t_episode": te,
                "camera": cam_name,
                "file": str(rel),
                "width": self.image_width,
                "height": self.image_height,
            })
        self.image_frame_id += 1

    def _joints(self) -> Dict[str, float]:
        r: Dict[str, float] = {}
        for i, jid in enumerate(self.joint_ids, 1):
            if jid == -1:
                r[f"q{i}"] = np.nan
                r[f"dq{i}"] = np.nan
            else:
                r[f"q{i}"] = float(self.data.qpos[self.model.jnt_qposadr[jid]])
                r[f"dq{i}"] = float(self.data.qvel[self.model.jnt_dofadr[jid]])
        return r

    def _ctrl(self) -> Dict[str, float]:
        r: Dict[str, float] = {}
        for i, aid in enumerate(self.actuator_ids, 1):
            r[f"ctrl{i}"] = float(self.data.ctrl[aid]) if aid != -1 else np.nan
        return r

    def _targets(self, controller) -> Dict[str, float]:
        r: Dict[str, float] = {}
        target = getattr(controller, "target_joints", None)
        command = getattr(controller, "command_joints", None)
        for i in range(7):
            r[f"q_target{i+1}"] = float(target[i]) if target is not None and i < len(target) else np.nan
            r[f"q_cmd{i+1}"] = float(command[i]) if command is not None and i < len(command) else np.nan
        return r

    def _pose(self, body_id: int, prefix: str) -> Dict[str, float]:
        keys = [f"{prefix}_{s}" for s in ["px", "py", "pz", "qw", "qx", "qy", "qz"]]
        if body_id == -1:
            return {k: np.nan for k in keys}
        p = self.data.xpos[body_id].copy()
        q = self.data.xquat[body_id].copy()
        return dict(zip(keys, [float(p[0]), float(p[1]), float(p[2]), float(q[0]), float(q[1]), float(q[2]), float(q[3])]))

    def _site(self, site_id: int, prefix: str) -> Dict[str, float]:
        if site_id == -1:
            return {f"{prefix}_{s}": np.nan for s in ["x", "y", "z"]}
        p = self.data.site_xpos[site_id].copy()
        return {f"{prefix}_x": float(p[0]), f"{prefix}_y": float(p[1]), f"{prefix}_z": float(p[2])}

    def _sensor(self, sid: int, n: int) -> List[float]:
        if sid == -1:
            return [np.nan] * n
        adr = self.model.sensor_adr[sid]
        dim = self.model.sensor_dim[sid]
        v = self.data.sensordata[adr: adr + dim].copy().tolist()
        return [float(x) for x in (v + [np.nan] * n)[:n]]

    def _ft(self) -> Dict[str, float]:
        f = self._sensor(self.ft_force_sensor_id, 3)
        tau = self._sensor(self.ft_torque_sensor_id, 3)
        return {"fx": f[0], "fy": f[1], "fz": f[2], "tx": tau[0], "ty": tau[1], "tz": tau[2]}

    def _task_error(self) -> Dict[str, float]:
        peg = self._site(self.peg_tip_site_id, "peg_tip")
        hole = self._site(self.hole_center_site_id, "hole_center")
        vals = [peg["peg_tip_x"], peg["peg_tip_y"], peg["peg_tip_z"], hole["hole_center_x"], hole["hole_center_y"], hole["hole_center_z"]]
        if any(np.isnan(v) for v in vals):
            return {"err_x": np.nan, "err_y": np.nan, "err_z": np.nan, "align_err_xz": np.nan, "insertion_err_y": np.nan}
        ex = peg["peg_tip_x"] - hole["hole_center_x"]
        ey = peg["peg_tip_y"] - hole["hole_center_y"]
        ez = peg["peg_tip_z"] - hole["hole_center_z"]
        return {"err_x": float(ex), "err_y": float(ey), "err_z": float(ez), "align_err_xz": float(np.linalg.norm([ex, ez])), "insertion_err_y": float(ey)}

    def _contact(self) -> Dict[str, Any]:
        peg_contact = 0
        min_dist = np.nan
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)
            if g1 == "cylindrical_peg" or g2 == "cylindrical_peg":
                peg_contact = 1
                if np.isnan(min_dist) or c.dist < min_dist:
                    min_dist = float(c.dist)
        return {"ncon": int(self.data.ncon), "peg_contact": int(peg_contact), "peg_contact_min_dist": min_dist}

    def _write_hdf5(self, status: str) -> None:
        assert self.hdf5_path is not None
        with h5py.File(self.hdf5_path, "w") as f:
            f.attrs["status"] = status
            f.attrs["episode_label"] = self.episode_label
            f.attrs["model_path"] = self.model_path
            f.attrs["mujoco_timestep"] = float(self.model.opt.timestep)
            f.attrs["force_hz"] = self.force_hz
            f.attrs["state_hz"] = self.state_hz
            f.attrs["image_hz"] = self.image_hz
            f.attrs["alignment_clock"] = "mujoco data.time"
            f.attrs["duration_sim"] = float(self.data.time - self.episode_start_sim_time)
            f.attrs["schema_json"] = json.dumps({
                "image_storage": "external JPG/PNG paths in observations/images/<camera>/file_paths",
                "task_convention": "wall-parallel: x-z alignment plane, y insertion axis",
            })

            self._write_state(f)
            self._write_force(f)
            self._write_images(f)
            self._write_events(f)

        sidecar = {
            "hdf5_path": str(self.hdf5_path),
            "status": status,
            "duration_sim": float(self.data.time - self.episode_start_sim_time),
            "n_state": len(self.state_rows),
            "n_force": len(self.force_rows),
            "n_image_rows": len(self.image_rows),
            "n_events": len(self.event_rows),
            "record_images": self.record_images,
            "camera_names": list(self.camera_ids.keys()),
        }
        assert self.session_dir is not None
        with (self.session_dir / "metadata.json").open("w", encoding="utf-8") as f:
            json.dump(sidecar, f, indent=2, ensure_ascii=False)

    def _write_state(self, f: h5py.File) -> None:
        rows = self.state_rows
        gts = f.require_group("timestamps")
        obs = f.require_group("observations")
        act = f.require_group("actions")
        task = f.require_group("task")
        con = f.require_group("contact")

        gts.create_dataset("state", data=self._col(rows, "t_sim"))
        gts.create_dataset("state_episode", data=self._col(rows, "t_episode"))
        obs.create_dataset("qpos", data=self._mat(rows, [f"q{i}" for i in range(1, 8)]))
        obs.create_dataset("qvel", data=self._mat(rows, [f"dq{i}" for i in range(1, 8)]))
        obs.create_dataset("ee_pose", data=self._mat(rows, ["peg_tool_px", "peg_tool_py", "peg_tool_pz", "peg_tool_qw", "peg_tool_qx", "peg_tool_qy", "peg_tool_qz"]))
        obs.create_dataset("link7_pose", data=self._mat(rows, ["link7_px", "link7_py", "link7_pz", "link7_qw", "link7_qx", "link7_qy", "link7_qz"]))
        obs.create_dataset("peg_tip_pos", data=self._mat(rows, ["peg_tip_x", "peg_tip_y", "peg_tip_z"]))
        obs.create_dataset("hole_center_pos", data=self._mat(rows, ["hole_center_x", "hole_center_y", "hole_center_z"]))
        obs.create_dataset("force_torque_state_sample", data=self._mat(rows, ["fx", "fy", "fz", "tx", "ty", "tz"]))
        act.create_dataset("q_target", data=self._mat(rows, [f"q_target{i}" for i in range(1, 8)]))
        act.create_dataset("q_cmd", data=self._mat(rows, [f"q_cmd{i}" for i in range(1, 8)]))
        act.create_dataset("ctrl", data=self._mat(rows, [f"ctrl{i}" for i in range(1, 8)]))
        task.create_dataset("state_error_xyz", data=self._mat(rows, ["err_x", "err_y", "err_z"]))
        task.create_dataset("state_align_err_xz", data=self._col(rows, "align_err_xz"))
        task.create_dataset("state_insertion_err_y", data=self._col(rows, "insertion_err_y"))
        con.create_dataset("state_peg_contact", data=self._col(rows, "peg_contact", dtype=np.int32))
        con.create_dataset("state_peg_contact_min_dist", data=self._col(rows, "peg_contact_min_dist"))

    def _write_force(self, f: h5py.File) -> None:
        rows = self.force_rows
        gts = f.require_group("timestamps")
        obs = f.require_group("observations")
        task = f.require_group("task")
        con = f.require_group("contact")
        act = f.require_group("actions")
        gts.create_dataset("force", data=self._col(rows, "t_sim"))
        gts.create_dataset("force_episode", data=self._col(rows, "t_episode"))
        obs.create_dataset("force_torque", data=self._mat(rows, ["fx", "fy", "fz", "tx", "ty", "tz"]))
        task.create_dataset("force_error_xyz", data=self._mat(rows, ["err_x", "err_y", "err_z"]))
        task.create_dataset("force_align_err_xz", data=self._col(rows, "align_err_xz"))
        task.create_dataset("force_insertion_err_y", data=self._col(rows, "insertion_err_y"))
        con.create_dataset("force_peg_contact", data=self._col(rows, "peg_contact", dtype=np.int32))
        con.create_dataset("force_peg_contact_min_dist", data=self._col(rows, "peg_contact_min_dist"))
        act.create_dataset("q_target_force_sample", data=self._mat(rows, [f"q_target{i}" for i in range(1, 8)]))
        act.create_dataset("q_cmd_force_sample", data=self._mat(rows, [f"q_cmd{i}" for i in range(1, 8)]))

    def _write_images(self, f: h5py.File) -> None:
        rows = self.image_rows
        gts = f.require_group("timestamps")
        img = f.require_group("observations").require_group("images")
        str_dt = h5py.string_dtype(encoding="utf-8")
        gts.create_dataset("image", data=self._col(rows, "t_sim"))
        gts.create_dataset("image_episode", data=self._col(rows, "t_episode"))
        cams = sorted(set(str(r.get("camera", "")) for r in rows if r.get("camera", "")))
        img.create_dataset("camera_names", data=np.asarray(cams, dtype=object), dtype=str_dt)
        for cam in cams:
            rs = [r for r in rows if r.get("camera") == cam]
            cg = img.require_group(cam)
            cg.create_dataset("timestamps", data=self._col(rs, "t_sim"))
            cg.create_dataset("timestamps_episode", data=self._col(rs, "t_episode"))
            cg.create_dataset("frame_id", data=self._col(rs, "frame_id", dtype=np.int64))
            paths = np.asarray([str(r.get("file", "")) for r in rs], dtype=object)
            cg.create_dataset("file_paths", data=paths, dtype=str_dt)
            cg.attrs["width"] = self.image_width
            cg.attrs["height"] = self.image_height

    def _write_events(self, f: h5py.File) -> None:
        rows = self.event_rows
        g = f.require_group("events")
        str_dt = h5py.string_dtype(encoding="utf-8")
        names = np.asarray([str(r.get("event", "")) for r in rows], dtype=object)
        g.create_dataset("names", data=names, dtype=str_dt)
        g.create_dataset("t_sim", data=self._col(rows, "t_sim"))
        g.create_dataset("t_episode", data=self._col(rows, "t_episode"))

    def _reset_buffers(self) -> None:
        self.force_rows: List[Dict[str, Any]] = []
        self.state_rows: List[Dict[str, Any]] = []
        self.image_rows: List[Dict[str, Any]] = []
        self.event_rows: List[Dict[str, Any]] = []

    @staticmethod
    def _safe(x: str) -> str:
        s = "".join(c if c.isalnum() or c in {"_", "-"} else "_" for c in str(x))
        return s.strip("_") or "episode"

    @staticmethod
    def _col(rows: List[Dict[str, Any]], key: str, dtype=np.float64) -> np.ndarray:
        if not rows:
            return np.asarray([], dtype=dtype)
        return np.asarray([r.get(key, np.nan) for r in rows], dtype=dtype)

    @staticmethod
    def _mat(rows: List[Dict[str, Any]], keys: List[str], dtype=np.float64) -> np.ndarray:
        if not rows:
            return np.empty((0, len(keys)), dtype=dtype)
        return np.asarray([[r.get(k, np.nan) for k in keys] for r in rows], dtype=dtype)
