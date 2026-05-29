#!/usr/bin/env python3
"""
MuJoCo data recorder for peg-in-hole teleoperation experiments.

Use MuJoCo simulation time `data.time` as the only alignment clock.

Files written:
  metadata.json
  force_500hz.csv
  state_30hz.csv
  all_500hz.csv
"""

from __future__ import annotations

import csv
import json
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import mujoco


class MujocoDataRecorder:
    def __init__(
        self,
        model,
        data,
        output_dir: str,
        model_path: str = "",
        force_hz: float = 500.0,
        state_hz: float = 30.0,
        write_all_500hz: bool = True,
        max_buffer_rows: int = 200000,
    ):
        self.model = model
        self.data = data
        self.model_path = model_path

        self.output_dir = Path(output_dir).expanduser().resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.session_dir = self.output_dir / f"peg_in_hole_{timestamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.force_hz = float(force_hz)
        self.state_hz = float(state_hz)
        self.write_all_500hz = bool(write_all_500hz)
        self.max_buffer_rows = int(max_buffer_rows)

        self.force_period = 1.0 / max(self.force_hz, 1e-6)
        self.state_period = 1.0 / max(self.state_hz, 1e-6)

        self.next_force_t = 0.0
        self.next_state_t = 0.0

        self.force_rows: List[Dict[str, Any]] = []
        self.state_rows: List[Dict[str, Any]] = []
        self.all_rows: List[Dict[str, Any]] = []

        self.start_wall_time = time.time()
        self.closed = False

        self.joint_names = [f"joint_{i}" for i in range(1, 8)]
        self.joint_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            for name in self.joint_names
        ]

        self.actuator_names = [f"motor_joint_{i}" for i in range(1, 8)]
        self.actuator_ids = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
            for name in self.actuator_names
        ]

        self.peg_tool_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "peg_tool"
        )
        self.link7_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "link_7"
        )
        self.peg_tip_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "peg_tip_site"
        )
        self.hole_center_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "hole_center_site"
        )

        self.ft_force_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_ft_force"
        )
        self.ft_torque_sensor_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR, "peg_ft_torque"
        )

        self._write_metadata()
        print(f"[MujocoDataRecorder] Recording to: {self.session_dir}")

    def _sensor_vec(self, sensor_id: int, dim_expected: int) -> List[float]:
        if sensor_id == -1:
            return [float("nan")] * dim_expected

        adr = self.model.sensor_adr[sensor_id]
        dim = self.model.sensor_dim[sensor_id]
        vec = self.data.sensordata[adr:adr + dim].copy().tolist()

        if len(vec) < dim_expected:
            vec += [float("nan")] * (dim_expected - len(vec))

        return [float(x) for x in vec[:dim_expected]]

    def _body_pose(self, body_id: int, prefix: str) -> Dict[str, float]:
        if body_id == -1:
            return {
                f"{prefix}_px": float("nan"),
                f"{prefix}_py": float("nan"),
                f"{prefix}_pz": float("nan"),
                f"{prefix}_qw": float("nan"),
                f"{prefix}_qx": float("nan"),
                f"{prefix}_qy": float("nan"),
                f"{prefix}_qz": float("nan"),
            }

        p = self.data.xpos[body_id].copy()
        q = self.data.xquat[body_id].copy()

        return {
            f"{prefix}_px": float(p[0]),
            f"{prefix}_py": float(p[1]),
            f"{prefix}_pz": float(p[2]),
            f"{prefix}_qw": float(q[0]),
            f"{prefix}_qx": float(q[1]),
            f"{prefix}_qy": float(q[2]),
            f"{prefix}_qz": float(q[3]),
        }

    def _site_pos(self, site_id: int, prefix: str) -> Dict[str, float]:
        if site_id == -1:
            return {
                f"{prefix}_x": float("nan"),
                f"{prefix}_y": float("nan"),
                f"{prefix}_z": float("nan"),
            }

        p = self.data.site_xpos[site_id].copy()
        return {
            f"{prefix}_x": float(p[0]),
            f"{prefix}_y": float(p[1]),
            f"{prefix}_z": float(p[2]),
        }

    def _joint_state(self) -> Dict[str, float]:
        row: Dict[str, float] = {}

        for i, jid in enumerate(self.joint_ids, start=1):
            if jid == -1:
                row[f"q{i}"] = float("nan")
                row[f"dq{i}"] = float("nan")
                continue

            qadr = self.model.jnt_qposadr[jid]
            dadr = self.model.jnt_dofadr[jid]

            row[f"q{i}"] = float(self.data.qpos[qadr])
            row[f"dq{i}"] = float(self.data.qvel[dadr])

        return row

    def _ctrl_state(self) -> Dict[str, float]:
        row: Dict[str, float] = {}

        for i, aid in enumerate(self.actuator_ids, start=1):
            row[f"ctrl{i}"] = float(self.data.ctrl[aid]) if aid != -1 else float("nan")

        return row

    def _controller_targets(self, controller) -> Dict[str, float]:
        row: Dict[str, float] = {}

        target = getattr(controller, "target_joints", None)
        command = getattr(controller, "command_joints", None)

        for i in range(7):
            row[f"q_target{i+1}"] = (
                float(target[i]) if target is not None and i < len(target) else float("nan")
            )
            row[f"q_cmd{i+1}"] = (
                float(command[i]) if command is not None and i < len(command) else float("nan")
            )

        return row

    def _ft_state(self) -> Dict[str, float]:
        f = self._sensor_vec(self.ft_force_sensor_id, 3)
        t = self._sensor_vec(self.ft_torque_sensor_id, 3)

        return {
            "fx": f[0],
            "fy": f[1],
            "fz": f[2],
            "tx": t[0],
            "ty": t[1],
            "tz": t[2],
        }

    def _task_error_state(self) -> Dict[str, float]:
        peg = self._site_pos(self.peg_tip_site_id, "peg_tip")
        hole = self._site_pos(self.hole_center_site_id, "hole_center")

        px, py, pz = peg["peg_tip_x"], peg["peg_tip_y"], peg["peg_tip_z"]
        hx, hy, hz = hole["hole_center_x"], hole["hole_center_y"], hole["hole_center_z"]

        if any(np.isnan(v) for v in [px, py, pz, hx, hy, hz]):
            return {
                **peg,
                **hole,
                "err_x": float("nan"),
                "err_y": float("nan"),
                "err_z": float("nan"),
                "align_err_xz": float("nan"),
                "insertion_err_y": float("nan"),
            }

        err_x = px - hx
        err_y = py - hy
        err_z = pz - hz

        return {
            **peg,
            **hole,
            "err_x": float(err_x),
            "err_y": float(err_y),
            "err_z": float(err_z),
            "align_err_xz": float(np.linalg.norm([err_x, err_z])),
            "insertion_err_y": float(err_y),
        }

    def _contact_state(self) -> Dict[str, Any]:
        peg_contact = False
        min_dist = float("nan")
        contact_count = int(self.data.ncon)

        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom1)
            g2 = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, c.geom2)

            if g1 == "cylindrical_peg" or g2 == "cylindrical_peg":
                peg_contact = True
                if np.isnan(min_dist) or c.dist < min_dist:
                    min_dist = float(c.dist)

        return {
            "ncon": contact_count,
            "peg_contact": int(peg_contact),
            "peg_contact_min_dist": min_dist,
        }

    def _base_row(self) -> Dict[str, Any]:
        return {
            "t_sim": float(self.data.time),
            "t_wall": float(time.time()),
            "t_wall_from_start": float(time.time() - self.start_wall_time),
        }

    def _full_state_row(self, controller) -> Dict[str, Any]:
        row: Dict[str, Any] = {}
        row.update(self._base_row())
        row.update(self._joint_state())
        row.update(self._ctrl_state())
        row.update(self._controller_targets(controller))
        row.update(self._body_pose(self.link7_body_id, "link7"))
        row.update(self._body_pose(self.peg_tool_body_id, "peg_tool"))
        row.update(self._task_error_state())
        row.update(self._ft_state())
        row.update(self._contact_state())
        return row

    def _force_row(self, controller) -> Dict[str, Any]:
        row: Dict[str, Any] = {}
        row.update(self._base_row())
        row.update(self._ft_state())
        row.update(self._task_error_state())
        row.update(self._contact_state())
        row.update(self._controller_targets(controller))
        return row

    def record_if_needed(self, controller) -> None:
        if self.closed:
            return

        t = float(self.data.time)

        if t + 1e-12 >= self.next_force_t:
            self.force_rows.append(self._force_row(controller))

            if self.write_all_500hz:
                self.all_rows.append(self._full_state_row(controller))

            while self.next_force_t <= t + 1e-12:
                self.next_force_t += self.force_period

        if t + 1e-12 >= self.next_state_t:
            self.state_rows.append(self._full_state_row(controller))

            while self.next_state_t <= t + 1e-12:
                self.next_state_t += self.state_period

        if (
            len(self.force_rows) > self.max_buffer_rows
            or len(self.state_rows) > self.max_buffer_rows
            or len(self.all_rows) > self.max_buffer_rows
        ):
            print("[MujocoDataRecorder] Buffer limit reached. Flushing and closing.")
            self.close()

    def close(self) -> None:
        if self.closed:
            return

        self._write_csv(self.session_dir / "force_500hz.csv", self.force_rows)
        self._write_csv(self.session_dir / "state_30hz.csv", self.state_rows)

        if self.write_all_500hz:
            self._write_csv(self.session_dir / "all_500hz.csv", self.all_rows)

        self.closed = True
        print(f"[MujocoDataRecorder] Saved data to: {self.session_dir}")

    def _write_csv(self, path: Path, rows: List[Dict[str, Any]]) -> None:
        if not rows:
            print(f"[MujocoDataRecorder] No rows for {path.name}")
            return

        fieldnames = list(rows[0].keys())

        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        print(f"[MujocoDataRecorder] Wrote {len(rows)} rows: {path.name}")

    def _write_metadata(self) -> None:
        metadata = {
            "created_wall_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "model_path": self.model_path,
            "mujoco_timestep": float(self.model.opt.timestep),
            "force_hz": self.force_hz,
            "state_hz": self.state_hz,
            "write_all_500hz": self.write_all_500hz,
            "alignment_clock": "mujoco data.time",
            "notes": [
                "force_500hz.csv is sampled from MuJoCo sensordata using data.time.",
                "state_30hz.csv is downsampled from the same simulation state clock.",
                "all_500hz.csv is the easiest file for aligned plotting.",
                "wall-parallel task assumes x-z alignment plane and y insertion axis.",
            ],
        }

        path = self.session_dir / "metadata.json"
        with path.open("w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=False)
