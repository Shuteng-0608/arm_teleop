#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Quality checker for one MuJoCo peg-in-hole HDF5 episode.

Checks:
1. HDF5 structure and required datasets.
2. Stream timing: state / force / image rates, monotonicity, gaps.
3. NaN / Inf.
4. Action recording:
   - /action
   - /actions/joint_pos_command
   - action-command consistency.
5. Joint torque statistics and limit violations.
6. Force/torque statistics and threshold violations.
7. Gravity compensation consistency:
   ft_wrench_raw - ft_wrench_gravity == ft_wrench
8. Action-qpos tracking error.
9. Terminal hold quality:
   - last window qvel
   - last window action change
   - last window qpos change
10. Reset contamination:
   - large qpos jump near the end.
11. Event markers:
   - task_success_site_reached
   - terminal_hold_start
   - auto_stop_task_success
   - joint_torque_over_limit
12. Image stream presence and basic validity.

Usage:
    python check_hdf5_episode_quality.py /path/to/episode.hdf5

Example:
    python check_hdf5_episode_quality.py \
        /home/stw/pangu/src/arm_teleop/data/peg_hole_auto_test/xxx/episode.hdf5 \
        --joint-torque-limits 20,20,20,20,10,10,10 \
        --force-limit 30 \
        --terminal-hold-sec 0.8
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import h5py
import numpy as np


# matplotlib is optional. The script still works without plots.
try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------

def resolve_episode_path(path_str: str) -> Path:
    p = Path(path_str).expanduser().resolve()
    if p.is_dir():
        candidate = p / "episode.hdf5"
        if candidate.exists():
            return candidate
        candidate = p / "episode.h5"
        if candidate.exists():
            return candidate
        raise FileNotFoundError(f"Directory does not contain episode.hdf5: {p}")
    if not p.exists():
        raise FileNotFoundError(f"Episode file not found: {p}")
    return p


def has_path(h5: h5py.File, path: str) -> bool:
    return path in h5


def read_array(h5: h5py.File, path: str, default=None):
    if path not in h5:
        return default
    return h5[path][()]


def read_str_array(h5: h5py.File, path: str) -> List[str]:
    if path not in h5:
        return []
    arr = h5[path][()]
    out = []
    for x in np.ravel(arr):
        if isinstance(x, bytes):
            out.append(x.decode("utf-8", errors="ignore"))
        else:
            out.append(str(x))
    return out


# def dataset_shape(h5: h5py.File, path: str):
#     if path not in h5:
#         return None
#     return tuple(h5[path].shape)
def dataset_shape(h5: h5py.File, path: str):
    if path not in h5:
        return None

    obj = h5[path]

    if isinstance(obj, h5py.Dataset):
        return {
            "type": "dataset",
            "shape": list(obj.shape),
            "dtype": str(obj.dtype),
        }

    if isinstance(obj, h5py.Group):
        return {
            "type": "group",
            "keys": list(obj.keys()),
        }

    return {
        "type": type(obj).__name__,
    }


def safe_norm(x: np.ndarray, axis: int = -1) -> np.ndarray:
    if x is None:
        return np.array([])
    return np.linalg.norm(np.asarray(x), axis=axis)


def finite_stats(x: np.ndarray) -> Dict[str, float]:
    x = np.asarray(x)
    x = x[np.isfinite(x)]
    if x.size == 0:
        return {
            "min": math.nan,
            "median": math.nan,
            "mean": math.nan,
            "std": math.nan,
            "p95": math.nan,
            "p99": math.nan,
            "max": math.nan,
            "rms": math.nan,
        }
    return {
        "min": float(np.min(x)),
        "median": float(np.median(x)),
        "mean": float(np.mean(x)),
        "std": float(np.std(x)),
        "p95": float(np.percentile(x, 95)),
        "p99": float(np.percentile(x, 99)),
        "max": float(np.max(x)),
        "rms": float(np.sqrt(np.mean(x * x))),
    }


def nan_inf_report(name: str, arr: Optional[np.ndarray]) -> Dict[str, Any]:
    if arr is None:
        return {"name": name, "exists": False}
    arr = np.asarray(arr)
    total = int(arr.size)
    bad = int(np.size(arr) - np.count_nonzero(np.isfinite(arr)))
    pct = 100.0 * bad / max(total, 1)
    return {
        "name": name,
        "exists": True,
        "shape": list(arr.shape),
        "bad": bad,
        "total": total,
        "pct": pct,
    }


def timing_report(name: str, t: Optional[np.ndarray], expected_hz: Optional[float] = None) -> Dict[str, Any]:
    if t is None:
        return {
            "name": name,
            "exists": False,
            "status": "MISSING",
        }

    t = np.asarray(t, dtype=float).reshape(-1)
    if t.size == 0:
        return {
            "name": name,
            "exists": True,
            "samples": 0,
            "status": "EMPTY",
        }

    out: Dict[str, Any] = {
        "name": name,
        "exists": True,
        "samples": int(t.size),
        "t_start": float(t[0]),
        "t_end": float(t[-1]),
        "duration": float(t[-1] - t[0]) if t.size >= 2 else 0.0,
    }

    if t.size < 2:
        out.update({
            "median_dt": math.nan,
            "mean_dt": math.nan,
            "max_dt": math.nan,
            "median_hz": math.nan,
            "mean_hz": math.nan,
            "monotonic": True,
            "duplicate_or_nonmono_count": 0,
            "gap_count": 0,
            "status": "TOO_SHORT",
        })
        return out

    dt = np.diff(t)
    pos_dt = dt[dt > 0]
    median_dt = float(np.median(pos_dt)) if pos_dt.size else math.nan
    mean_dt = float(np.mean(pos_dt)) if pos_dt.size else math.nan
    max_dt = float(np.max(dt)) if dt.size else math.nan
    median_hz = 1.0 / median_dt if median_dt > 0 else math.nan
    mean_hz = 1.0 / mean_dt if mean_dt > 0 else math.nan

    nonmono = int(np.count_nonzero(dt <= 0))
    gap_count = 0
    if np.isfinite(median_dt) and median_dt > 0:
        gap_count = int(np.count_nonzero(dt > 1.5 * median_dt))

    out.update({
        "median_dt": median_dt,
        "mean_dt": mean_dt,
        "max_dt": max_dt,
        "median_hz": median_hz,
        "mean_hz": mean_hz,
        "monotonic": bool(nonmono == 0),
        "duplicate_or_nonmono_count": nonmono,
        "gap_count": gap_count,
    })

    status = "OK"
    if nonmono > 0:
        status = "WARN"
    if gap_count > 0:
        status = "WARN"

    if expected_hz is not None and np.isfinite(median_hz):
        # Loose frequency check: +/- 20%.
        lo = 0.8 * expected_hz
        hi = 1.2 * expected_hz
        if not (lo <= median_hz <= hi):
            status = "WARN"

    out["status"] = status
    return out


def parse_limits(s: str, n: int = 7) -> np.ndarray:
    vals = [float(x.strip()) for x in s.split(",") if x.strip()]
    if len(vals) == 1:
        return np.full(n, vals[0], dtype=float)
    if len(vals) != n:
        raise ValueError(f"Expected 1 or {n} torque limits, got {len(vals)}: {vals}")
    return np.asarray(vals, dtype=float)


def ensure_out_dir(out_dir: Optional[str], episode_path: Path) -> Path:
    if out_dir:
        p = Path(out_dir).expanduser().resolve()
    else:
        p = episode_path.parent / "quality_check"
    p.mkdir(parents=True, exist_ok=True)
    return p


def get_first_existing(h5: h5py.File, paths: List[str]):
    for p in paths:
        if p in h5:
            return p, h5[p][()]
    return None, None


def write_text(path: Path, text: str):
    path.write_text(text, encoding="utf-8")


def add_issue(issues: List[Dict[str, str]], level: str, msg: str):
    issues.append({"level": level, "message": msg})


def issue_score(issues: List[Dict[str, str]]) -> str:
    n_fail = sum(1 for x in issues if x["level"] == "FAIL")
    n_warn = sum(1 for x in issues if x["level"] == "WARN")
    if n_fail > 0:
        return "FAIL"
    if n_warn > 0:
        return "WARN"
    return "PASS"


# ---------------------------------------------------------------------
# Image checks
# ---------------------------------------------------------------------

def check_images(h5: h5py.File, episode_dir: Path) -> Dict[str, Any]:
    out: Dict[str, Any] = {
        "exists": False,
        "cameras": {},
    }

    if "observations/images" not in h5:
        return out

    out["exists"] = True
    img_group = h5["observations/images"]

    for cam_name in img_group.keys():
        if cam_name == "camera_names":
            continue
        cam_obj = img_group[cam_name]
        cam_report: Dict[str, Any] = {}

        # Case 1: image dataset directly: [N,H,W,3]
        if isinstance(cam_obj, h5py.Dataset):
            shape = tuple(cam_obj.shape)
            cam_report["storage"] = "hdf5_dataset"
            cam_report["shape"] = list(shape)
            cam_report["dtype"] = str(cam_obj.dtype)
            cam_report["samples"] = int(shape[0]) if len(shape) > 0 else 0

            # Basic pixel sanity using first/middle/last frame only.
            if len(shape) >= 4 and shape[0] > 0:
                idxs = sorted(set([0, shape[0] // 2, shape[0] - 1]))
                means = []
                stds = []
                for idx in idxs:
                    frame = cam_obj[idx]
                    means.append(float(np.mean(frame)))
                    stds.append(float(np.std(frame)))
                cam_report["sample_pixel_mean"] = means
                cam_report["sample_pixel_std"] = stds

        # Case 2: group with file_paths dataset.
        elif isinstance(cam_obj, h5py.Group):
            cam_report["storage"] = "group"
            if "file_paths" in cam_obj:
                paths = read_str_array(h5, f"observations/images/{cam_name}/file_paths")
                missing = 0
                for fp in paths:
                    p = Path(fp)
                    if not p.is_absolute():
                        p = episode_dir / fp
                    if not p.exists():
                        missing += 1

                cam_report["samples"] = len(paths)
                cam_report["file_paths"] = True
                cam_report["missing_files"] = int(missing)
            else:
                cam_report["keys"] = list(cam_obj.keys())
                cam_report["samples"] = None

        out["cameras"][cam_name] = cam_report

    return out


# ---------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------

def save_plot_time_series(out_dir: Path, name: str, t: np.ndarray, ys: Dict[str, np.ndarray],
                          ylabel: str, title: str):
    if not HAS_MPL:
        return

    plt.figure(figsize=(11, 5))
    for label, y in ys.items():
        plt.plot(t, y, label=label, linewidth=1.2)
    plt.xlabel("time [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    if len(ys) <= 10:
        plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / f"{name}.png", dpi=160)
    plt.close()


# ---------------------------------------------------------------------
# Main check
# ---------------------------------------------------------------------

def analyze_episode(args) -> Tuple[Dict[str, Any], str]:
    episode_path = resolve_episode_path(args.episode)
    episode_dir = episode_path.parent
    out_dir = ensure_out_dir(args.out, episode_path)

    issues: List[Dict[str, str]] = []
    report: Dict[str, Any] = {
        "episode_path": str(episode_path),
        "episode_dir": str(episode_dir),
        "out_dir": str(out_dir),
        "issues": issues,
        "streams": {},
        "nan_inf": {},
        "shapes": {},
        "events": [],
        "metrics": {},
    }

    torque_limits = parse_limits(args.joint_torque_limits, n=7)

    with h5py.File(episode_path, "r") as h5:
        # -----------------------------------------------------------------
        # Root attrs
        # -----------------------------------------------------------------
        root_attrs = {}
        for k, v in h5.attrs.items():
            try:
                if isinstance(v, bytes):
                    root_attrs[k] = v.decode("utf-8")
                elif isinstance(v, np.ndarray):
                    root_attrs[k] = v.tolist()
                else:
                    root_attrs[k] = str(v)
            except Exception:
                root_attrs[k] = repr(v)
        report["root_attrs"] = root_attrs

        # -----------------------------------------------------------------
        # Required / optional datasets
        # -----------------------------------------------------------------
        paths_to_check = [
            "observations/joint_pos",
            "observations/joint_vel",
            "observations/joint_torque",
            "observations/ee_pose",
            "observations/ft_wrench",
            "observations/ft_wrench_raw",
            "observations/ft_wrench_gravity",
            "action",
            "actions/joint_pos_command",
            "timestamps/state",
            "timestamps/state_episode",
            "timestamps/force",
            "timestamps/force_episode",
            "timestamps/image",
            "timestamps/image_episode",
            "observations/images",
            "episode_metadata/joint_names",
            "events/names",
        ]

        for p in paths_to_check:
            report["shapes"][p] = dataset_shape(h5, p) if p in h5 else None

        required = [
            "observations/joint_pos",
            "observations/joint_vel",
            "observations/joint_torque",
            "observations/ft_wrench",
            "action",
        ]
        for p in required:
            if p not in h5:
                add_issue(issues, "FAIL", f"Missing required dataset: {p}")

        # -----------------------------------------------------------------
        # Load main arrays
        # -----------------------------------------------------------------
        qpos = read_array(h5, "observations/joint_pos")
        qvel = read_array(h5, "observations/joint_vel")
        jtau = read_array(h5, "observations/joint_torque")
        ee_pose = read_array(h5, "observations/ee_pose")
        ft = read_array(h5, "observations/ft_wrench")
        ft_raw = read_array(h5, "observations/ft_wrench_raw")
        ft_grav = read_array(h5, "observations/ft_wrench_gravity")
        action = read_array(h5, "action")
        cmd = read_array(h5, "actions/joint_pos_command")

        t_state_path, t_state = get_first_existing(h5, [
            "timestamps/state_episode",
            "timestamps/state",
        ])
        t_force_path, t_force = get_first_existing(h5, [
            "timestamps/force_episode",
            "timestamps/force",
        ])
        t_image_path, t_image = get_first_existing(h5, [
            "timestamps/image_episode",
            "timestamps/image",
        ])

        if t_state is not None:
            t_state = np.asarray(t_state, dtype=float).reshape(-1)
        if t_force is not None:
            t_force = np.asarray(t_force, dtype=float).reshape(-1)
        if t_image is not None:
            t_image = np.asarray(t_image, dtype=float).reshape(-1)

        # -----------------------------------------------------------------
        # Timing
        # -----------------------------------------------------------------
        report["streams"]["state"] = timing_report("state", t_state, expected_hz=args.expected_state_hz)
        report["streams"]["force"] = timing_report("force", t_force, expected_hz=args.expected_force_hz)
        report["streams"]["image"] = timing_report("image", t_image, expected_hz=args.expected_image_hz)

        for stream_name, stream in report["streams"].items():
            if stream.get("status") == "MISSING":
                add_issue(issues, "WARN", f"Missing timestamp stream: {stream_name}")
            elif stream.get("status") == "WARN":
                add_issue(issues, "WARN", f"Timestamp warning in stream: {stream_name}")

        duration = report["streams"]["state"].get("duration", None)
        if duration is not None and np.isfinite(duration):
            if duration < args.min_duration:
                add_issue(issues, "WARN", f"Episode duration too short: {duration:.3f}s < {args.min_duration:.3f}s")
            if duration > args.max_duration:
                add_issue(issues, "WARN", f"Episode duration too long: {duration:.3f}s > {args.max_duration:.3f}s")

        # -----------------------------------------------------------------
        # Shape consistency
        # -----------------------------------------------------------------
        n_state = qpos.shape[0] if qpos is not None and qpos.ndim >= 1 else None
        n_force = ft.shape[0] if ft is not None and ft.ndim >= 1 else None

        if n_state is not None and t_state is not None and len(t_state) != n_state:
            add_issue(issues, "FAIL", f"state timestamp length mismatch: {len(t_state)} vs qpos {n_state}")

        if n_force is not None and t_force is not None and len(t_force) != n_force:
            add_issue(issues, "FAIL", f"force timestamp length mismatch: {len(t_force)} vs ft_wrench {n_force}")

        for name, arr in [
            ("qpos", qpos),
            ("qvel", qvel),
            ("joint_torque", jtau),
            ("ee_pose", ee_pose),
            ("ft_wrench", ft),
            ("ft_wrench_raw", ft_raw),
            ("ft_wrench_gravity", ft_grav),
            ("action", action),
            ("joint_pos_command", cmd),
        ]:
            report["nan_inf"][name] = nan_inf_report(name, arr)
            if report["nan_inf"][name].get("bad", 0) > 0:
                add_issue(issues, "FAIL", f"{name} contains NaN/Inf")

        # -----------------------------------------------------------------
        # Events
        # -----------------------------------------------------------------
        events = read_str_array(h5, "events/names")
        report["events"] = events

        expected_success_events = [
            "task_success_site_reached",
            "terminal_hold_start",
            "auto_stop_task_success",
        ]
        report["metrics"]["events"] = {
            "has_task_success_site_reached": "task_success_site_reached" in events,
            "has_terminal_hold_start": "terminal_hold_start" in events,
            "has_auto_stop_task_success": "auto_stop_task_success" in events,
            "has_joint_torque_over_limit": "joint_torque_over_limit" in events,
            "all_events": events,
        }

        if args.expect_auto_success:
            for ev in expected_success_events:
                if ev not in events:
                    add_issue(issues, "WARN", f"Expected auto-success event missing: {ev}")

        if "joint_torque_over_limit" in events:
            add_issue(issues, "WARN", "Event contains joint_torque_over_limit")

        # -----------------------------------------------------------------
        # Action consistency
        # -----------------------------------------------------------------
        if action is not None and cmd is not None:
            if action.shape != cmd.shape:
                add_issue(issues, "FAIL", f"/action and /actions/joint_pos_command shape mismatch: {action.shape} vs {cmd.shape}")
            else:
                max_action_cmd_err = float(np.max(np.abs(action - cmd))) if action.size else 0.0
                report["metrics"]["action_cmd_max_abs_error"] = max_action_cmd_err
                if max_action_cmd_err > 1e-9:
                    add_issue(issues, "WARN", f"/action differs from /actions/joint_pos_command: max error {max_action_cmd_err:.3e}")

        if action is not None and qpos is not None:
            if action.shape[0] != qpos.shape[0]:
                add_issue(issues, "FAIL", f"action/qpos sample mismatch: {action.shape[0]} vs {qpos.shape[0]}")
            else:
                action_qpos_err = np.abs(action[:, :qpos.shape[1]] - qpos)
                report["metrics"]["action_qpos_tracking_abs"] = finite_stats(action_qpos_err.reshape(-1))
                report["metrics"]["action_qpos_tracking_per_joint_mean"] = np.mean(action_qpos_err, axis=0).tolist()
                report["metrics"]["action_qpos_tracking_per_joint_p95"] = np.percentile(action_qpos_err, 95, axis=0).tolist()

                if np.nanmax(action_qpos_err) > args.max_tracking_error:
                    add_issue(
                        issues,
                        "WARN",
                        f"Large action-qpos tracking error: max {np.nanmax(action_qpos_err):.4f} rad > {args.max_tracking_error:.4f} rad"
                    )

        # -----------------------------------------------------------------
        # Joint torque quality
        # -----------------------------------------------------------------
        if jtau is not None:
            abs_tau = np.abs(jtau)
            per_joint = []
            for j in range(min(abs_tau.shape[1], torque_limits.shape[0])):
                x = abs_tau[:, j]
                st = finite_stats(x)
                st["limit"] = float(torque_limits[j])
                over = x > torque_limits[j]
                st["over_count"] = int(np.count_nonzero(over))

                over_sec = math.nan
                if t_state is not None and len(t_state) == len(x) and len(t_state) >= 2:
                    med_dt = float(np.median(np.diff(t_state)))
                    over_sec = float(np.count_nonzero(over) * med_dt)
                st["over_sec_est"] = over_sec

                if st["over_count"] > 0:
                    add_issue(
                        issues,
                        "WARN",
                        f"joint_{j+1} torque over limit: max={st['max']:.3f}, limit={st['limit']:.3f}, over_count={st['over_count']}, over_sec≈{over_sec:.3f}"
                    )
                per_joint.append(st)

            report["metrics"]["joint_torque_abs_per_joint"] = per_joint

        # -----------------------------------------------------------------
        # Force / torque quality
        # -----------------------------------------------------------------
        if ft is not None and ft.ndim == 2 and ft.shape[1] >= 6:
            force_mag = safe_norm(ft[:, :3], axis=1)
            torque_mag = safe_norm(ft[:, 3:6], axis=1)

            report["metrics"]["force_magnitude"] = finite_stats(force_mag)
            report["metrics"]["ft_torque_magnitude"] = finite_stats(torque_mag)

            if np.nanmax(force_mag) > args.force_limit:
                add_issue(
                    issues,
                    "WARN",
                    f"Force magnitude over limit: max={np.nanmax(force_mag):.3f} N > {args.force_limit:.3f} N"
                )

            if np.nanmax(torque_mag) > args.ft_torque_limit:
                add_issue(
                    issues,
                    "WARN",
                    f"FT torque magnitude over limit: max={np.nanmax(torque_mag):.3f} Nm > {args.ft_torque_limit:.3f} Nm"
                )

        # Gravity compensation consistency
        if ft is not None and ft_raw is not None and ft_grav is not None:
            if ft.shape == ft_raw.shape == ft_grav.shape:
                comp_err = np.abs(ft_raw - ft_grav - ft)
                max_comp_err = float(np.max(comp_err)) if comp_err.size else 0.0
                report["metrics"]["gravity_compensation_max_abs_error"] = max_comp_err
                if max_comp_err > args.gravity_comp_tolerance:
                    add_issue(
                        issues,
                        "WARN",
                        f"Gravity compensation inconsistency: max abs(raw-gravity-comp)={max_comp_err:.3e}"
                    )
            else:
                add_issue(
                    issues,
                    "WARN",
                    f"ft_wrench/raw/gravity shape mismatch: {getattr(ft,'shape',None)}, {getattr(ft_raw,'shape',None)}, {getattr(ft_grav,'shape',None)}"
                )

        # -----------------------------------------------------------------
        # Terminal hold quality
        # -----------------------------------------------------------------
        terminal_report: Dict[str, Any] = {}
        if qpos is not None and qvel is not None and action is not None and t_state is not None:
            if len(t_state) == qpos.shape[0] and qpos.shape[0] >= 3:
                t_end = float(t_state[-1])
                idx = np.where(t_state >= t_end - args.terminal_hold_sec)[0]
                if idx.size >= 3:
                    qpos_tail = qpos[idx]
                    qvel_tail = qvel[idx]
                    action_tail = action[idx]

                    qvel_norm = safe_norm(qvel_tail, axis=1)
                    qpos_step = safe_norm(np.diff(qpos_tail, axis=0), axis=1)
                    action_step = safe_norm(np.diff(action_tail, axis=0), axis=1)

                    terminal_report["window_sec"] = args.terminal_hold_sec
                    terminal_report["samples"] = int(idx.size)
                    terminal_report["qvel_norm"] = finite_stats(qvel_norm)
                    terminal_report["qpos_step_norm"] = finite_stats(qpos_step)
                    terminal_report["action_step_norm"] = finite_stats(action_step)

                    if np.nanmax(qvel_norm) > args.terminal_qvel_limit:
                        add_issue(
                            issues,
                            "WARN",
                            f"Terminal hold qvel not stable: max ||qvel||={np.nanmax(qvel_norm):.4f} > {args.terminal_qvel_limit:.4f}"
                        )

                    if np.nanmax(action_step) > args.terminal_action_step_limit:
                        add_issue(
                            issues,
                            "WARN",
                            f"Terminal hold action still changing: max step={np.nanmax(action_step):.4f} > {args.terminal_action_step_limit:.4f}"
                        )

                    if np.nanmax(qpos_step) > args.terminal_qpos_step_limit:
                        add_issue(
                            issues,
                            "WARN",
                            f"Terminal hold qpos still moving: max step={np.nanmax(qpos_step):.4f} > {args.terminal_qpos_step_limit:.4f}"
                        )
                else:
                    add_issue(issues, "WARN", "Not enough samples for terminal hold check")
        

        # Terminal force quality
        if ft is not None and t_force is not None and len(t_force) == ft.shape[0]:
            t_force_end = float(t_force[-1])
            fidx = np.where(t_force >= t_force_end - args.terminal_hold_sec)[0]

            if fidx.size >= 3 and ft.ndim == 2 and ft.shape[1] >= 6:
                ft_tail = ft[fidx]
                terminal_force_mag = np.linalg.norm(ft_tail[:, :3], axis=1)
                terminal_torque_mag = np.linalg.norm(ft_tail[:, 3:6], axis=1)

                terminal_report["force_magnitude"] = finite_stats(terminal_force_mag)
                terminal_report["ft_torque_magnitude"] = finite_stats(terminal_torque_mag)

                if np.nanmax(terminal_force_mag) > args.force_limit:
                    add_issue(
                        issues,
                        "WARN",
                        f"Terminal hold force too high: max={np.nanmax(terminal_force_mag):.3f} N > {args.force_limit:.3f} N"
                    )

        report["metrics"]["terminal_hold"] = terminal_report

        # -----------------------------------------------------------------
        # Reset contamination / sudden jumps
        # -----------------------------------------------------------------
        if qpos is not None and t_state is not None and len(t_state) == qpos.shape[0] and qpos.shape[0] >= 2:
            qpos_step_all = safe_norm(np.diff(qpos, axis=0), axis=1)
            max_step = float(np.max(qpos_step_all))
            max_step_idx = int(np.argmax(qpos_step_all))
            max_step_time = float(t_state[max_step_idx + 1])

            report["metrics"]["qpos_step_norm"] = {
                **finite_stats(qpos_step_all),
                "max_step_time": max_step_time,
            }

            if max_step > args.max_qpos_step:
                add_issue(
                    issues,
                    "WARN",
                    f"Large qpos jump detected: max step={max_step:.4f} rad at t={max_step_time:.3f}s. Possible reset contamination or discontinuity."
                )

        # -----------------------------------------------------------------
        # Metadata task error, if available
        # -----------------------------------------------------------------
        meta_metrics = {}
        for name in [
            "initial_task_error_xyz",
            "final_task_error_xyz",
            "initial_peg_tip_pos",
            "final_peg_tip_pos",
            "initial_hole_center_pos",
            "final_hole_center_pos",
            "initial_hole_goal_pos",
            "final_hole_goal_pos",
        ]:
            p = f"episode_metadata/{name}"
            if p in h5:
                val = np.asarray(h5[p][()])
                meta_metrics[name] = val.tolist()

        if "final_task_error_xyz" in meta_metrics:
            e = np.asarray(meta_metrics["final_task_error_xyz"], dtype=float).reshape(-1)
            if e.size >= 3:
                xz = float(np.linalg.norm([e[0], e[2]]))
                norm = float(np.linalg.norm(e[:3]))
                meta_metrics["final_task_error_xz_norm"] = xz
                meta_metrics["final_task_error_xyz_norm"] = norm
                if xz > args.final_align_error_limit:
                    add_issue(
                        issues,
                        "WARN",
                        f"Final x-z alignment error large: {xz:.4f} m > {args.final_align_error_limit:.4f} m"
                    )

        report["metrics"]["metadata"] = meta_metrics

        # -----------------------------------------------------------------
        # Image check
        # -----------------------------------------------------------------
        img_report = check_images(h5, episode_dir)
        report["metrics"]["images"] = img_report

        if not img_report["exists"]:
            add_issue(issues, "WARN", "No observations/images group found")
        else:
            for cam, cam_report in img_report.get("cameras", {}).items():
                samples = cam_report.get("samples", 0)
                if samples == 0:
                    add_issue(issues, "WARN", f"Camera {cam} has 0 image samples")
                if cam_report.get("missing_files", 0) > 0:
                    add_issue(issues, "WARN", f"Camera {cam} has missing image files: {cam_report['missing_files']}")

                if t_image is not None and samples is not None and samples > 0:
                    # If image timestamp is shared per camera, expect samples ~= len(t_image).
                    if abs(samples - len(t_image)) > 2:
                        add_issue(
                            issues,
                            "WARN",
                            f"Camera {cam} sample count differs from image timestamps: {samples} vs {len(t_image)}"
                        )

        # -----------------------------------------------------------------
        # Plots
        # -----------------------------------------------------------------
        if not args.no_plots and HAS_MPL:
            if jtau is not None and t_state is not None and len(t_state) == jtau.shape[0]:
                ys = {}
                for j in range(min(jtau.shape[1], 7)):
                    ys[f"j{j+1}"] = jtau[:, j]
                save_plot_time_series(
                    out_dir,
                    "joint_torque",
                    t_state,
                    ys,
                    ylabel="joint torque [Nm]",
                    title="Joint Torque",
                )

            if ft is not None and ft.ndim == 2 and ft.shape[1] >= 6 and t_force is not None and len(t_force) == ft.shape[0]:
                force_mag = safe_norm(ft[:, :3], axis=1)
                torque_mag = safe_norm(ft[:, 3:6], axis=1)
                save_plot_time_series(
                    out_dir,
                    "force_torque_magnitude",
                    t_force,
                    {
                        "force_norm_N": force_mag,
                        "torque_norm_Nm": torque_mag,
                    },
                    ylabel="magnitude",
                    title="FT Wrench Magnitude",
                )

            if action is not None and qpos is not None and t_state is not None and len(t_state) == qpos.shape[0]:
                err = safe_norm(action[:, :qpos.shape[1]] - qpos, axis=1)
                save_plot_time_series(
                    out_dir,
                    "action_qpos_tracking_error",
                    t_state,
                    {"||action-qpos||": err},
                    ylabel="rad",
                    title="Action-Qpos Tracking Error",
                )

            if qvel is not None and action is not None and t_state is not None and len(t_state) == qvel.shape[0]:
                qvel_norm = safe_norm(qvel, axis=1)
                action_step = np.zeros_like(qvel_norm)
                if action.shape[0] >= 2:
                    action_step[1:] = safe_norm(np.diff(action, axis=0), axis=1)
                save_plot_time_series(
                    out_dir,
                    "terminal_motion_check",
                    t_state,
                    {
                        "||qvel||": qvel_norm,
                        "||delta_action||": action_step,
                    },
                    ylabel="norm",
                    title="Motion / Action Stability",
                )

    # -----------------------------------------------------------------
    # Final decision
    # -----------------------------------------------------------------
    verdict = issue_score(issues)
    report["verdict"] = verdict

    # Save JSON
    json_path = out_dir / "quality_summary.json"
    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    # Save human-readable TXT
    text = format_text_report(report)
    txt_path = out_dir / "quality_report.txt"
    write_text(txt_path, text)

    return report, text


def fmt_float(x, nd=4):
    try:
        if x is None or not np.isfinite(float(x)):
            return "nan"
        return f"{float(x):.{nd}f}"
    except Exception:
        return "nan"


def format_text_report(report: Dict[str, Any]) -> str:
    lines: List[str] = []

    lines.append("=" * 80)
    lines.append("MuJoCo Peg-in-Hole HDF5 Episode Quality Report")
    lines.append("=" * 80)
    lines.append(f"Episode: {report['episode_path']}")
    lines.append(f"Output : {report['out_dir']}")
    lines.append(f"Verdict: {report['verdict']}")
    lines.append("")

    lines.append("=== Issues ===")
    if not report["issues"]:
        lines.append("PASS: no issue detected.")
    else:
        for item in report["issues"]:
            lines.append(f"[{item['level']}] {item['message']}")
    lines.append("")

    lines.append("=== Root attributes ===")
    for k, v in report.get("root_attrs", {}).items():
        lines.append(f"{k}: {v}")
    lines.append("")

    lines.append("=== Dataset shapes ===")
    for k, v in report.get("shapes", {}).items():
        lines.append(f"{k}: {v}")
    lines.append("")

    lines.append("=== Stream timing ===")
    for name, r in report.get("streams", {}).items():
        if not r.get("exists", False):
            lines.append(f"[{name}] MISSING")
            continue
        lines.append(
            f"[{name}] samples={r.get('samples')} "
            f"duration={fmt_float(r.get('duration'), 3)}s "
            f"median_hz={fmt_float(r.get('median_hz'), 2)} "
            f"mean_hz={fmt_float(r.get('mean_hz'), 2)} "
            f"max_dt={fmt_float(r.get('max_dt'), 4)} "
            f"monotonic={r.get('monotonic')} "
            f"gaps={r.get('gap_count')} "
            f"status={r.get('status')}"
        )
    lines.append("")

    lines.append("=== NaN / Inf ===")
    for name, r in report.get("nan_inf", {}).items():
        if not r.get("exists", False):
            lines.append(f"{name}: missing")
        else:
            lines.append(
                f"{name}: shape={r.get('shape')} bad={r.get('bad')}/{r.get('total')} "
                f"({fmt_float(r.get('pct'), 4)}%)"
            )
    lines.append("")

    metrics = report.get("metrics", {})

    lines.append("=== Events ===")
    ev = metrics.get("events", {})
    for k in [
        "has_task_success_site_reached",
        "has_terminal_hold_start",
        "has_auto_stop_task_success",
        "has_joint_torque_over_limit",
    ]:
        if k in ev:
            lines.append(f"{k}: {ev[k]}")
    lines.append(f"all_events: {ev.get('all_events', [])}")
    lines.append("")

    lines.append("=== Action / command ===")
    if "action_cmd_max_abs_error" in metrics:
        lines.append(f"max abs(/action - /actions/joint_pos_command): {metrics['action_cmd_max_abs_error']:.6e}")
    tr = metrics.get("action_qpos_tracking_abs")
    if tr:
        lines.append(
            "action-qpos abs tracking error: "
            f"median={fmt_float(tr.get('median'))}, "
            f"p95={fmt_float(tr.get('p95'))}, "
            f"p99={fmt_float(tr.get('p99'))}, "
            f"max={fmt_float(tr.get('max'))}"
        )
        if "action_qpos_tracking_per_joint_mean" in metrics:
            lines.append(f"per-joint mean: {np.round(metrics['action_qpos_tracking_per_joint_mean'], 4).tolist()}")
            lines.append(f"per-joint p95 : {np.round(metrics['action_qpos_tracking_per_joint_p95'], 4).tolist()}")
    lines.append("")

    lines.append("=== Joint torque ===")
    per_joint = metrics.get("joint_torque_abs_per_joint", [])
    for i, st in enumerate(per_joint):
        lines.append(
            f"joint_{i+1}: "
            f"rms={fmt_float(st.get('rms'))}, "
            f"p95={fmt_float(st.get('p95'))}, "
            f"p99={fmt_float(st.get('p99'))}, "
            f"max={fmt_float(st.get('max'))}, "
            f"limit={fmt_float(st.get('limit'))}, "
            f"over_n={st.get('over_count')}, "
            f"over_s≈{fmt_float(st.get('over_sec_est'), 3)}"
        )
    lines.append("")

    lines.append("=== FT wrench ===")
    fm = metrics.get("force_magnitude")
    tm = metrics.get("ft_torque_magnitude")
    if fm:
        lines.append(
            "force magnitude [N]: "
            f"median={fmt_float(fm.get('median'))}, "
            f"p95={fmt_float(fm.get('p95'))}, "
            f"p99={fmt_float(fm.get('p99'))}, "
            f"max={fmt_float(fm.get('max'))}"
        )
    if tm:
        lines.append(
            "FT torque magnitude [Nm]: "
            f"median={fmt_float(tm.get('median'))}, "
            f"p95={fmt_float(tm.get('p95'))}, "
            f"p99={fmt_float(tm.get('p99'))}, "
            f"max={fmt_float(tm.get('max'))}"
        )
    if "gravity_compensation_max_abs_error" in metrics:
        lines.append(f"gravity compensation max abs error: {metrics['gravity_compensation_max_abs_error']:.6e}")
    lines.append("")

    lines.append("=== Terminal hold ===")
    th = metrics.get("terminal_hold", {})
    if th:
        lines.append(f"window_sec: {th.get('window_sec')}")
        lines.append(f"samples: {th.get('samples')}")
        for key in ["qvel_norm", "qpos_step_norm", "action_step_norm"]:
            st = th.get(key)
            if st:
                lines.append(
                    f"{key}: median={fmt_float(st.get('median'))}, "
                    f"p95={fmt_float(st.get('p95'))}, "
                    f"max={fmt_float(st.get('max'))}"
                )
    else:
        lines.append("terminal hold metrics unavailable.")
    lines.append("")

    lines.append("=== Qpos discontinuity / reset contamination ===")
    qstep = metrics.get("qpos_step_norm")
    if qstep:
        lines.append(
            f"qpos step norm: median={fmt_float(qstep.get('median'))}, "
            f"p95={fmt_float(qstep.get('p95'))}, "
            f"max={fmt_float(qstep.get('max'))}, "
            f"max_time={fmt_float(qstep.get('max_step_time'), 3)}s"
        )
    lines.append("")

    lines.append("=== Metadata task metrics ===")
    md = metrics.get("metadata", {})
    if not md:
        lines.append("No task metadata found.")
    else:
        for k, v in md.items():
            lines.append(f"{k}: {v}")
    lines.append("")

    lines.append("=== Images ===")
    img = metrics.get("images", {})
    if not img.get("exists", False):
        lines.append("No image group found.")
    else:
        for cam, cr in img.get("cameras", {}).items():
            lines.append(f"{cam}: {cr}")
    lines.append("")

    lines.append("=" * 80)
    lines.append("Interpretation")
    lines.append("=" * 80)
    if report["verdict"] == "PASS":
        lines.append("PASS: This episode is structurally clean under the current thresholds.")
    elif report["verdict"] == "WARN":
        lines.append("WARN: This episode is usable only after manual review. Check force/torque peaks, terminal hold, and image quality.")
    else:
        lines.append("FAIL: This episode has critical issues and should usually be discarded.")
    lines.append("")

    return "\n".join(lines)


def build_argparser():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=str, help="Path to episode.hdf5 or episode directory.")
    parser.add_argument("--out", type=str, default=None, help="Output directory. Default: <episode_dir>/quality_check")

    parser.add_argument("--expected-state-hz", type=float, default=30.0)
    parser.add_argument("--expected-force-hz", type=float, default=500.0)
    parser.add_argument("--expected-image-hz", type=float, default=30.0)

    parser.add_argument("--min-duration", type=float, default=5.0)
    parser.add_argument("--max-duration", type=float, default=40.0)

    parser.add_argument(
        "--joint-torque-limits",
        type=str,
        default="20,20,20,20,10,10,10",
        help="Scalar or 7 comma-separated joint torque quality limits in Nm.",
    )

    parser.add_argument("--force-limit", type=float, default=30.0, help="Force magnitude warning limit [N].")
    parser.add_argument("--ft-torque-limit", type=float, default=5.0, help="FT torque magnitude warning limit [Nm].")
    parser.add_argument("--gravity-comp-tolerance", type=float, default=1e-6)

    parser.add_argument("--max-tracking-error", type=float, default=0.50, help="Max |action-qpos| warning threshold [rad].")
    parser.add_argument("--max-qpos-step", type=float, default=0.30, help="Large qpos jump warning threshold [rad/sample].")

    parser.add_argument("--terminal-hold-sec", type=float, default=0.8)
    parser.add_argument("--terminal-qvel-limit", type=float, default=0.20, help="Terminal max ||qvel|| warning threshold.")
    parser.add_argument("--terminal-action-step-limit", type=float, default=0.03, help="Terminal max ||delta action|| warning threshold.")
    parser.add_argument("--terminal-qpos-step-limit", type=float, default=0.03, help="Terminal max ||delta qpos|| warning threshold.")

    parser.add_argument("--final-align-error-limit", type=float, default=0.010, help="Final x-z alignment error warning threshold [m].")
    parser.add_argument("--expect-auto-success", action="store_true", help="Warn if auto-success events are missing.")
    parser.add_argument("--no-plots", action="store_true")

    return parser


def main():
    parser = build_argparser()
    args = parser.parse_args()

    report, text = analyze_episode(args)

    print(text)
    print("")
    print("Saved:")
    print("  ", Path(report["out_dir"]) / "quality_report.txt")
    print("  ", Path(report["out_dir"]) / "quality_summary.json")
    if HAS_MPL and not args.no_plots:
        print("  plots in:", report["out_dir"])
    elif not HAS_MPL:
        print("  matplotlib not available; plots skipped.")


if __name__ == "__main__":
    main()