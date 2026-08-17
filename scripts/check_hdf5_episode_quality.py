#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Quality checker for MuJoCo peg-in-hole HDF5 episodes with actuator forcerange limits.

Designed for current actuator forcerange:
    [20, 20, 20, 20, 10, 10, 10] Nm

Main idea:
    After limiting forcerange, "torque over limit" is no longer the main signal.
    We should check:
        1. actuator saturation duration
        2. action-qpos tracking error
        3. force magnitude and force duration
        4. whether high-force contact still has insertion progress
        5. terminal hold stability
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Any, List, Optional, Tuple

import h5py
import numpy as np


try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except Exception:
    HAS_MPL = False


# ---------------------------------------------------------------------
# Basic helpers
# ---------------------------------------------------------------------

def parse_csv_floats(s: str, expected: Optional[int] = None) -> np.ndarray:
    vals = [float(x.strip()) for x in s.split(",") if x.strip()]
    if expected is not None and len(vals) not in (1, expected):
        raise ValueError(f"Expected 1 or {expected} values, got {len(vals)}: {vals}")
    if expected is not None and len(vals) == 1:
        return np.full(expected, vals[0], dtype=float)
    return np.asarray(vals, dtype=float)


def decode_str_array(arr) -> List[str]:
    out = []
    for x in np.ravel(arr):
        if isinstance(x, bytes):
            out.append(x.decode("utf-8", errors="ignore"))
        else:
            out.append(str(x))
    return out


def get_array(h5: h5py.File, path: str, default=None):
    if path not in h5:
        return default
    return h5[path][()]


def get_shape_info(h5: h5py.File, path: str):
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
    return {"type": type(obj).__name__}


def finite_stats(x) -> Dict[str, float]:
    x = np.asarray(x, dtype=float)
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


def timing_stats(t, expected_hz: Optional[float]) -> Dict[str, Any]:
    if t is None:
        return {"exists": False, "status": "MISSING"}

    t = np.asarray(t, dtype=float).reshape(-1)
    if len(t) < 2:
        return {
            "exists": True,
            "samples": int(len(t)),
            "status": "TOO_SHORT",
        }

    dt = np.diff(t)
    positive_dt = dt[dt > 0]
    median_dt = float(np.median(positive_dt)) if positive_dt.size > 0 else math.nan
    mean_dt = float(np.mean(positive_dt)) if positive_dt.size > 0 else math.nan
    median_hz = 1.0 / median_dt if median_dt > 0 else math.nan
    mean_hz = 1.0 / mean_dt if mean_dt > 0 else math.nan

    nonmono = int(np.sum(dt <= 0))
    gap_count = 0
    if np.isfinite(median_dt) and median_dt > 0:
        gap_count = int(np.sum(dt > 1.5 * median_dt))

    status = "OK"
    if nonmono > 0 or gap_count > 0:
        status = "WARN"

    if expected_hz is not None and np.isfinite(median_hz):
        if not (0.8 * expected_hz <= median_hz <= 1.2 * expected_hz):
            status = "WARN"

    return {
        "exists": True,
        "samples": int(len(t)),
        "duration": float(t[-1] - t[0]),
        "t_start": float(t[0]),
        "t_end": float(t[-1]),
        "median_dt": median_dt,
        "mean_dt": mean_dt,
        "max_dt": float(np.max(dt)),
        "median_hz": median_hz,
        "mean_hz": mean_hz,
        "monotonic": bool(nonmono == 0),
        "nonmono_count": nonmono,
        "gap_count": gap_count,
        "status": status,
    }


def add_issue(issues: List[Dict[str, str]], level: str, msg: str):
    issues.append({"level": level, "message": msg})


def verdict_from_issues(issues: List[Dict[str, str]]) -> str:
    if any(x["level"] == "FAIL" for x in issues):
        return "FAIL"
    if any(x["level"] == "WARN" for x in issues):
        return "WARN"
    return "PASS"


def continuous_segments(mask: np.ndarray) -> List[np.ndarray]:
    idx = np.where(mask)[0]
    if len(idx) == 0:
        return []
    split_points = np.where(np.diff(idx) > 1)[0] + 1
    return np.split(idx, split_points)


def segment_summary(t: np.ndarray, values: np.ndarray, mask: np.ndarray, sample_hz: float):
    segs = continuous_segments(mask)
    if not segs:
        return {
            "count": 0,
            "duration_sec": 0.0,
            "longest_duration_sec": 0.0,
            "longest_start": None,
            "longest_end": None,
            "longest_max": None,
        }

    longest = max(segs, key=len)
    return {
        "count": int(np.sum(mask)),
        "duration_sec": float(np.sum(mask) / sample_hz),
        "longest_duration_sec": float(len(longest) / sample_hz),
        "longest_start": float(t[longest[0]]),
        "longest_end": float(t[longest[-1]]),
        "longest_max": float(np.max(values[longest])),
    }


def nearest_values(query_t: np.ndarray, source_t: np.ndarray, source_values: np.ndarray) -> np.ndarray:
    query_t = np.asarray(query_t, dtype=float).reshape(-1)
    source_t = np.asarray(source_t, dtype=float).reshape(-1)
    source_values = np.asarray(source_values)

    right = np.searchsorted(source_t, query_t, side="left")
    right = np.clip(right, 0, len(source_t) - 1)
    left = np.clip(right - 1, 0, len(source_t) - 1)
    use_left = np.abs(query_t - source_t[left]) <= np.abs(source_t[right] - query_t)
    idx = np.where(use_left, left, right)
    return source_values[idx]


def force_aware_filter_diagnostics(
    force_mag: np.ndarray,
    t_force: np.ndarray,
    action: Optional[np.ndarray],
    command: Optional[np.ndarray],
    t_state: Optional[np.ndarray],
    events: List[str],
    event_times: List[float],
    expected_force_hz: float,
    force_filter_bands: np.ndarray,
) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "status": "OK",
        "warnings": [],
        "force_bands": {},
        "thresholds": force_filter_bands.tolist(),
        "action_source": None,
        "action_step_comparison": {},
        "high_force_hold_events": {},
    }

    force_mag = np.asarray(force_mag, dtype=float).reshape(-1)
    t_force = np.asarray(t_force, dtype=float).reshape(-1)
    if len(force_mag) == 0 or len(t_force) == 0 or len(force_mag) != len(t_force):
        result["status"] = "SKIPPED"
        result["warnings"].append("Force data or force timestamps are missing or mismatched.")
        return result

    dt_force = (
        float(np.median(np.diff(t_force)))
        if len(t_force) >= 2
        else 1.0 / expected_force_hz
    )
    force_hz = 1.0 / dt_force if dt_force > 0 else expected_force_hz
    total_force_duration = (
        float(t_force[-1] - t_force[0])
        if len(t_force) >= 2
        else float(len(t_force) / expected_force_hz)
    )

    medium_threshold, high_threshold, hold_threshold = force_filter_bands

    bands = [
        (f"<{medium_threshold:g}N", force_mag < medium_threshold),
        (
            f"{medium_threshold:g}-{high_threshold:g}N",
            (force_mag >= medium_threshold) & (force_mag < high_threshold),
        ),
        (
            f"{high_threshold:g}-{hold_threshold:g}N",
            (force_mag >= high_threshold) & (force_mag < hold_threshold),
        ),
        (f">={hold_threshold:g}N", force_mag >= hold_threshold),
    ]

    for label, mask in bands:
        summary = segment_summary(t_force, force_mag, mask, force_hz)
        band_values = force_mag[mask]
        result["force_bands"][label] = {
            "duration_sec": summary["duration_sec"],
            "percentage": (
                100.0 * summary["duration_sec"] / total_force_duration
                if total_force_duration > 0
                else math.nan
            ),
            "longest_duration_sec": summary["longest_duration_sec"],
            "longest_start": summary["longest_start"],
            "longest_end": summary["longest_end"],
            "max_force": float(np.max(band_values)) if band_values.size else math.nan,
            "action_steps": {
                "samples": 0,
                "mean": math.nan,
                "p95": math.nan,
                "max": math.nan,
            },
        }

    action_data = action if action is not None else command
    result["action_source"] = (
        "action" if action is not None
        else "actions/joint_pos_command" if command is not None
        else None
    )

    action_step_t = np.asarray([], dtype=float)
    action_step_norm = np.asarray([], dtype=float)
    if action_data is None:
        result["warnings"].append(
            "No /action or actions/joint_pos_command dataset; action diagnostics skipped."
        )
    elif t_state is None:
        result["warnings"].append("State/action timestamps missing; action diagnostics skipped.")
    else:
        action_data = np.asarray(action_data, dtype=float)
        t_state_arr = np.asarray(t_state, dtype=float).reshape(-1)
        n = min(len(action_data), len(t_state_arr))
        if n < 2:
            result["warnings"].append("Too few aligned action samples for step diagnostics.")
        else:
            action_data = action_data[:n]
            t_state_arr = t_state_arr[:n]
            action_step_norm = np.linalg.norm(np.diff(action_data, axis=0), axis=1)
            action_step_t = t_state_arr[1:]
            force_on_action = nearest_values(action_step_t, t_force, force_mag)

            for label, _ in bands:
                if label == f"<{medium_threshold:g}N":
                    mask = force_on_action < medium_threshold
                elif label == f"{medium_threshold:g}-{high_threshold:g}N":
                    mask = (
                        (force_on_action >= medium_threshold)
                        & (force_on_action < high_threshold)
                    )
                elif label == f"{high_threshold:g}-{hold_threshold:g}N":
                    mask = (
                        (force_on_action >= high_threshold)
                        & (force_on_action < hold_threshold)
                    )
                else:
                    mask = force_on_action >= hold_threshold

                values = action_step_norm[mask]
                stats = finite_stats(values)
                result["force_bands"][label]["action_steps"] = {
                    "samples": int(values.size),
                    "mean": stats["mean"],
                    "p95": stats["p95"],
                    "max": stats["max"],
                }

            low_label = f"<{medium_threshold:g}N"
            medium_label = f"{medium_threshold:g}-{high_threshold:g}N"
            low_mean = result["force_bands"][low_label]["action_steps"]["mean"]
            medium_mean = result["force_bands"][medium_label]["action_steps"]["mean"]
            high_values = action_step_norm[force_on_action >= high_threshold]
            high_mean = finite_stats(high_values)["mean"]
            result["action_step_comparison"] = {
                "medium_smaller_than_low": (
                    bool(medium_mean < low_mean)
                    if np.isfinite(medium_mean) and np.isfinite(low_mean)
                    else None
                ),
                "high_smaller_than_medium": (
                    bool(high_mean < medium_mean)
                    if np.isfinite(high_mean) and np.isfinite(medium_mean)
                    else None
                ),
                "low_mean": low_mean,
                "medium_mean": medium_mean,
                "high_ge_threshold_mean": high_mean,
            }

    hold_event_rows = [
        {"name": name, "time": float(event_times[i])}
        for i, name in enumerate(events)
        if name in {"high_force_hold_start", "high_force_hold_end"}
        and i < len(event_times)
        and np.isfinite(event_times[i])
    ]
    starts = [row["time"] for row in hold_event_rows if row["name"] == "high_force_hold_start"]
    ends = [row["time"] for row in hold_event_rows if row["name"] == "high_force_hold_end"]

    hold_report: Dict[str, Any] = {
        "events_found": bool(hold_event_rows),
        "event_count": len(hold_event_rows),
        "events": hold_event_rows,
        "start_count": len(starts),
        "end_count": len(ends),
    }

    intervals = []
    end_idx = 0
    for start in starts:
        while end_idx < len(ends) and ends[end_idx] < start:
            end_idx += 1
        if end_idx < len(ends):
            intervals.append((start, ends[end_idx]))
            end_idx += 1

    if starts and ends:
        hold_report["intervals"] = [
            {"start": float(start), "end": float(end), "duration_sec": float(end - start)}
            for start, end in intervals
        ]
        hold_report["hold_count"] = len(intervals)
        hold_report["hold_total_duration_sec"] = float(
            sum(end - start for start, end in intervals)
        )

        force_hold_mask = np.zeros(len(t_force), dtype=bool)
        action_hold_mask = np.zeros(len(action_step_t), dtype=bool)
        for start, end in intervals:
            force_hold_mask |= (t_force >= start) & (t_force <= end)
            action_hold_mask |= (action_step_t >= start) & (action_step_t <= end)

        hold_report["force_magnitude"] = finite_stats(force_mag[force_hold_mask])
        hold_report["action_step_norm"] = finite_stats(action_step_norm[action_hold_mask])

    result["high_force_hold_events"] = hold_report
    return result


def fmt(x, nd=4) -> str:
    try:
        x = float(x)
        if not np.isfinite(x):
            return "nan"
        return f"{x:.{nd}f}"
    except Exception:
        return "nan"


def _decode_h5_scalar(value):
    if isinstance(value, bytes):
        return value.decode("utf-8", errors="ignore")
    if isinstance(value, np.generic):
        return value.item()
    return value


def read_hole_grid_record(episode_path: Path) -> Optional[Dict[str, Any]]:
    """Read the compact grid fields needed for dataset-level coverage."""
    try:
        with h5py.File(episode_path, "r") as h5:
            if "episode_metadata" not in h5:
                return None
            group = h5["episode_metadata"]
            mode = _decode_h5_scalar(group.attrs.get("hole_sampling_mode", ""))
            if mode != "grid":
                return None

            def attr(name, default=None):
                return _decode_h5_scalar(group.attrs.get(name, default))

            events = []
            if "events/names" in h5:
                events = decode_str_array(h5["events/names"][()])

            return {
                "episode_path": str(episode_path),
                "status": str(attr("status", h5.attrs.get("status", "unknown"))),
                "rows": int(attr("hole_grid_rows")),
                "cols": int(attr("hole_grid_cols")),
                "cycle": int(attr("hole_grid_cycle")),
                "index": int(attr("hole_grid_index")),
                "row": int(attr("hole_grid_row")),
                "col": int(attr("hole_grid_col")),
                "cell_label": str(attr("hole_grid_cell_label")),
                "sample_mode": str(attr("hole_grid_sample_mode", "unknown")),
                "traversal_order": str(
                    attr("hole_grid_traversal_order", "unknown")
                ),
                "seed": int(attr("hole_grid_seed", -1)),
                "task_success": "task_success_site_reached" in events,
                "joint_torque_alarm": "joint_torque_over_limit" in events,
                "ft_wrench_alarm": "ft_wrench_over_limit" in events,
            }
    except (OSError, KeyError, TypeError, ValueError) as exc:
        return {
            "episode_path": str(episode_path),
            "read_error": str(exc),
        }


def summarize_hole_grid_coverage(
    root: Path,
    target_per_cell: int = 10,
) -> Dict[str, Any]:
    """Summarize all grid-tagged episode files beneath a collection root."""
    root = root.expanduser().resolve()
    if root.is_file():
        episode_paths = [root]
    else:
        episode_paths = sorted(root.rglob("episode.hdf5"))

    records = []
    errors = []
    for path in episode_paths:
        record = read_hole_grid_record(path)
        if record is None:
            continue
        if "read_error" in record:
            errors.append(record)
        else:
            records.append(record)

    rows = max((record["rows"] for record in records), default=0)
    cols = max((record["cols"] for record in records), default=0)
    excluded_statuses = {
        "manual_discard",
        "teleop_start_failed",
        "controller_shutdown",
        "buffer_limit",
    }
    counted_records = [
        record for record in records if record["status"] not in excluded_statuses
    ]
    excluded_records = [
        record for record in records if record["status"] in excluded_statuses
    ]
    counts = np.zeros((rows, cols), dtype=int)
    successes = np.zeros((rows, cols), dtype=int)
    joint_alarms = np.zeros((rows, cols), dtype=int)
    ft_alarms = np.zeros((rows, cols), dtype=int)
    invalid_cells = []

    for record in counted_records:
        row = record["row"] - 1
        col = record["col"] - 1
        if not (0 <= row < rows and 0 <= col < cols):
            invalid_cells.append(record)
            continue
        counts[row, col] += 1
        successes[row, col] += int(record["task_success"])
        joint_alarms[row, col] += int(record["joint_torque_alarm"])
        ft_alarms[row, col] += int(record["ft_wrench_alarm"])

    target = max(0, int(target_per_cell))
    shortfall = np.maximum(target - counts, 0)

    def ratio_matrix(numerator: np.ndarray):
        return [
            [
                (float(numerator[row, col]) / int(counts[row, col]))
                if counts[row, col] > 0
                else None
                for col in range(cols)
            ]
            for row in range(rows)
        ]

    missing = [
        f"R{row + 1}C{col + 1}"
        for row in range(rows)
        for col in range(cols)
        if counts[row, col] == 0
    ]
    under_target = [
        f"R{row + 1}C{col + 1}"
        for row in range(rows)
        for col in range(cols)
        if counts[row, col] < target
    ]

    cycle_cells: Dict[int, set] = {}
    for record in counted_records:
        cycle_cells.setdefault(record["cycle"], set()).add(
            (record["row"], record["col"])
        )
    cycle_coverage = {
        str(cycle): {
            "unique_cells": len(cells),
            "complete": len(cells) == rows * cols,
            "missing_cells": [
                f"R{row}C{col}"
                for row in range(1, rows + 1)
                for col in range(1, cols + 1)
                if (row, col) not in cells
            ],
        }
        for cycle, cells in sorted(cycle_cells.items())
    }

    return {
        "root": str(root),
        "episode_files_scanned": len(episode_paths),
        "grid_episodes": len(records),
        "counted_grid_episodes": len(counted_records),
        "excluded_records": excluded_records,
        "rows": rows,
        "cols": cols,
        "target_per_cell": target,
        "counts": counts.tolist(),
        "task_success_counts": successes.tolist(),
        "task_success_rates": ratio_matrix(successes),
        "joint_torque_alarm_counts": joint_alarms.tolist(),
        "joint_torque_alarm_rates": ratio_matrix(joint_alarms),
        "ft_wrench_alarm_counts": ft_alarms.tolist(),
        "ft_wrench_alarm_rates": ratio_matrix(ft_alarms),
        "shortfall": shortfall.tolist(),
        "missing_cells": missing,
        "under_target_cells": under_target,
        "cycle_coverage": cycle_coverage,
        "complete_cycles": [
            int(cycle)
            for cycle, coverage in cycle_coverage.items()
            if coverage["complete"]
        ],
        "read_errors": errors,
        "invalid_cells": invalid_cells,
        "records": records,
    }


def format_hole_grid_coverage(report: Dict[str, Any]) -> str:
    lines = [
        "=" * 72,
        "MuJoCo Hole Grid Coverage Report",
        "=" * 72,
        f"Root: {report['root']}",
        (
            f"Grid episodes counted: {report['counted_grid_episodes']} "
            f"({report['grid_episodes']} tagged) / "
            f"{report['episode_files_scanned']} scanned"
        ),
        f"Grid: {report['rows']} x {report['cols']}",
        f"Target per cell: {report['target_per_cell']}",
        "",
        "Kept/present episode count by cell (R1 is high Z, C1 is low X):",
    ]

    cols = int(report["cols"])
    if cols:
        lines.append("        " + " ".join(f"C{col + 1:>3}" for col in range(cols)))
        for row, values in enumerate(report["counts"]):
            lines.append(
                f"R{row + 1:<3} " + " ".join(f"{int(value):>4}" for value in values)
            )
    else:
        lines.append("No grid-tagged episodes found.")

    def append_rate_matrix(title: str, key: str) -> None:
        if not cols:
            return
        lines.extend(["", title])
        lines.append("        " + " ".join(f"C{col + 1:>5}" for col in range(cols)))
        for row, values in enumerate(report[key]):
            rendered = ["    -" if value is None else f"{100.0 * value:5.1f}" for value in values]
            lines.append(f"R{row + 1:<3} " + " ".join(rendered))

    append_rate_matrix("Task-success event rate by cell (%):", "task_success_rates")
    append_rate_matrix(
        "Joint-torque alarm episode rate by cell (%):",
        "joint_torque_alarm_rates",
    )
    append_rate_matrix(
        "FT-wrench alarm episode rate by cell (%):",
        "ft_wrench_alarm_rates",
    )

    lines.extend(
        [
            "",
            "Missing cells: " + (", ".join(report["missing_cells"]) or "none"),
            "Under target: "
            + (", ".join(report["under_target_cells"]) or "none"),
            f"Read errors: {len(report['read_errors'])}",
            f"Invalid cell metadata: {len(report['invalid_cells'])}",
            f"Excluded failed/discarded episodes: {len(report['excluded_records'])}",
            "Complete cycles: "
            + (", ".join(map(str, report["complete_cycles"])) or "none"),
        ]
    )
    return "\n".join(lines)


# ---------------------------------------------------------------------
# Main analysis
# ---------------------------------------------------------------------

def analyze_episode(args) -> Dict[str, Any]:
    episode_path = Path(args.episode).expanduser().resolve()
    if episode_path.is_dir():
        episode_path = episode_path / "episode.hdf5"

    if not episode_path.exists():
        raise FileNotFoundError(f"Episode not found: {episode_path}")

    out_dir = Path(args.out).expanduser().resolve() if args.out else episode_path.parent / "quality_forcerange"
    out_dir.mkdir(parents=True, exist_ok=True)

    forcerange = parse_csv_floats(args.forcerange, expected=7)
    force_thresholds = parse_csv_floats(args.force_thresholds)
    force_filter_bands = parse_csv_floats(args.force_filter_bands, expected=3)
    if not np.all(np.diff(force_filter_bands) > 0):
        raise ValueError(
            "--force-filter-bands must contain three increasing thresholds"
        )

    issues: List[Dict[str, str]] = []

    report: Dict[str, Any] = {
        "episode_path": str(episode_path),
        "out_dir": str(out_dir),
        "forcerange": forcerange.tolist(),
        "issues": issues,
        "shapes": {},
        "timing": {},
        "metrics": {},
    }

    with h5py.File(episode_path, "r") as h5:
        # -------------------------------------------------------------
        # Dataset shape
        # -------------------------------------------------------------
        check_paths = [
            "observations/joint_pos",
            "observations/joint_vel",
            "observations/joint_torque",
            "observations/ee_pose",
            "observations/ft_wrench",
            "observations/ft_wrench_raw",
            "observations/ft_wrench_gravity",
            "action",
            "actions/joint_pos_command",
            "timestamps/state_episode",
            "timestamps/force_episode",
            "timestamps/image_episode",
            "observations/images",
            "events/names",
        ]

        for p in check_paths:
            report["shapes"][p] = get_shape_info(h5, p)

        required = [
            "observations/joint_pos",
            "observations/joint_vel",
            "observations/joint_torque",
            "observations/ft_wrench",
            "timestamps/state_episode",
            "timestamps/force_episode",
            "action",
        ]

        for p in required:
            if p not in h5:
                add_issue(issues, "FAIL", f"Missing required dataset: {p}")

        if any(x["level"] == "FAIL" for x in issues):
            report["verdict"] = "FAIL"
            return report

        # -------------------------------------------------------------
        # Load arrays
        # -------------------------------------------------------------
        qpos = get_array(h5, "observations/joint_pos")
        qvel = get_array(h5, "observations/joint_vel")
        tau = get_array(h5, "observations/joint_torque")
        ee_pose = get_array(h5, "observations/ee_pose")
        ft = get_array(h5, "observations/ft_wrench")
        action = get_array(h5, "action")
        command = get_array(h5, "actions/joint_pos_command")
        t_state = get_array(h5, "timestamps/state_episode")
        t_force = get_array(h5, "timestamps/force_episode")
        t_image = get_array(h5, "timestamps/image_episode")

        t_state = np.asarray(t_state, dtype=float).reshape(-1)
        t_force = np.asarray(t_force, dtype=float).reshape(-1)
        if t_image is not None:
            t_image = np.asarray(t_image, dtype=float).reshape(-1)

        # -------------------------------------------------------------
        # Root attrs
        # -------------------------------------------------------------
        root_attrs = {}
        for k, v in h5.attrs.items():
            if isinstance(v, bytes):
                root_attrs[k] = v.decode("utf-8", errors="ignore")
            elif isinstance(v, np.ndarray):
                root_attrs[k] = v.tolist()
            else:
                root_attrs[k] = str(v)
        report["root_attrs"] = root_attrs

        if "episode_metadata" in h5:
            metadata = h5["episode_metadata"]
            hole_grid = {}
            for key, value in metadata.attrs.items():
                if key.startswith(("hole_", "peg_")):
                    hole_grid[key] = _decode_h5_scalar(value)
            for key, value in metadata.items():
                if key.startswith(("hole_", "peg_")) and isinstance(
                    value,
                    h5py.Dataset,
                ):
                    hole_grid[key] = np.asarray(value[()]).tolist()
            report["hole_grid"] = hole_grid

        # -------------------------------------------------------------
        # Timing
        # -------------------------------------------------------------
        report["timing"]["state"] = timing_stats(t_state, args.expected_state_hz)
        report["timing"]["force"] = timing_stats(t_force, args.expected_force_hz)
        report["timing"]["image"] = timing_stats(t_image, args.expected_image_hz)

        for name, st in report["timing"].items():
            if st.get("status") == "WARN":
                add_issue(issues, "WARN", f"{name} timestamp has gaps/nonmonotonic/frequency issue")
            if st.get("status") == "MISSING":
                add_issue(issues, "WARN", f"{name} timestamp missing")

        # -------------------------------------------------------------
        # NaN / Inf
        # -------------------------------------------------------------
        nan_report = {}
        for name, arr in [
            ("qpos", qpos),
            ("qvel", qvel),
            ("joint_torque", tau),
            ("ee_pose", ee_pose),
            ("ft_wrench", ft),
            ("action", action),
            ("joint_pos_command", command),
        ]:
            arr = np.asarray(arr)
            bad = int(arr.size - np.sum(np.isfinite(arr)))
            nan_report[name] = {
                "shape": list(arr.shape),
                "bad": bad,
                "total": int(arr.size),
            }
            if bad > 0:
                add_issue(issues, "FAIL", f"{name} contains NaN/Inf: {bad}")

        report["metrics"]["nan_inf"] = nan_report

        # -------------------------------------------------------------
        # Events
        # -------------------------------------------------------------
        events = []
        event_times = []
        if "events/names" in h5:
            events = decode_str_array(h5["events/names"][()])
            if "events/t_episode" in h5:
                event_times = np.asarray(h5["events/t_episode"][()], dtype=float).tolist()
            elif "events/t_sim" in h5:
                event_times = np.asarray(h5["events/t_sim"][()], dtype=float).tolist()

        report["metrics"]["events"] = {
            "names": events,
            "times": event_times,
            "has_task_success_site_reached": "task_success_site_reached" in events,
            "has_terminal_hold_start": "terminal_hold_start" in events,
            "has_terminal_hold_complete": "terminal_hold_complete" in events,
            "has_auto_stop_task_success": "auto_stop_task_success" in events,
            "has_scripted_replay_success": "scripted_replay_success" in events,
            "has_joint_torque_over_limit": "joint_torque_over_limit" in events,
        }

        success_time = None
        if "task_success_site_reached" in events and event_times:
            idx = events.index("task_success_site_reached")
            success_time = float(event_times[idx])

        terminal_start_time = None
        if "terminal_hold_start" in events and event_times:
            idx = events.index("terminal_hold_start")
            terminal_start_time = float(event_times[idx])

        terminal_complete_time = None
        if "terminal_hold_complete" in events and event_times:
            idx = events.index("terminal_hold_complete")
            terminal_complete_time = float(event_times[idx])

        if args.expect_auto_success:
            for ev in [
                "task_success_site_reached",
                "terminal_hold_start",
                "terminal_hold_complete",
            ]:
                if ev not in events:
                    add_issue(issues, "WARN", f"Expected event missing: {ev}")
            lifecycle_success_events = {
                "auto_stop_task_success",
                "scripted_success",
                "scripted_replay_success",
            }
            if not lifecycle_success_events.intersection(events):
                add_issue(
                    issues,
                    "WARN",
                    "Expected successful recording-stop event missing",
                )

        # -------------------------------------------------------------
        # Action / command consistency
        # -------------------------------------------------------------
        if command is not None:
            if action.shape == command.shape:
                err = np.abs(action - command)
                report["metrics"]["action_command_max_abs_error"] = float(np.max(err))
                if np.max(err) > 1e-9:
                    add_issue(issues, "WARN", "action and joint_pos_command are not identical")
            else:
                add_issue(issues, "FAIL", f"action/command shape mismatch: {action.shape} vs {command.shape}")

        # -------------------------------------------------------------
        # Action-qpos tracking
        # -------------------------------------------------------------
        tracking = np.abs(action[:, :7] - qpos[:, :7])
        tracking_norm = np.linalg.norm(action[:, :7] - qpos[:, :7], axis=1)

        report["metrics"]["tracking_abs_all"] = finite_stats(tracking.reshape(-1))
        report["metrics"]["tracking_norm"] = finite_stats(tracking_norm)
        report["metrics"]["tracking_per_joint_mean"] = np.mean(tracking, axis=0).tolist()
        report["metrics"]["tracking_per_joint_p95"] = np.percentile(tracking, 95, axis=0).tolist()
        report["metrics"]["tracking_per_joint_max"] = np.max(tracking, axis=0).tolist()

        if np.percentile(tracking_norm, 95) > args.tracking_norm_p95_warn:
            add_issue(
                issues,
                "WARN",
                f"Tracking norm p95 too high: {np.percentile(tracking_norm, 95):.4f} > {args.tracking_norm_p95_warn:.4f}"
            )

        if np.max(tracking_norm) > args.tracking_norm_max_warn:
            add_issue(
                issues,
                "WARN",
                f"Tracking norm max too high: {np.max(tracking_norm):.4f} > {args.tracking_norm_max_warn:.4f}"
            )

        # -------------------------------------------------------------
        # Joint torque + actuator saturation
        # -------------------------------------------------------------
        abs_tau = np.abs(tau[:, :7])
        dt_state = float(np.median(np.diff(t_state))) if len(t_state) >= 2 else 1.0 / args.expected_state_hz

        joint_report = []
        for j in range(7):
            x = abs_tau[:, j]
            limit = forcerange[j]

            over_80 = x > 0.80 * limit
            over_95 = x > 0.95 * limit
            over_99 = x > 0.99 * limit

            st = finite_stats(x)
            st.update({
                "forcerange": float(limit),
                "over_80_count": int(np.sum(over_80)),
                "over_80_sec": float(np.sum(over_80) * dt_state),
                "over_95_count": int(np.sum(over_95)),
                "over_95_sec": float(np.sum(over_95) * dt_state),
                "over_99_count": int(np.sum(over_99)),
                "over_99_sec": float(np.sum(over_99) * dt_state),
            })
            joint_report.append(st)

            if st["over_95_sec"] > args.saturation_95_sec_warn:
                add_issue(
                    issues,
                    "WARN",
                    f"joint_{j+1} near forcerange saturation: >95% for {st['over_95_sec']:.3f}s"
                )

            if st["over_99_sec"] > args.saturation_99_sec_warn:
                add_issue(
                    issues,
                    "WARN",
                    f"joint_{j+1} almost saturated: >99% for {st['over_99_sec']:.3f}s"
                )

        report["metrics"]["joint_torque_abs"] = joint_report

        # -------------------------------------------------------------
        # Force magnitude and duration
        # -------------------------------------------------------------
        force_mag = np.linalg.norm(ft[:, :3], axis=1)
        ft_torque_mag = np.linalg.norm(ft[:, 3:6], axis=1)
        dt_force = float(np.median(np.diff(t_force))) if len(t_force) >= 2 else 1.0 / args.expected_force_hz
        force_hz_est = 1.0 / dt_force if dt_force > 0 else args.expected_force_hz

        report["metrics"]["force_magnitude"] = finite_stats(force_mag)
        report["metrics"]["ft_torque_magnitude"] = finite_stats(ft_torque_mag)

        force_duration_report = {}
        for th in force_thresholds:
            mask = force_mag > th
            force_duration_report[f"over_{th:g}N"] = segment_summary(
                t_force,
                force_mag,
                mask,
                force_hz_est,
            )

        report["metrics"]["force_duration"] = force_duration_report
        report["metrics"]["force_aware_filter_diagnostics"] = (
            force_aware_filter_diagnostics(
                force_mag=force_mag,
                t_force=t_force,
                action=action,
                command=command,
                t_state=t_state,
                events=events,
                event_times=event_times,
                expected_force_hz=args.expected_force_hz,
                force_filter_bands=force_filter_bands,
            )
        )

        if np.max(force_mag) > args.force_max_warn:
            add_issue(
                issues,
                "WARN",
                f"Force max too high: {np.max(force_mag):.3f} N > {args.force_max_warn:.3f} N"
            )

        if np.percentile(force_mag, 95) > args.force_p95_warn:
            add_issue(
                issues,
                "WARN",
                f"Force p95 too high: {np.percentile(force_mag, 95):.3f} N > {args.force_p95_warn:.3f} N"
            )

        over_40 = force_duration_report.get("over_40N")
        if over_40 and over_40["longest_duration_sec"] > args.force_over40_longest_warn:
            add_issue(
                issues,
                "WARN",
                f"Force >40N longest segment too long: {over_40['longest_duration_sec']:.3f}s"
            )

        over_50 = force_duration_report.get("over_50N")
        if over_50 and over_50["duration_sec"] > args.force_over50_total_warn:
            add_issue(
                issues,
                "WARN",
                f"Force >50N total duration too long: {over_50['duration_sec']:.3f}s"
            )

        # -------------------------------------------------------------
        # Insertion progress during high force
        # -------------------------------------------------------------
        # Default: y axis with negative sign because your insertion direction is roughly -Y.
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_id = axis_map[args.insert_axis]
        insert_sign = float(args.insert_sign)

        insert_pos = insert_sign * ee_pose[:, axis_id]
        force_on_state = np.interp(t_state, t_force, force_mag)

        progress_report = {}
        for th in force_thresholds:
            mask = force_on_state > th
            if np.sum(mask) >= 2:
                progress = float(insert_pos[mask][-1] - insert_pos[mask][0])
                duration = float(t_state[mask][-1] - t_state[mask][0])
                progress_report[f"force_over_{th:g}N"] = {
                    "state_samples": int(np.sum(mask)),
                    "duration_sec": duration,
                    "insert_progress_m": progress,
                    "force_median": float(np.median(force_on_state[mask])),
                    "force_max": float(np.max(force_on_state[mask])),
                }

                if th >= 50 and duration > args.jam_duration_warn and progress < args.jam_progress_min:
                    add_issue(
                        issues,
                        "WARN",
                        f"Possible jam: force>{th:g}N for {duration:.3f}s but insertion progress only {progress:.5f}m"
                    )
            else:
                progress_report[f"force_over_{th:g}N"] = {
                    "state_samples": int(np.sum(mask)),
                    "duration_sec": 0.0,
                    "insert_progress_m": 0.0,
                }

        report["metrics"]["insertion_progress_under_force"] = progress_report

        # -------------------------------------------------------------
        # Force around success / terminal hold
        # -------------------------------------------------------------
        success_force_report = {}
        if success_time is not None:
            for window_name, lo, hi in [
                ("before_success_0p5s", success_time - 0.5, success_time),
                ("around_success_0p2s", success_time - 0.1, success_time + 0.1),
                ("after_success_0p5s", success_time, success_time + 0.5),
            ]:
                mask = (t_force >= lo) & (t_force <= hi)
                if np.sum(mask) > 0:
                    success_force_report[window_name] = finite_stats(force_mag[mask])

        report["metrics"]["success_force_windows"] = success_force_report

        # -------------------------------------------------------------
        # Terminal hold
        # -------------------------------------------------------------
        terminal = {}
        if terminal_start_time is not None:
            terminal_end = (
                terminal_complete_time
                if terminal_complete_time is not None
                else float(t_state[-1])
            )
            state_tail = (t_state >= terminal_start_time) & (
                t_state <= terminal_end
            )
            force_tail = (t_force >= terminal_start_time) & (
                t_force <= terminal_end
            )
            terminal["window_source"] = "terminal_hold_events"
            terminal["window_start_s"] = terminal_start_time
            terminal["window_end_s"] = terminal_end
        else:
            t_end = float(t_state[-1])
            state_tail = t_state >= t_end - args.terminal_sec
            force_tail = t_force >= float(t_force[-1]) - args.terminal_sec
            terminal["window_source"] = "episode_tail"

        if np.sum(state_tail) >= 3:
            qvel_tail = qvel[state_tail]
            qpos_tail = qpos[state_tail]
            action_tail = action[state_tail]

            qvel_norm = np.linalg.norm(qvel_tail, axis=1)
            qpos_step = np.linalg.norm(np.diff(qpos_tail, axis=0), axis=1)
            action_step = np.linalg.norm(np.diff(action_tail, axis=0), axis=1)

            terminal["state_samples"] = int(np.sum(state_tail))
            terminal["qvel_norm"] = finite_stats(qvel_norm)
            terminal["qpos_step_norm"] = finite_stats(qpos_step)
            terminal["action_step_norm"] = finite_stats(action_step)

            if np.max(qvel_norm) > args.terminal_qvel_max_warn:
                add_issue(
                    issues,
                    "WARN",
                    f"Terminal qvel max too high: {np.max(qvel_norm):.4f}"
                )

        if np.sum(force_tail) >= 3:
            terminal_force = force_mag[force_tail]
            terminal["force_magnitude"] = finite_stats(terminal_force)

            if np.max(terminal_force) > args.terminal_force_max_warn:
                add_issue(
                    issues,
                    "WARN",
                    f"Terminal force max too high: {np.max(terminal_force):.3f} N"
                )

        report["metrics"]["terminal_hold"] = terminal

        # -------------------------------------------------------------
        # Reset contamination / qpos discontinuity
        # -------------------------------------------------------------
        qpos_step_all = np.linalg.norm(np.diff(qpos, axis=0), axis=1)
        report["metrics"]["qpos_step_norm"] = finite_stats(qpos_step_all)
        report["metrics"]["qpos_step_norm"]["max_time"] = float(t_state[int(np.argmax(qpos_step_all)) + 1])

        if np.max(qpos_step_all) > args.qpos_step_max_warn:
            add_issue(
                issues,
                "WARN",
                f"Large qpos jump detected: {np.max(qpos_step_all):.4f} rad"
            )

        # -------------------------------------------------------------
        # Image check
        # -------------------------------------------------------------
        images = {}
        if "observations/images" in h5:
            img_group = h5["observations/images"]
            for cam_name in img_group.keys():
                if cam_name == "camera_names":
                    continue

                obj = img_group[cam_name]
                if not isinstance(obj, h5py.Dataset):
                    images[cam_name] = {"type": "not_dataset"}
                    continue

                cam_report = {
                    "shape": list(obj.shape),
                    "dtype": str(obj.dtype),
                    "samples": int(obj.shape[0]) if len(obj.shape) > 0 else 0,
                }

                if len(obj.shape) == 4 and obj.shape[0] > 0:
                    idxs = sorted(set([0, obj.shape[0] // 2, obj.shape[0] - 1]))
                    means = []
                    stds = []
                    for idx in idxs:
                        frame = obj[idx]
                        means.append(float(np.mean(frame)))
                        stds.append(float(np.std(frame)))
                    cam_report["sample_pixel_mean"] = means
                    cam_report["sample_pixel_std"] = stds

                    if min(stds) < args.image_std_min_warn:
                        add_issue(
                            issues,
                            "WARN",
                            f"Camera {cam_name} image std too low, possible blank image"
                        )

                if t_image is not None and len(t_image) > 0 and abs(cam_report["samples"] - len(t_image)) > 2:
                    add_issue(
                        issues,
                        "WARN",
                        f"Camera {cam_name} samples mismatch image timestamp: {cam_report['samples']} vs {len(t_image)}"
                    )

                images[cam_name] = cam_report
        else:
            add_issue(issues, "WARN", "observations/images missing")

        report["metrics"]["images"] = images

    report["verdict"] = verdict_from_issues(issues)

    # -------------------------------------------------------------
    # Save reports
    # -------------------------------------------------------------
    json_path = out_dir / "quality_forcerange_summary.json"
    txt_path = out_dir / "quality_forcerange_report.txt"

    json_path.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    txt_path.write_text(format_report(report), encoding="utf-8")

    if HAS_MPL and not args.no_plots:
        save_plots(report, out_dir, episode_path)

    return report


# ---------------------------------------------------------------------
# Report formatting
# ---------------------------------------------------------------------

def format_report(report: Dict[str, Any]) -> str:
    lines = []
    lines.append("=" * 90)
    lines.append("MuJoCo Peg-in-Hole Quality Report with Forcerange Saturation Check")
    lines.append("=" * 90)
    lines.append(f"Episode : {report['episode_path']}")
    lines.append(f"Output  : {report['out_dir']}")
    lines.append(f"Verdict : {report['verdict']}")
    lines.append(f"Forcerange: {report['forcerange']}")
    lines.append("")

    if report.get("hole_grid"):
        grid = report["hole_grid"]
        lines.append("=== Task geometry and hole grid assignment ===")
        lines.append(
            f"cell={grid.get('hole_grid_cell_label')}, "
            f"cycle={grid.get('hole_grid_cycle')}, "
            f"index={grid.get('hole_grid_index')}, "
            f"offset={grid.get('hole_actual_offset_xyz')}"
        )
        lines.append(
            f"peg: radius={grid.get('peg_radius_m')}m, "
            f"length={grid.get('peg_length_m')}m, "
            f"initial_tip={grid.get('peg_tip_site_initial_pos_world')}"
        )
        lines.append(
            f"hole: radius={grid.get('hole_radius_m')}m, "
            f"depth={grid.get('hole_depth_m')}m, "
            f"goal={grid.get('hole_goal_site_initial_pos_world')}"
        )
        lines.append("")

    lines.append("=== Issues ===")
    if not report["issues"]:
        lines.append("PASS: no issue detected.")
    else:
        for item in report["issues"]:
            lines.append(f"[{item['level']}] {item['message']}")
    lines.append("")

    lines.append("=== Timing ===")
    for name, st in report["timing"].items():
        lines.append(
            f"{name}: samples={st.get('samples')} "
            f"duration={fmt(st.get('duration'), 3)}s "
            f"median_hz={fmt(st.get('median_hz'), 2)} "
            f"max_dt={fmt(st.get('max_dt'), 4)} "
            f"gaps={st.get('gap_count')} "
            f"status={st.get('status')}"
        )
    lines.append("")

    m = report["metrics"]

    lines.append("=== Events ===")
    ev = m.get("events", {})
    names = ev.get("names", [])
    times = ev.get("times", [])
    if times:
        for i, name in enumerate(names):
            t = times[i] if i < len(times) else None
            lines.append(f"{i}: {name}, t={fmt(t, 3)}")
    else:
        lines.append(str(names))
    lines.append("")

    lines.append("=== Action / qpos tracking ===")
    tr = m.get("tracking_norm", {})
    lines.append(
        f"tracking norm: median={fmt(tr.get('median'))}, "
        f"p95={fmt(tr.get('p95'))}, "
        f"p99={fmt(tr.get('p99'))}, "
        f"max={fmt(tr.get('max'))}"
    )
    lines.append(f"per-joint p95: {np.round(m.get('tracking_per_joint_p95', []), 5).tolist()}")
    lines.append(f"per-joint max : {np.round(m.get('tracking_per_joint_max', []), 5).tolist()}")
    lines.append("")

    lines.append("=== Joint torque / forcerange saturation ===")
    for i, st in enumerate(m.get("joint_torque_abs", [])):
        lines.append(
            f"joint_{i+1}: "
            f"rms={fmt(st.get('rms'))}, "
            f"p95={fmt(st.get('p95'))}, "
            f"p99={fmt(st.get('p99'))}, "
            f"max={fmt(st.get('max'))}, "
            f"limit={fmt(st.get('forcerange'))}, "
            f">80%={fmt(st.get('over_80_sec'), 3)}s, "
            f">95%={fmt(st.get('over_95_sec'), 3)}s, "
            f">99%={fmt(st.get('over_99_sec'), 3)}s"
        )
    lines.append("")

    lines.append("=== Force magnitude ===")
    fm = m.get("force_magnitude", {})
    tm = m.get("ft_torque_magnitude", {})
    lines.append(
        f"force [N]: median={fmt(fm.get('median'))}, "
        f"p95={fmt(fm.get('p95'))}, "
        f"p99={fmt(fm.get('p99'))}, "
        f"max={fmt(fm.get('max'))}"
    )
    lines.append(
        f"FT torque [Nm]: median={fmt(tm.get('median'))}, "
        f"p95={fmt(tm.get('p95'))}, "
        f"p99={fmt(tm.get('p99'))}, "
        f"max={fmt(tm.get('max'))}"
    )

    for key, st in m.get("force_duration", {}).items():
        lines.append(
            f"{key}: total={fmt(st.get('duration_sec'), 3)}s, "
            f"longest={fmt(st.get('longest_duration_sec'), 3)}s, "
            f"range={fmt(st.get('longest_start'), 3)}->{fmt(st.get('longest_end'), 3)}, "
            f"longest_max={fmt(st.get('longest_max'))}"
        )
    lines.append("")

    lines.append("=== Force-aware filter diagnostics ===")
    fad = m.get("force_aware_filter_diagnostics", {})
    if fad.get("status") == "SKIPPED":
        for warning in fad.get("warnings", []):
            lines.append(f"Warning: {warning}")
    else:
        for label, st in fad.get("force_bands", {}).items():
            action_st = st.get("action_steps", {})
            lines.append(
                f"{label}: total={fmt(st.get('duration_sec'), 3)}s "
                f"({fmt(st.get('percentage'), 2)}%), "
                f"longest={fmt(st.get('longest_duration_sec'), 3)}s, "
                f"range={fmt(st.get('longest_start'), 3)}->{fmt(st.get('longest_end'), 3)}, "
                f"max_force={fmt(st.get('max_force'), 3)}N"
            )
            lines.append(
                f"  action steps: samples={action_st.get('samples', 0)}, "
                f"mean={fmt(action_st.get('mean'), 6)}, "
                f"p95={fmt(action_st.get('p95'), 6)}, "
                f"max={fmt(action_st.get('max'), 6)}"
            )

        lines.append(f"action source: {fad.get('action_source')}")
        comparison = fad.get("action_step_comparison", {})
        medium_cmp = comparison.get("medium_smaller_than_low")
        high_cmp = comparison.get("high_smaller_than_medium")
        thresholds = fad.get("thresholds", [40.0, 80.0, 100.0])
        medium_threshold, high_threshold, _ = thresholds
        lines.append(
            f"{medium_threshold:g}-{high_threshold:g}N action steps smaller "
            f"than <{medium_threshold:g}N: "
            + ("unavailable" if medium_cmp is None else str(medium_cmp))
        )
        lines.append(
            f">={high_threshold:g}N action steps smaller than "
            f"{medium_threshold:g}-{high_threshold:g}N: "
            + ("unavailable" if high_cmp is None else str(high_cmp))
        )

        for warning in fad.get("warnings", []):
            lines.append(f"Warning: {warning}")

        hold = fad.get("high_force_hold_events", {})
        if not hold.get("events_found", False):
            lines.append(
                "No high_force_hold_start/end events found. "
                "Hold diagnostics unavailable from events."
            )
        else:
            lines.append(
                f"hold events: count={hold.get('event_count', 0)}, "
                f"starts={hold.get('start_count', 0)}, "
                f"ends={hold.get('end_count', 0)}"
            )
            for row in hold.get("events", []):
                lines.append(
                    f"  {row.get('name')}: t={fmt(row.get('time'), 3)}s"
                )

            if "hold_count" in hold:
                lines.append(
                    f"estimated holds: count={hold.get('hold_count', 0)}, "
                    f"total={fmt(hold.get('hold_total_duration_sec'), 3)}s"
                )
                hold_action = hold.get("action_step_norm", {})
                hold_force = hold.get("force_magnitude", {})
                lines.append(
                    "  action_step_norm during hold: "
                    f"mean={fmt(hold_action.get('mean'), 6)}, "
                    f"p95={fmt(hold_action.get('p95'), 6)}, "
                    f"max={fmt(hold_action.get('max'), 6)}"
                )
                lines.append(
                    "  force magnitude during hold: "
                    f"mean={fmt(hold_force.get('mean'), 3)}, "
                    f"p95={fmt(hold_force.get('p95'), 3)}, "
                    f"max={fmt(hold_force.get('max'), 3)}"
                )
    lines.append("")

    lines.append("=== Insertion progress under force ===")
    for key, st in m.get("insertion_progress_under_force", {}).items():
        lines.append(
            f"{key}: samples={st.get('state_samples')}, "
            f"duration={fmt(st.get('duration_sec'), 3)}s, "
            f"insert_progress={fmt(st.get('insert_progress_m'), 5)}m, "
            f"force_median={fmt(st.get('force_median'))}, "
            f"force_max={fmt(st.get('force_max'))}"
        )
    lines.append("")

    lines.append("=== Force around success ===")
    for key, st in m.get("success_force_windows", {}).items():
        lines.append(
            f"{key}: median={fmt(st.get('median'))}, "
            f"p95={fmt(st.get('p95'))}, "
            f"max={fmt(st.get('max'))}"
        )
    lines.append("")

    lines.append("=== Terminal hold ===")
    th = m.get("terminal_hold", {})
    for key in ["qvel_norm", "qpos_step_norm", "action_step_norm", "force_magnitude"]:
        st = th.get(key)
        if st:
            lines.append(
                f"{key}: median={fmt(st.get('median'))}, "
                f"p95={fmt(st.get('p95'))}, "
                f"max={fmt(st.get('max'))}"
            )
    lines.append("")

    lines.append("=== Qpos discontinuity / reset contamination ===")
    qstep = m.get("qpos_step_norm", {})
    lines.append(
        f"qpos step norm: median={fmt(qstep.get('median'))}, "
        f"p95={fmt(qstep.get('p95'))}, "
        f"max={fmt(qstep.get('max'))}, "
        f"max_time={fmt(qstep.get('max_time'), 3)}s"
    )
    lines.append("")

    lines.append("=== Images ===")
    for cam, st in m.get("images", {}).items():
        lines.append(f"{cam}: {st}")
    lines.append("")

    lines.append("=" * 90)
    lines.append("Interpretation")
    lines.append("=" * 90)
    if report["verdict"] == "PASS":
        lines.append("PASS: This episode is a clean candidate under current thresholds.")
    elif report["verdict"] == "WARN":
        lines.append("WARN: Structurally usable, but requires manual review. Check force duration, saturation, and video.")
    else:
        lines.append("FAIL: Critical data problem. Usually discard.")
    lines.append("")

    return "\n".join(lines)


# ---------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------

def save_plots(report: Dict[str, Any], out_dir: Path, episode_path: Path):
    if not HAS_MPL:
        return

    with h5py.File(episode_path, "r") as h5:
        t_state = h5["timestamps/state_episode"][()]
        t_force = h5["timestamps/force_episode"][()]
        qpos = h5["observations/joint_pos"][()]
        qvel = h5["observations/joint_vel"][()]
        tau = h5["observations/joint_torque"][()]
        action = h5["action"][()]
        ft = h5["observations/ft_wrench"][()]

    force_mag = np.linalg.norm(ft[:, :3], axis=1)
    ft_torque_mag = np.linalg.norm(ft[:, 3:6], axis=1)
    tracking_norm = np.linalg.norm(action[:, :7] - qpos[:, :7], axis=1)
    qvel_norm = np.linalg.norm(qvel[:, :7], axis=1)

    # Joint torque
    plt.figure(figsize=(12, 5))
    for j in range(7):
        plt.plot(t_state, tau[:, j], label=f"j{j+1}", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("joint torque [Nm]")
    plt.title("Joint Torque")
    plt.legend(ncol=4)
    plt.tight_layout()
    plt.savefig(out_dir / "joint_torque.png", dpi=160)
    plt.close()

    # Force magnitude
    plt.figure(figsize=(12, 5))
    plt.plot(t_force, force_mag, label="||F|| [N]", linewidth=1.0)
    plt.plot(t_force, ft_torque_mag, label="||T|| [Nm]", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("magnitude")
    plt.title("Wrist FT Magnitude")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "ft_magnitude.png", dpi=160)
    plt.close()

    # Tracking
    plt.figure(figsize=(12, 5))
    plt.plot(t_state, tracking_norm, label="||action - qpos||", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("rad")
    plt.title("Action-Qpos Tracking Error")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "tracking_error.png", dpi=160)
    plt.close()

    # Terminal motion
    plt.figure(figsize=(12, 5))
    plt.plot(t_state, qvel_norm, label="||qvel||", linewidth=1.0)
    plt.xlabel("time [s]")
    plt.ylabel("norm")
    plt.title("Joint Velocity Norm")
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_dir / "qvel_norm.png", dpi=160)
    plt.close()


# ---------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------

def build_parser():
    p = argparse.ArgumentParser()

    p.add_argument(
        "episode",
        type=str,
        nargs="?",
        help="Path to episode.hdf5 or episode directory.",
    )
    p.add_argument("--out", type=str, default=None)
    p.add_argument(
        "--coverage-root",
        type=str,
        default=None,
        help="Scan a dataset directory and report 5x5 hole-grid coverage.",
    )
    p.add_argument("--grid-target-per-cell", type=int, default=10)

    p.add_argument(
        "--forcerange",
        type=str,
        default="20,20,20,20,10,10,10",
        help="7 actuator forcerange absolute limits in Nm.",
    )

    p.add_argument("--expected-state-hz", type=float, default=30.0)
    p.add_argument("--expected-force-hz", type=float, default=500.0)
    p.add_argument("--expected-image-hz", type=float, default=30.0)

    p.add_argument(
        "--force-thresholds",
        type=str,
        default="30,40,50",
        help="Force thresholds for duration statistics.",
    )
    p.add_argument(
        "--force-filter-bands",
        type=str,
        default="40,80,100",
        help="Increasing medium, high, and hold thresholds for force-aware diagnostics.",
    )

    # Force quality thresholds.
    p.add_argument("--force-p95-warn", type=float, default=40.0)
    p.add_argument("--force-max-warn", type=float, default=60.0)
    p.add_argument("--force-over40-longest-warn", type=float, default=0.25)
    p.add_argument("--force-over50-total-warn", type=float, default=0.15)

    # Saturation quality thresholds.
    p.add_argument("--saturation-95-sec-warn", type=float, default=0.20)
    p.add_argument("--saturation-99-sec-warn", type=float, default=0.05)

    # Tracking thresholds.
    p.add_argument("--tracking-norm-p95-warn", type=float, default=0.03)
    p.add_argument("--tracking-norm-max-warn", type=float, default=0.06)

    # Insertion progress estimation.
    p.add_argument("--insert-axis", type=str, default="y", choices=["x", "y", "z"])
    p.add_argument(
        "--insert-sign",
        type=float,
        default=-1.0,
        help="Use -1 if insertion direction is negative axis, +1 if positive.",
    )
    p.add_argument("--jam-duration-warn", type=float, default=0.50)
    p.add_argument("--jam-progress-min", type=float, default=0.0015)

    # Terminal hold.
    p.add_argument("--terminal-sec", type=float, default=0.8)
    p.add_argument("--terminal-qvel-max-warn", type=float, default=0.05)
    p.add_argument("--terminal-force-max-warn", type=float, default=15.0)

    # Reset contamination.
    p.add_argument("--qpos-step-max-warn", type=float, default=0.05)

    # Image.
    p.add_argument("--image-std-min-warn", type=float, default=3.0)

    p.add_argument("--expect-auto-success", action="store_true")
    p.add_argument("--no-plots", action="store_true")

    return p


def main():
    args = build_parser().parse_args()
    if args.coverage_root:
        root = Path(args.coverage_root)
        report = summarize_hole_grid_coverage(
            root,
            target_per_cell=args.grid_target_per_cell,
        )
        resolved_root = root.expanduser().resolve()
        default_out = (
            resolved_root.parent / "grid_coverage"
            if resolved_root.is_file()
            else resolved_root / "grid_coverage"
        )
        out_dir = (
            Path(args.out).expanduser().resolve()
            if args.out
            else default_out
        )
        out_dir.mkdir(parents=True, exist_ok=True)
        json_path = out_dir / "hole_grid_coverage.json"
        txt_path = out_dir / "hole_grid_coverage.txt"
        json_path.write_text(
            json.dumps(report, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        txt = format_hole_grid_coverage(report)
        txt_path.write_text(txt, encoding="utf-8")
        print(txt)
        print("Saved:")
        print(" ", txt_path)
        print(" ", json_path)
        return

    if not args.episode:
        raise SystemExit("episode is required unless --coverage-root is provided")

    report = analyze_episode(args)

    txt_path = Path(report["out_dir"]) / "quality_forcerange_report.txt"
    json_path = Path(report["out_dir"]) / "quality_forcerange_summary.json"

    print(txt_path.read_text(encoding="utf-8"))
    print("Saved:")
    print(" ", txt_path)
    print(" ", json_path)
    if HAS_MPL and not args.no_plots:
        print(" plots in:", report["out_dir"])


if __name__ == "__main__":
    main()
