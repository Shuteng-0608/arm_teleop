#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path

import h5py
import numpy as np
import matplotlib.pyplot as plt


def resolve_episode_path(path_str: str) -> Path:
    """
    Accept either:
      1. direct /path/to/episode.hdf5
      2. an episode folder containing episode.hdf5
      3. a root folder containing many episode.hdf5 files

    If a folder contains many episode.hdf5 files, use the newest one.
    """
    path = Path(path_str).expanduser().resolve()

    if path.is_file():
        return path

    if not path.exists():
        raise FileNotFoundError(f"Path does not exist: {path}")

    direct = path / "episode.hdf5"
    if direct.exists():
        return direct

    candidates = list(path.rglob("episode.hdf5"))
    if not candidates:
        raise FileNotFoundError(f"No episode.hdf5 found under: {path}")

    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def read_string_list(h5: h5py.File, key: str):
    if key not in h5:
        return None

    arr = h5[key][()]
    if np.isscalar(arr):
        arr = [arr]

    names = []
    for x in arr:
        if isinstance(x, bytes):
            names.append(x.decode("utf-8"))
        else:
            names.append(str(x))
    return names


def parse_limits(limit_str, n_joints: int):
    if limit_str is None or limit_str.strip() == "":
        return None

    values = [float(x.strip()) for x in limit_str.split(",") if x.strip()]

    if len(values) == 1:
        return np.full(n_joints, values[0], dtype=float)

    if len(values) != n_joints:
        raise ValueError(
            f"--limits must be either one scalar or {n_joints} comma-separated values. "
            f"Got {len(values)} values."
        )

    return np.asarray(values, dtype=float)


def get_time_vector(h5: h5py.File, n: int):
    if "timestamps/state_episode" in h5:
        t = h5["timestamps/state_episode"][:]
    elif "timestamps/state" in h5:
        t = h5["timestamps/state"][:]
        t = t - t[0]
    else:
        t = np.arange(n, dtype=float) / 30.0

    if len(t) != n:
        raise ValueError(
            f"Time length mismatch: len(t)={len(t)}, torque length={n}"
        )

    return t


def get_joint_names(h5: h5py.File, n_joints: int):
    candidates = [
        "episode_metadata/joint_names",
        "observations/joint_names",
    ]

    for key in candidates:
        names = read_string_list(h5, key)
        if names is not None and len(names) == n_joints:
            return names

    return [f"joint_{i + 1}" for i in range(n_joints)]


def read_events(h5: h5py.File):
    if "events/names" not in h5:
        return []

    raw = h5["events/names"][()]
    events = []
    for x in raw:
        if isinstance(x, bytes):
            events.append(x.decode("utf-8"))
        else:
            events.append(str(x))
    return events


def summarize_joint_torque(t, torque, joint_names, limits=None):
    """
    torque shape: [N, J]
    """
    n, j = torque.shape

    dt = np.diff(t)
    if len(dt) > 0:
        median_dt = float(np.median(dt))
    else:
        median_dt = 1.0 / 30.0

    rows = []

    for k in range(j):
        tau = torque[:, k]
        abs_tau = np.abs(tau)

        peak_idx = int(np.nanargmax(abs_tau))
        peak_time = float(t[peak_idx])
        peak_value = float(tau[peak_idx])
        max_abs = float(abs_tau[peak_idx])

        row = {
            "joint": joint_names[k],
            "mean_Nm": float(np.nanmean(tau)),
            "std_Nm": float(np.nanstd(tau)),
            "rms_Nm": float(np.sqrt(np.nanmean(tau ** 2))),
            "p95_abs_Nm": float(np.nanpercentile(abs_tau, 95)),
            "p99_abs_Nm": float(np.nanpercentile(abs_tau, 99)),
            "max_abs_Nm": max_abs,
            "max_signed_Nm": peak_value,
            "max_time_s": peak_time,
        }

        if limits is not None:
            limit = float(limits[k])
            over = abs_tau > limit

            over_count = int(np.sum(over))
            over_duration = float(over_count * median_dt)

            if over_count > 0:
                max_over_margin = float(np.nanmax(abs_tau - limit))
                first_over_idx = int(np.where(over)[0][0])
                first_over_time = float(t[first_over_idx])
            else:
                max_over_margin = 0.0
                first_over_time = np.nan

            row.update(
                {
                    "limit_Nm": limit,
                    "over_count": over_count,
                    "over_duration_s": over_duration,
                    "first_over_time_s": first_over_time,
                    "max_over_margin_Nm": max_over_margin,
                }
            )

        rows.append(row)

    return rows


def print_summary(rows, limits_enabled: bool):
    print("\n========== Joint torque summary ==========")

    if limits_enabled:
        header = (
            f"{'joint':>10} | {'rms':>9} | {'p95_abs':>9} | {'p99_abs':>9} | "
            f"{'max_abs':>9} | {'t_max':>8} | {'limit':>8} | "
            f"{'over_n':>7} | {'over_s':>8}"
        )
    else:
        header = (
            f"{'joint':>10} | {'rms':>9} | {'p95_abs':>9} | {'p99_abs':>9} | "
            f"{'max_abs':>9} | {'t_max':>8}"
        )

    print(header)
    print("-" * len(header))

    for r in rows:
        if limits_enabled:
            print(
                f"{r['joint']:>10} | "
                f"{r['rms_Nm']:9.4f} | "
                f"{r['p95_abs_Nm']:9.4f} | "
                f"{r['p99_abs_Nm']:9.4f} | "
                f"{r['max_abs_Nm']:9.4f} | "
                f"{r['max_time_s']:8.3f} | "
                f"{r['limit_Nm']:8.3f} | "
                f"{r['over_count']:7d} | "
                f"{r['over_duration_s']:8.3f}"
            )
        else:
            print(
                f"{r['joint']:>10} | "
                f"{r['rms_Nm']:9.4f} | "
                f"{r['p95_abs_Nm']:9.4f} | "
                f"{r['p99_abs_Nm']:9.4f} | "
                f"{r['max_abs_Nm']:9.4f} | "
                f"{r['max_time_s']:8.3f}"
            )


def save_csv(rows, csv_path: Path):
    if not rows:
        return

    keys = list(rows[0].keys())

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def plot_joint_torque(t, torque, joint_names, limits, out_path: Path):
    n_joints = torque.shape[1]

    fig, axes = plt.subplots(
        n_joints,
        1,
        figsize=(14, max(8, 1.8 * n_joints)),
        sharex=True,
    )

    if n_joints == 1:
        axes = [axes]

    for i, ax in enumerate(axes):
        ax.plot(t, torque[:, i], linewidth=1.0)
        ax.axhline(0.0, linewidth=0.8)

        if limits is not None:
            lim = float(limits[i])
            ax.axhline(lim, linestyle="--", linewidth=0.9)
            ax.axhline(-lim, linestyle="--", linewidth=0.9)

        ax.set_ylabel(f"{joint_names[i]}\nNm")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel("episode time [s]")
    fig.suptitle("Joint actuator torque")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def plot_abs_torque_bar(rows, out_path: Path):
    joint_names = [r["joint"] for r in rows]
    max_abs = [r["max_abs_Nm"] for r in rows]
    p99 = [r["p99_abs_Nm"] for r in rows]
    p95 = [r["p95_abs_Nm"] for r in rows]

    x = np.arange(len(joint_names))
    width = 0.25

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.bar(x - width, p95, width, label="P95 |tau|")
    ax.bar(x, p99, width, label="P99 |tau|")
    ax.bar(x + width, max_abs, width, label="max |tau|")

    ax.set_xticks(x)
    ax.set_xticklabels(joint_names, rotation=30, ha="right")
    ax.set_ylabel("Torque [Nm]")
    ax.set_title("Joint torque statistics")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()

    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(
        description="Analyze joint actuator torques in a MuJoCo HDF5 episode."
    )
    parser.add_argument(
        "episode",
        help="Path to episode.hdf5, episode folder, or root folder containing episode.hdf5 files.",
    )
    parser.add_argument(
        "--limits",
        default=None,
        help=(
            "Torque limits in Nm. Either one scalar, e.g. '20', "
            "or 7 comma-separated values, e.g. '20,20,20,20,10,10,10'."
        ),
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory. Default: <episode folder>/joint_torque_analysis",
    )

    args = parser.parse_args()

    episode_path = Path(resolve_episode_path(args.episode)).expanduser().resolve()
    # episode_path = "/home/stw/pangu/src/arm_teleop/data/peg_hole_joints_torque_test/20260621_190841_teleop_002"
    

    if args.out_dir is None:
        out_dir = episode_path.parent / "joint_torque_analysis"
    else:
        out_dir = Path(args.out_dir).expanduser().resolve()

    out_dir.mkdir(parents=True, exist_ok=True)

    with h5py.File(episode_path, "r") as h5:
        if "observations/joint_torque" not in h5:
            raise KeyError("Missing dataset: observations/joint_torque")

        torque = h5["observations/joint_torque"][:]
        t = get_time_vector(h5, torque.shape[0])
        joint_names = get_joint_names(h5, torque.shape[1])
        events = read_events(h5)

    limits = parse_limits(args.limits, torque.shape[1])
    rows = summarize_joint_torque(t, torque, joint_names, limits=limits)

    print(f"\nEpisode: {episode_path}")
    print(f"Output directory: {out_dir}")
    print(f"Samples: {torque.shape[0]}")
    print(f"Joints: {torque.shape[1]}")
    print(f"Duration: {float(t[-1] - t[0]) if len(t) > 1 else 0.0:.3f} s")

    if events:
        print("\nEvents:")
        for e in events:
            print(f"  - {e}")

        if "joint_torque_over_limit" in events:
            print("\nWARNING: HDF5 event contains joint_torque_over_limit")

    print_summary(rows, limits_enabled=limits is not None)

    csv_path = out_dir / "joint_torque_summary.csv"
    curve_path = out_dir / "joint_torque_curves.png"
    bar_path = out_dir / "joint_torque_stats.png"

    save_csv(rows, csv_path)
    plot_joint_torque(t, torque, joint_names, limits, curve_path)
    plot_abs_torque_bar(rows, bar_path)

    print("\nSaved:")
    print(f"  {csv_path}")
    print(f"  {curve_path}")
    print(f"  {bar_path}")

    if limits is not None:
        any_over = any(r["over_count"] > 0 for r in rows)
        if any_over:
            print("\nResult: torque limit exceeded.")
        else:
            print("\nResult: no torque limit exceeded in sampled joint_torque stream.")


if __name__ == "__main__":
    main()