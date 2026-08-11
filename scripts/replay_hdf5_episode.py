#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Replay / inspect MuJoCo peg-in-hole HDF5 files.

This is a data-level replay tool. It does not command MuJoCo or the robot.
It reads either:

  1. stage2 training file: episode.hdf5
     - /action
     - /observations/*
     - /timestamps/*

  2. stage1 diagnostic file: stage1_trace.hdf5
     - /stage1_trace/action_command
     - /stage1_trace/ft_wrench
     - /stage1_trace/joint_pos
     - /stage1_trace/ee_pose
     - /stage1_trace/timestamps

Typical usage:

  python3 scripts/replay_hdf5_episode.py \
      data/hole_random_60mm_hmj/20260811_xxxxxx_scripted/episode.hdf5 --summary

  python3 scripts/replay_hdf5_episode.py \
      data/hole_random_60mm_hmj/20260811_xxxxxx_scripted/stage1_trace.hdf5 \
      --play --hz 5

  python3 scripts/replay_hdf5_episode.py episode.hdf5 \
      --export-images /tmp/hdf5_frames --camera ee_cam --image-stride 30

  python3 scripts/replay_hdf5_episode.py episode.hdf5 \
      --export-video replay.mp4 --camera ee_cam --video-fps 30
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Any, Dict, Iterable, Optional, Tuple

import h5py
import numpy as np


def dataset_info(f: h5py.File) -> Iterable[str]:
    """Yield readable HDF5 tree lines."""

    def visitor(name: str, obj):
        if isinstance(obj, h5py.Dataset):
            lines.append(f"{name}: shape={tuple(obj.shape)} dtype={obj.dtype}")
        elif isinstance(obj, h5py.Group):
            lines.append(f"{name}/")

    lines = []
    f.visititems(visitor)
    return lines


def read_array(f: h5py.File, path: str, default=None):
    if path in f:
        return f[path][()]
    return default


def finite_minmax(x: Optional[np.ndarray]) -> Tuple[Optional[float], Optional[float]]:
    if x is None:
        return None, None
    arr = np.asarray(x, dtype=float)
    mask = np.isfinite(arr)
    if not np.any(mask):
        return None, None
    return float(np.nanmin(arr)), float(np.nanmax(arr))


def detect_mode(f: h5py.File, requested: str) -> str:
    if requested != "auto":
        return requested
    if "stage1_trace/action_command" in f:
        return "stage1"
    if "action" in f or "actions/joint_pos_command" in f:
        return "stage2"
    raise ValueError("Cannot detect HDF5 mode: expected episode.hdf5 or stage1_trace.hdf5")


def load_streams(f: h5py.File, mode: str) -> Dict[str, Any]:
    """Load the core streams for text replay."""
    if mode == "stage1":
        ts = read_array(f, "stage1_trace/timestamps")
        if ts is not None and ts.ndim == 2:
            t = ts[:, 0]  # wall-relative time, stage1-local
            t_sim = ts[:, 1] if ts.shape[1] > 1 else None
        else:
            t = np.arange(len(read_array(f, "stage1_trace/action_command", [])), dtype=float)
            t_sim = None
        return {
            "mode": mode,
            "t": np.asarray(t, dtype=float),
            "t_sim": None if t_sim is None else np.asarray(t_sim, dtype=float),
            "action": read_array(f, "stage1_trace/action_command"),
            "qpos": read_array(f, "stage1_trace/joint_pos"),
            "ee_pose": read_array(f, "stage1_trace/ee_pose"),
            "ft_t": np.asarray(t, dtype=float),
            "ft": read_array(f, "stage1_trace/ft_wrench"),
        }

    # stage2 / episode.hdf5
    t = read_array(f, "timestamps/state_episode")
    if t is None:
        t = read_array(f, "timestamps/state")
    action = read_array(f, "action")
    if action is None:
        action = read_array(f, "actions/joint_pos_command")
    ft_t = read_array(f, "timestamps/force_episode")
    if ft_t is None:
        ft_t = read_array(f, "timestamps/force")
    return {
        "mode": mode,
        "t": np.asarray(t, dtype=float) if t is not None else np.arange(len(action), dtype=float),
        "t_sim": None,
        "action": action,
        "qpos": read_array(f, "observations/joint_pos"),
        "ee_pose": read_array(f, "observations/ee_pose"),
        "ft_t": np.asarray(ft_t, dtype=float) if ft_t is not None else None,
        "ft": read_array(f, "observations/ft_wrench"),
    }


def nearest_force_norm(ft_t, ft, t_value: float) -> Optional[float]:
    if ft is None or len(ft) == 0:
        return None
    if ft_t is None or len(ft_t) == 0:
        idx = 0
    else:
        idx = int(np.searchsorted(ft_t, t_value))
        idx = max(0, min(idx, len(ft) - 1))
    row = np.asarray(ft[idx], dtype=float)
    if row.size < 3 or not np.all(np.isfinite(row[:3])):
        return None
    return float(np.linalg.norm(row[:3]))


def print_summary(path: Path, f: h5py.File, mode: str, streams: Dict[str, Any]) -> None:
    print(f"file: {path}")
    print(f"mode: {mode}")
    print("\n[tree]")
    for line in dataset_info(f):
        print("  " + line)

    print("\n[core streams]")
    for name in ("t", "action", "qpos", "ee_pose", "ft"):
        arr = streams.get(name)
        if arr is None:
            print(f"  {name}: MISSING")
            continue
        mn, mx = finite_minmax(arr)
        print(f"  {name}: shape={tuple(np.asarray(arr).shape)} min={mn} max={mx}")

    t = streams.get("t")
    if t is not None and len(t) >= 2:
        dt = np.diff(t)
        dt = dt[dt > 0]
        if len(dt):
            print(
                f"  time: {float(t[0]):.3f} -> {float(t[-1]):.3f}s, "
                f"duration={float(t[-1]-t[0]):.3f}s, median_hz={1.0/float(np.median(dt)):.2f}"
            )

    ft = streams.get("ft")
    if ft is not None and len(ft):
        force_norm = np.linalg.norm(np.asarray(ft)[:, :3], axis=1)
        print(
            f"  force_norm: min={float(np.nanmin(force_norm)):.3f} "
            f"mean={float(np.nanmean(force_norm)):.3f} "
            f"max={float(np.nanmax(force_norm)):.3f}"
        )

    print("\n[attrs]")
    for group_name in ("episode_metadata", "stage1_trace"):
        if group_name in f:
            print(f"  {group_name}:")
            for k, v in f[group_name].attrs.items():
                print(f"    {k}: {v}")


def fmt_vec(x, n: int = 3) -> str:
    if x is None:
        return "-"
    arr = np.asarray(x, dtype=float).reshape(-1)
    if arr.size == 0:
        return "-"
    vals = ", ".join(f"{v:+.3f}" for v in arr[:n])
    if arr.size > n:
        vals += ", ..."
    return "[" + vals + "]"


def replay_text(streams: Dict[str, Any], args) -> None:
    t = streams["t"]
    action = streams.get("action")
    qpos = streams.get("qpos")
    ee = streams.get("ee_pose")
    ft_t = streams.get("ft_t")
    ft = streams.get("ft")

    if action is None:
        raise ValueError("No action/action_command stream found for replay")

    n = min(len(t), len(action))
    start = max(0, int(args.start_index))
    stride = max(1, int(args.stride))
    indices = list(range(start, n, stride))
    if args.max_steps is not None:
        indices = indices[:max(0, int(args.max_steps))]

    print(f"[replay] samples={n}, showing {len(indices)} rows from start={start}, stride={stride}, mode={streams['mode']}")
    prev_t = None
    for i in indices:
        ti = float(t[i]) if i < len(t) else float(i)
        fn = nearest_force_norm(ft_t, ft, ti)
        qrow = qpos[i] if qpos is not None and i < len(qpos) else None
        erow = ee[i] if ee is not None and i < len(ee) else None
        arow = action[i] if i < len(action) else None

        print(
            f"i={i:06d} t={ti:8.3f}s "
            f"|F|={fn if fn is not None else math.nan:8.3f} "
            f"action={fmt_vec(arow, 4)} qpos={fmt_vec(qrow, 4)} ee_xyz={fmt_vec(erow, 3)}"
        )

        if args.realtime:
            if prev_t is not None:
                dt = max(0.0, (ti - prev_t) / max(float(args.speed), 1e-6))
                time.sleep(min(dt, 2.0))
            prev_t = ti
        elif args.hz > 0:
            time.sleep(1.0 / float(args.hz))


def find_image_dataset(f: h5py.File, camera: Optional[str]):
    if "observations/images" not in f:
        return None, None
    g = f["observations/images"]
    keys = [k for k in g.keys() if isinstance(g[k], h5py.Dataset) and g[k].ndim >= 3]
    if not keys:
        return None, None
    cam = camera if camera is not None else keys[0]
    if cam not in g:
        raise ValueError(f"Camera '{cam}' not found. Available: {keys}")
    return cam, g[cam]


def write_png(path: Path, image: np.ndarray) -> None:
    try:
        from PIL import Image
        Image.fromarray(image).save(path)
        return
    except Exception:
        pass

    try:
        import imageio.v2 as imageio
        imageio.imwrite(path, image)
        return
    except Exception as exc:
        raise RuntimeError("Install pillow or imageio to export PNG images") from exc


def export_images(f: h5py.File, out_dir: Path, camera: Optional[str], stride: int, max_images: int) -> None:
    cam, ds = find_image_dataset(f, camera)
    if ds is None:
        print("[images] no image dataset found")
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    stride = max(1, int(stride))
    count = 0
    for i in range(0, len(ds), stride):
        if count >= max_images:
            break
        out_path = out_dir / f"{cam}_{i:06d}.png"
        write_png(out_path, ds[i])
        count += 1
    print(f"[images] exported {count} frames from camera '{cam}' to {out_dir}")


def export_video(
    f: h5py.File,
    out_path: Path,
    camera: Optional[str],
    fps: float,
    stride: int,
    max_frames: Optional[int],
) -> None:
    """Export an HDF5 image stream to an MP4 video using OpenCV."""
    cam, ds = find_image_dataset(f, camera)
    if ds is None:
        print("[video] no image dataset found")
        return

    try:
        import cv2
    except Exception as exc:
        raise RuntimeError("OpenCV/cv2 is required for --export-video") from exc

    out_path.parent.mkdir(parents=True, exist_ok=True)
    stride = max(1, int(stride))
    fps = float(fps) if fps and fps > 0 else 30.0

    first = np.asarray(ds[0])
    if first.ndim != 3 or first.shape[2] not in (3, 4):
        raise ValueError(f"Expected RGB/RGBA image stream, got shape {first.shape}")
    height, width = int(first.shape[0]), int(first.shape[1])

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open video writer for {out_path}")

    count = 0
    try:
        for i in range(0, len(ds), stride):
            if max_frames is not None and count >= int(max_frames):
                break
            frame = np.asarray(ds[i])
            if frame.shape[2] == 4:
                frame = frame[:, :, :3]
            # HDF5 stores RGB; OpenCV expects BGR.
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            writer.write(frame_bgr)
            count += 1
    finally:
        writer.release()

    print(
        f"[video] exported {count} frames from camera '{cam}' "
        f"to {out_path} at {fps:.2f} fps, stride={stride}"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("hdf5", type=Path, help="episode.hdf5 or stage1_trace.hdf5")
    ap.add_argument("--mode", choices=["auto", "stage1", "stage2"], default="auto")
    ap.add_argument("--summary", action="store_true", help="print HDF5 tree and stream summary")
    ap.add_argument("--play", action="store_true", help="text replay action/qpos/force streams")
    ap.add_argument("--hz", type=float, default=0.0, help="text replay print rate; 0 means no sleep")
    ap.add_argument("--realtime", action="store_true", help="sleep according to timestamps")
    ap.add_argument("--speed", type=float, default=1.0, help="realtime playback speed multiplier")
    ap.add_argument("--stride", type=int, default=1, help="show every Nth sample")
    ap.add_argument("--start-index", type=int, default=0)
    ap.add_argument("--max-steps", type=int, default=80)
    ap.add_argument("--export-images", type=Path, default=None, help="export image frames to directory")
    ap.add_argument("--camera", type=str, default=None, help="camera name for image export")
    ap.add_argument("--image-stride", type=int, default=30)
    ap.add_argument("--max-images", type=int, default=100)
    ap.add_argument("--export-video", type=Path, default=None, help="export image stream to MP4")
    ap.add_argument("--video-fps", type=float, default=30.0)
    ap.add_argument("--video-stride", type=int, default=1)
    ap.add_argument("--max-video-frames", type=int, default=None)
    args = ap.parse_args()

    if not args.hdf5.exists():
        raise FileNotFoundError(args.hdf5)

    with h5py.File(args.hdf5, "r") as f:
        mode = detect_mode(f, args.mode)
        streams = load_streams(f, mode)

        did_export_only = args.export_images is not None or args.export_video is not None
        if args.summary or (not args.play and not did_export_only):
            print_summary(args.hdf5, f, mode, streams)

        if args.export_images is not None:
            export_images(f, args.export_images, args.camera, args.image_stride, args.max_images)

        if args.export_video is not None:
            export_video(
                f,
                args.export_video,
                args.camera,
                args.video_fps,
                args.video_stride,
                args.max_video_frames,
            )

        if args.play:
            replay_text(streams, args)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
