#!/usr/bin/env python3
"""
trim_mujoco_hdf5_episode.py

Crop a MuJoCo peg-in-hole HDF5 episode by time and write a new trimmed episode.

Usage:
    python3 scripts/trim_mujoco_hdf5_episode.py \
        --episode data/peg_in_hole_hdf5/20260528_200642_teleop/episode.hdf5 \
        --start 12.0 \
        --end 22.3 \
        --time-mode sim \
        --copy-images

Output:
    <original_episode_dir>/trim_sim_12p000_22p300/episode.hdf5

This script preserves the multi-rate structure:
    - state stream is cropped with timestamps/state
    - force stream is cropped with timestamps/force
    - image stream is cropped per camera with observations/images/<camera>/timestamps
    - events are cropped with events/t_sim or events/t_episode
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Iterable

import h5py
import numpy as np


STATE_DATASETS = [
    "observations/qpos",
    "observations/qvel",
    "observations/ee_pose",
    "observations/link7_pose",
    "observations/peg_tip_pos",
    "observations/hole_center_pos",
    "observations/force_torque_state_sample",
    "actions/q_target",
    "actions/q_cmd",
    "actions/ctrl",
    "task/state_error_xyz",
    "task/state_align_err_xz",
    "task/state_insertion_err_y",
    "contact/state_peg_contact",
    "contact/state_peg_contact_min_dist",
]

FORCE_DATASETS = [
    "observations/force_torque",
    "task/force_error_xyz",
    "task/force_align_err_xz",
    "task/force_insertion_err_y",
    "contact/force_peg_contact",
    "contact/force_peg_contact_min_dist",
    "actions/q_target_force_sample",
    "actions/q_cmd_force_sample",
]

EVENT_DATASETS = [
    "events/names",
    "events/t_sim",
    "events/t_episode",
]


def decode_str(x) -> str:
    if isinstance(x, bytes):
        return x.decode("utf-8")
    return str(x)


def copy_attrs(src_obj, dst_obj) -> None:
    for key, value in src_obj.attrs.items():
        dst_obj.attrs[key] = value


def ensure_parent_group(dst: h5py.File, path: str) -> None:
    parent = "/".join(path.split("/")[:-1])
    if parent:
        dst.require_group(parent)


def write_dataset(dst: h5py.File, path: str, data, dtype=None) -> None:
    ensure_parent_group(dst, path)
    if path in dst:
        del dst[path]
    parent_path, name = path.rsplit("/", 1)
    parent = dst[parent_path] if parent_path else dst
    if dtype is None:
        parent.create_dataset(name, data=data)
    else:
        parent.create_dataset(name, data=data, dtype=dtype)


def crop_dataset(src: h5py.File, dst: h5py.File, path: str, mask: np.ndarray) -> None:
    if path not in src:
        return

    data = src[path][:]
    if data.shape[0] != len(mask):
        print(f"[skip] {path}: first dim {data.shape[0]} != mask length {len(mask)}")
        return

    write_dataset(dst, path, data[mask])
    if path in dst:
        copy_attrs(src[path], dst[path])


def copy_dataset(src: h5py.File, dst: h5py.File, path: str) -> None:
    if path not in src:
        return
    write_dataset(dst, path, src[path][:])
    copy_attrs(src[path], dst[path])


def copy_group_attrs(src: h5py.File, dst: h5py.File) -> None:
    def visitor(name, obj):
        if isinstance(obj, h5py.Group):
            dst.require_group(name)
            copy_attrs(obj, dst[name])

    src.visititems(visitor)


def safe_time_tag(x: float) -> str:
    return f"{x:.3f}".replace(".", "p").replace("-", "m")


def crop_standard_streams(
    src: h5py.File,
    dst: h5py.File,
    start: float,
    end: float,
    time_mode: str,
) -> dict:
    stats = {}

    # State stream
    state_time_path = "timestamps/state_episode" if time_mode == "episode" else "timestamps/state"
    if state_time_path in src:
        t_state = src[state_time_path][:]
        state_mask = (t_state >= start) & (t_state <= end)

        crop_dataset(src, dst, "timestamps/state", state_mask)
        crop_dataset(src, dst, "timestamps/state_episode", state_mask)

        for path in STATE_DATASETS:
            crop_dataset(src, dst, path, state_mask)

        stats["state_kept"] = int(np.sum(state_mask))
        stats["state_total"] = int(len(state_mask))

    # Force stream
    force_time_path = "timestamps/force_episode" if time_mode == "episode" else "timestamps/force"
    if force_time_path in src:
        t_force = src[force_time_path][:]
        force_mask = (t_force >= start) & (t_force <= end)

        crop_dataset(src, dst, "timestamps/force", force_mask)
        crop_dataset(src, dst, "timestamps/force_episode", force_mask)

        for path in FORCE_DATASETS:
            crop_dataset(src, dst, path, force_mask)

        stats["force_kept"] = int(np.sum(force_mask))
        stats["force_total"] = int(len(force_mask))

    # Events
    event_time_path = "events/t_episode" if time_mode == "episode" else "events/t_sim"
    if event_time_path in src:
        t_event = src[event_time_path][:]
        event_mask = (t_event >= start) & (t_event <= end)

        for path in EVENT_DATASETS:
            crop_dataset(src, dst, path, event_mask)

        stats["events_kept"] = int(np.sum(event_mask))
        stats["events_total"] = int(len(event_mask))

    return stats


def crop_image_streams(
    src: h5py.File,
    dst: h5py.File,
    original_episode_dir: Path,
    trimmed_episode_dir: Path,
    start: float,
    end: float,
    time_mode: str,
    copy_images: bool,
) -> dict:
    stats = {}

    if "observations/images" not in src:
        return stats

    src_images = src["observations/images"]
    dst_images = dst.require_group("observations").require_group("images")
    copy_attrs(src_images, dst_images)

    str_dtype = h5py.string_dtype(encoding="utf-8")

    if "observations/images/camera_names" in src:
        camera_names = [decode_str(x) for x in src["observations/images/camera_names"][:]]
    else:
        camera_names = [
            name for name in src_images.keys()
            if isinstance(src_images[name], h5py.Group)
        ]

    write_dataset(
        dst,
        "observations/images/camera_names",
        np.asarray(camera_names, dtype=object),
        dtype=str_dtype,
    )

    all_image_times = []
    all_image_episode_times = []

    for cam in camera_names:
        cam_src_path = f"observations/images/{cam}"
        if cam_src_path not in src:
            continue

        cam_src = src[cam_src_path]
        if "timestamps" not in cam_src or "file_paths" not in cam_src:
            continue

        cam_dst = dst.require_group(cam_src_path)
        copy_attrs(cam_src, cam_dst)

        cam_time_name = "timestamps_episode" if time_mode == "episode" and "timestamps_episode" in cam_src else "timestamps"
        t_cam = cam_src[cam_time_name][:]
        mask = (t_cam >= start) & (t_cam <= end)

        for dset_name in cam_src.keys():
            if dset_name == "file_paths":
                continue

            data = cam_src[dset_name][:]
            if data.shape[0] == len(mask):
                write_dataset(dst, f"{cam_src_path}/{dset_name}", data[mask])
            else:
                write_dataset(dst, f"{cam_src_path}/{dset_name}", data)

            if f"{cam_src_path}/{dset_name}" in dst:
                copy_attrs(cam_src[dset_name], dst[f"{cam_src_path}/{dset_name}"])

        old_paths = [decode_str(x) for x in cam_src["file_paths"][:]]
        selected_indices = np.where(mask)[0]
        selected_paths = [old_paths[i] for i in selected_indices]
        new_paths = []

        for rel in selected_paths:
            src_img = original_episode_dir / rel
            rel_path = Path(rel)

            if copy_images:
                dst_img = trimmed_episode_dir / rel_path
                dst_img.parent.mkdir(parents=True, exist_ok=True)
                if src_img.exists():
                    shutil.copy2(src_img, dst_img)
                else:
                    print(f"[warning] missing image: {src_img}")
                new_paths.append(str(rel_path))
            else:
                # Keep absolute path to original file if not copying.
                new_paths.append(str(src_img.resolve()))

        write_dataset(
            dst,
            f"{cam_src_path}/file_paths",
            np.asarray(new_paths, dtype=object),
            dtype=str_dtype,
        )

        if "timestamps" in cam_src:
            ts = cam_src["timestamps"][:][mask]
            all_image_times.extend(ts.tolist())
        if "timestamps_episode" in cam_src:
            tse = cam_src["timestamps_episode"][:][mask]
            all_image_episode_times.extend(tse.tolist())

        stats[f"{cam}_images_kept"] = int(np.sum(mask))
        stats[f"{cam}_images_total"] = int(len(mask))

    write_dataset(dst, "timestamps/image", np.asarray(all_image_times, dtype=np.float64))
    write_dataset(dst, "timestamps/image_episode", np.asarray(all_image_episode_times, dtype=np.float64))

    return stats


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", required=True, help="Source episode.hdf5")
    parser.add_argument("--start", type=float, required=True, help="Crop start time")
    parser.add_argument("--end", type=float, required=True, help="Crop end time")
    parser.add_argument(
        "--time-mode",
        choices=["sim", "episode"],
        default="sim",
        help="Interpret start/end as t_sim or t_episode.",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="Output directory. Default: <episode_dir>/trim_<mode>_<start>_<end>",
    )
    parser.add_argument(
        "--copy-images",
        action="store_true",
        help="Copy selected image files into trimmed folder.",
    )
    args = parser.parse_args()

    src_h5 = Path(args.episode).expanduser().resolve()
    if not src_h5.exists():
        raise FileNotFoundError(src_h5)

    if args.end <= args.start:
        raise ValueError("--end must be greater than --start")

    original_episode_dir = src_h5.parent

    if args.out_dir:
        out_dir = Path(args.out_dir).expanduser().resolve()
    else:
        out_dir = original_episode_dir / f"trim_{args.time_mode}_{safe_time_tag(args.start)}_{safe_time_tag(args.end)}"

    out_dir.mkdir(parents=True, exist_ok=True)
    out_h5 = out_dir / "episode.hdf5"

    with h5py.File(src_h5, "r") as src, h5py.File(out_h5, "w") as dst:
        copy_attrs(src, dst)
        dst.attrs["trimmed"] = 1
        dst.attrs["trim_start"] = float(args.start)
        dst.attrs["trim_end"] = float(args.end)
        dst.attrs["trim_time_mode"] = args.time_mode
        dst.attrs["duration_trim"] = float(args.end - args.start)

        copy_group_attrs(src, dst)

        stats = {}
        stats.update(crop_standard_streams(src, dst, args.start, args.end, args.time_mode))
        stats.update(
            crop_image_streams(
                src=src,
                dst=dst,
                original_episode_dir=original_episode_dir,
                trimmed_episode_dir=out_dir,
                start=args.start,
                end=args.end,
                time_mode=args.time_mode,
                copy_images=args.copy_images,
            )
        )

    sidecar = {
        "source_episode": str(src_h5),
        "trimmed_episode": str(out_h5),
        "trim_start": args.start,
        "trim_end": args.end,
        "trim_time_mode": args.time_mode,
        "copy_images": args.copy_images,
        "stats": stats,
    }

    with (out_dir / "trim_metadata.json").open("w", encoding="utf-8") as f:
        json.dump(sidecar, f, indent=2, ensure_ascii=False)

    print("Trimmed episode saved:")
    print(f"  {out_h5}")
    print("")
    print("Stats:")
    for k, v in stats.items():
        print(f"  {k}: {v}")
    print("")
    print("Next:")
    print(f"  python3 scripts/analyze_mujoco_hdf5_episode.py --episode {out_h5}")


if __name__ == "__main__":
    main()
