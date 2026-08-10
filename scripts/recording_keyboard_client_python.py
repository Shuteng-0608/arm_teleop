#!/usr/bin/env python3
"""Keyboard client for pure-Python Marvin HDF5 episode recording."""

from __future__ import annotations

import argparse
import json
import socket
import sys
import time

DEFAULT_RECORDING_HOST = "127.0.0.1"
DEFAULT_RECORDING_PORT = 8765


def send_recording_command(request, host, port, timeout):
    """Use only the Python standard library in the separate client process."""
    payload = (json.dumps(request, ensure_ascii=False) + "\n").encode("utf-8")
    with socket.create_connection((host, port), timeout=timeout) as sock:
        sock.sendall(payload)
        response = sock.makefile("rb").readline(1024 * 1024)
    if not response:
        raise ConnectionError("recording server closed without a response")
    result = json.loads(response.decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("recording server returned a non-object response")
    return result


def _parse_args():
    parser = argparse.ArgumentParser(
        description="Control Marvin MuJoCo HDF5 episodes without ROS."
    )
    parser.add_argument("--host", default=DEFAULT_RECORDING_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_RECORDING_PORT)
    parser.add_argument("--label-prefix", default="marvin_teleop")
    parser.add_argument("--timeout", type=float, default=5.0)
    return parser.parse_args()


def ask_keep() -> bool:
    answer = input("Keep this episode? [Y/n]: ").strip().lower()
    return answer not in {"n", "no", "0", "false"}


def _request(args, command, **values):
    return send_recording_command(
        {"command": command, **values},
        host=args.host,
        port=args.port,
        timeout=args.timeout,
    )


def _print_result(result) -> None:
    print(result.get("message", ""))
    if result.get("episode_path"):
        print("Episode path:", result["episode_path"])
    if result.get("hole_cell"):
        print("Current/next hole:", result["hole_cell"])


def main() -> int:
    args = _parse_args()
    try:
        initial = _request(args, "status")
    except Exception as exc:
        print(
            "Cannot connect to the pure-Python recording server at "
            f"{args.host}:{args.port}: {exc}",
            file=sys.stderr,
        )
        print("Start vptele/main_mujoco_marvin.py first.", file=sys.stderr)
        return 1

    print("")
    print("Marvin MuJoCo pure-Python recording client")
    print("-------------------------------------------")
    print("Press Enter to start or stop/review an episode.")
    print("Ctrl+C to exit the client; simulation keeps running.")
    _print_result(initial)
    print("")

    episode_count = 0
    while True:
        try:
            input("Press Enter... ")
            status = _request(args, "status")
            if status.get("active") or status.get("pending_review"):
                result = _request(args, "stop", keep=ask_keep())
            else:
                episode_count += 1
                label = f"{args.label_prefix}_{episode_count:03d}"
                result = _request(args, "start", label=label)
            _print_result(result)
            time.sleep(0.1)
        except (EOFError, KeyboardInterrupt):
            print("")
            return 0
        except Exception as exc:
            print(f"Recording command failed: {exc}", file=sys.stderr)
            time.sleep(0.5)


if __name__ == "__main__":
    raise SystemExit(main())
