#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard client for MuJoCo HDF5 episode recording.

Usage:
    rosrun arm_teleop recording_keyboard_client.py

Behavior:
    Press Enter:
        - if not recording: start recording
        - if recording: ask whether to keep this episode, then stop

Service:
    /mujoco_hdf5_recording/set_recording
"""

from __future__ import annotations

import time
import rospy

from arm_teleop.srv import SetRecording, SetRecordingRequest


def ask_keep() -> bool:
    ans = input("Stop recording. Keep this episode? [Y/n]: ").strip().lower()
    return ans not in {"n", "no", "0", "false"}


def main() -> None:
    rospy.init_node("mujoco_recording_keyboard_client", anonymous=True)

    service_name = rospy.get_param(
        "~service_name",
        "/mujoco_hdf5_recording/set_recording",
    )

    rospy.loginfo("Waiting for recording service: %s", service_name)
    rospy.wait_for_service(service_name)

    proxy = rospy.ServiceProxy(service_name, SetRecording)

    recording = False
    episode_count = 0

    print("")
    print("MuJoCo HDF5 recording keyboard client")
    print("------------------------------------")
    print("Press Enter to start recording.")
    print("Press Enter again to stop recording.")
    print("Ctrl+C to exit.")
    print("")

    while not rospy.is_shutdown():
        try:
            input("Press Enter... ")
        except EOFError:
            break
        except KeyboardInterrupt:
            print("")
            break

        if not recording:
            episode_count += 1
            label = f"teleop_{episode_count:03d}"

            req = SetRecordingRequest()
            req.record = True
            req.keep = True
            req.label = label

            try:
                resp = proxy(req)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to start recording: %s", e)
                continue

            recording = bool(resp.active)

            print(resp.message)
            if resp.episode_path:
                print("Episode path:", resp.episode_path)

        else:
            keep = ask_keep()

            req = SetRecordingRequest()
            req.record = False
            req.keep = keep
            req.label = ""

            try:
                resp = proxy(req)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to stop recording: %s", e)
                continue

            recording = bool(resp.active)

            print(resp.message)
            if resp.episode_path:
                print("Episode path:", resp.episode_path)

        time.sleep(0.1)


if __name__ == "__main__":
    main()
