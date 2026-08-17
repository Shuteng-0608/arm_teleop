#!/usr/bin/env python3

import argparse
import json
import math
import os
import time
from datetime import datetime

import rospy
from arm_teleop.msg import DualArmMovej
from arm_teleop.srv import (
    FeedbackService,
    FeedbackServiceRequest,
    LogService,
    LogServiceRequest,
    MovejService,
    MovejServiceRequest,
    StartDualTeleOP,
    StartDualTeleOPRequest,
)

from core.right_joint_playback import load_redundancy_trajectory


LEFT_HOME_JOINTS = (-0.046, 0.2, 0.0, 1.6, -1.32, -0.005, -0.005)
CONFIRMATION = "RIGHT_ARM_AREA_CLEAR"


def package_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_input_path():
    return os.path.join(
        package_root(),
        "data_log",
        "offline_right_ik_compare_full_20260817.csv",
    )


def default_audit_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(
        package_root(), "data_log", "right_joint_playback_{}.json".format(timestamp)
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Safely replay the validated redundancy trajectory on the right arm."
    )
    parser.add_argument("--input", default=default_input_path())
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-motion", default="")
    parser.add_argument("--service-timeout", type=float, default=15.0)
    parser.add_argument("--movej-vel", type=float, default=0.15)
    parser.add_argument("--movej-acc", type=float, default=1.0)
    parser.add_argument("--movej-jerk", type=float, default=5.0)
    parser.add_argument("--left-home-tolerance", type=float, default=0.15)
    parser.add_argument("--first-frame-tolerance", type=float, default=0.08)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--audit-output", default=None)
    args = parser.parse_args()
    positive = (
        "service_timeout",
        "movej_vel",
        "movej_acc",
        "movej_jerk",
        "left_home_tolerance",
        "first_frame_tolerance",
    )
    for name in positive:
        if getattr(args, name) <= 0.0:
            parser.error("--{} must be positive".format(name.replace("_", "-")))
    if args.hold_seconds < 0.0:
        parser.error("--hold-seconds must be non-negative")
    if args.execute and args.confirm_motion != CONFIRMATION:
        parser.error(
            "--execute requires --confirm-motion {}".format(CONFIRMATION)
        )
    return args


def maximum_error(actual, expected):
    return max(abs(a - b) for a, b in zip(actual, expected))


def finite_vector(values, length, name):
    result = [float(value) for value in values]
    if len(result) != length or not all(math.isfinite(value) for value in result):
        raise RuntimeError("{} feedback is invalid".format(name))
    return result


def call_feedback(service):
    request = FeedbackServiceRequest()
    request.request = "right_joint_playback"
    response = service.call(request)
    return {
        "left": finite_vector(response.joints_left, 7, "left arm"),
        "right": finite_vector(response.joints_right, 7, "right arm"),
        "others": finite_vector(response.joints_others, 5, "other joints"),
    }


def call_movej(service, joints, args):
    request = MovejServiceRequest()
    request.arm_id = 1
    request.target_joints = list(joints)
    request.vel = args.movej_vel
    request.acc = args.movej_acc
    request.jerk = args.movej_jerk
    service.call(request)


def set_teleop(service, running):
    request = StartDualTeleOPRequest()
    request.running_flag = running
    service.call(request)


def set_log(service, state):
    request = LogServiceRequest()
    request.log_state = state
    service.call(request)


def wait_until(deadline):
    while not rospy.is_shutdown():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        time.sleep(min(remaining, 0.01))
    raise rospy.ROSInterruptException("ROS shutdown during playback")


def make_message(sequence, right_joints, left_joints, head_z_rotation):
    message = DualArmMovej()
    message.header.stamp = rospy.Time.now()
    message.header.frame_id = "pangu_base"
    message.sequence = sequence
    message.right_arm.arm_id = 1
    message.right_arm.arm_joints = list(right_joints)
    message.left_arm.arm_id = 0
    message.left_arm.arm_joints = list(left_joints)
    message.head_z_rotation = head_z_rotation
    return message


def write_audit(path, payload):
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if os.path.exists(path):
        raise FileExistsError("refusing to overwrite audit output: {}".format(path))
    with open(path, "x", encoding="utf-8") as output_file:
        json.dump(payload, output_file, ensure_ascii=True, indent=2, sort_keys=True)
        output_file.write("\n")
    return path


def execute(args, frames, summary):
    rospy.init_node("right_joint_trajectory_playback", anonymous=True)
    service_names = (
        "/aris_node/feedback_srv",
        "/aris_node/movej_srv",
        "/aris_node/start_teleop_srv",
        "/aris_node/log_srv",
    )
    for service_name in service_names:
        rospy.wait_for_service(service_name, timeout=args.service_timeout)

    feedback_service = rospy.ServiceProxy(service_names[0], FeedbackService)
    movej_service = rospy.ServiceProxy(service_names[1], MovejService)
    teleop_service = rospy.ServiceProxy(service_names[2], StartDualTeleOP)
    log_service = rospy.ServiceProxy(service_names[3], LogService)
    publisher = rospy.Publisher(
        "/arm_teleop/dual_arm_movej", DualArmMovej, queue_size=20
    )

    before = call_feedback(feedback_service)
    left_home_error = maximum_error(before["left"], LEFT_HOME_JOINTS)
    if left_home_error > args.left_home_tolerance:
        raise RuntimeError(
            "left arm is {:.6f} rad away from the stop-service home pose; "
            "refusing right-only playback".format(left_home_error)
        )
    head_z_rotation = before["others"][4] / 0.8
    if abs(head_z_rotation) > math.pi / 4.0 + 1e-6:
        raise RuntimeError("current auxiliary-axis target is outside the teleop clamp")

    audit = {
        "trajectory": summary.__dict__,
        "started_at": datetime.now().isoformat(),
        "before_movej": before,
        "left_home_max_error_rad": left_home_error,
        "first_frame_from_start_max_delta_rad": maximum_error(
            before["right"], frames[0].joints
        ),
        "settings": {
            "movej_vel": args.movej_vel,
            "movej_acc": args.movej_acc,
            "movej_jerk": args.movej_jerk,
            "hold_seconds": args.hold_seconds,
        },
    }

    teleop_started = False
    log_started = False
    try:
        rospy.logwarn("Moving only the right arm to the first trajectory frame")
        call_movej(movej_service, frames[0].joints, args)
        after_movej = call_feedback(feedback_service)
        audit["after_movej"] = after_movej
        first_frame_error = maximum_error(after_movej["right"], frames[0].joints)
        audit["first_frame_max_error_rad"] = first_frame_error
        if first_frame_error > args.first_frame_tolerance:
            raise RuntimeError(
                "right arm failed first-frame verification: {:.6f} rad > {:.6f} rad".format(
                    first_frame_error, args.first_frame_tolerance
                )
            )

        connection_deadline = time.monotonic() + args.service_timeout
        while publisher.get_num_connections() < 1:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("lower controller did not subscribe to the playback topic")
            time.sleep(0.05)

        set_log(log_service, "start_log")
        log_started = True
        set_teleop(teleop_service, True)
        teleop_started = True
        started = time.monotonic()
        source_start = frames[0].timestamp
        for sequence, frame in enumerate(frames):
            wait_until(started + frame.timestamp - source_start)
            publisher.publish(
                make_message(
                    sequence,
                    frame.joints,
                    before["left"],
                    head_z_rotation,
                )
            )

        hold_deadline = time.monotonic() + args.hold_seconds
        sequence = len(frames)
        while time.monotonic() < hold_deadline:
            publisher.publish(
                make_message(
                    sequence,
                    frames[-1].joints,
                    before["left"],
                    head_z_rotation,
                )
            )
            sequence += 1
            time.sleep(1.0 / 30.0)
        audit["before_stop"] = call_feedback(feedback_service)
        audit["completed_frames"] = len(frames)
    finally:
        if log_started:
            try:
                set_log(log_service, "end_log")
                time.sleep(0.1)
            except Exception as error:
                rospy.logerr("Failed to stop lower-controller logging: %s", error)
        if teleop_started:
            try:
                set_teleop(teleop_service, False)
            except Exception as error:
                rospy.logerr("Failed to stop lower-controller teleop: %s", error)
        audit["finished_at"] = datetime.now().isoformat()
        audit["teleop_stop_requested"] = teleop_started
        audit_path = write_audit(args.audit_output or default_audit_path(), audit)
        rospy.loginfo("Playback audit written to %s", audit_path)


def main():
    args = parse_args()
    input_path = os.path.abspath(args.input)
    frames, summary = load_redundancy_trajectory(input_path)
    print("Validated trajectory: {}".format(input_path))
    print(
        "frames={}, duration={:.3f}s, dt=[{:.6f}, {:.6f}]s".format(
            summary.frame_count,
            summary.duration,
            summary.minimum_dt,
            summary.maximum_dt,
        )
    )
    print(
        "max_step={:.6f}rad, max_velocity={:.6f}rad/s, sha256={}".format(
            summary.maximum_step,
            summary.maximum_velocity,
            summary.file_sha256,
        )
    )
    if not args.execute:
        print("Dry-run only: no ROS interfaces were opened and no command was sent.")
        return
    execute(args, frames, summary)


if __name__ == "__main__":
    main()
