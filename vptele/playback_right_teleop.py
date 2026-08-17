#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time
from datetime import datetime

import rospy
from arm_teleop.msg import DualArmMovej
from arm_teleop.srv import ArmIK, ArmIKRequest
from arm_teleop.srv import (
    FeedbackService,
    LogService,
    MovejService,
    StartDualTeleOP,
)

from core.right_arm_trajectory import (
    INITIAL_RIGHT_ARM_ANGLE,
    INITIAL_RIGHT_JOINTS,
    RightArmTrajectoryMapper,
)
from core.right_teleop_playback import (
    load_teleop_trajectory,
    rounded_solver_state,
    validate_online_solution,
)
from playback_right_joint_trajectory import (
    LEFT_HOME_JOINTS,
    call_feedback,
    call_movej,
    make_message,
    maximum_error,
    set_log,
    set_teleop,
    wait_until,
    write_audit,
)


def package_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_input_path():
    return os.path.join(package_root(), "data_log", "circle_engineering.csv")


def timestamped_output(suffix):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(
        package_root(), "data_log", "online_right_teleop_{}{}".format(timestamp, suffix)
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Replay a recorded right-hand trajectory through the online "
            "redundancy IK service and publish each new solution. "
            "Running without mode flags executes robot motion."
        )
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--ik-only", action="store_true")
    mode.add_argument("--preflight", action="store_true")
    parser.add_argument("--input", default=default_input_path())
    parser.add_argument("--output", default=None)
    parser.add_argument("--audit-output", default=None)
    parser.add_argument(
        "--right-ik-service", default="/arm_teleop/right_arm_ik_srv"
    )
    parser.add_argument("--service-timeout", type=float, default=15.0)
    parser.add_argument("--movej-vel", type=float, default=0.15)
    parser.add_argument("--movej-acc", type=float, default=1.0)
    parser.add_argument("--movej-jerk", type=float, default=5.0)
    parser.add_argument("--left-home-tolerance", type=float, default=0.15)
    parser.add_argument("--first-frame-tolerance", type=float, default=0.08)
    parser.add_argument("--maximum-step", type=float, default=0.03)
    parser.add_argument("--maximum-velocity", type=float, default=0.8)
    parser.add_argument("--maximum-lateness", type=float, default=0.05)
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    args = parser.parse_args()
    positive = (
        "service_timeout",
        "movej_vel",
        "movej_acc",
        "movej_jerk",
        "left_home_tolerance",
        "first_frame_tolerance",
        "maximum_step",
        "maximum_velocity",
        "maximum_lateness",
    )
    for name in positive:
        if getattr(args, name) <= 0.0:
            parser.error("--{} must be positive".format(name.replace("_", "-")))
    if args.hold_seconds < 0.0:
        parser.error("--hold-seconds must be non-negative")
    return args


def output_fields():
    fields = [
        "source_file",
        "source_sha256",
        "frame_index",
        "source_timestamp",
        "target_x",
        "target_y",
        "target_z",
        "target_qw",
        "target_qx",
        "target_qy",
        "target_qz",
        "ik_success",
        "ik_status",
        "ik_latency_us",
        "arm_angle",
        "maximum_joint_step_rad",
        "maximum_joint_velocity_rad_s",
        "solver_start_since_playback_s",
        "publish_since_playback_s",
        "publish_lateness_s",
    ]
    fields.extend("q{}".format(index) for index in range(1, 8))
    return fields


def make_ik_request(target, previous_joints, previous_arm_angle):
    request = ArmIKRequest()
    request.method = "redundancy_selector"
    request.init_joints = list(previous_joints)
    request.current_arm_angle = float(previous_arm_angle)
    request.offset_list = []
    request.offset_refer = 0.0
    request.target_pose.position.x = float(target[0])
    request.target_pose.position.y = float(target[1])
    request.target_pose.position.z = float(target[2])
    request.target_pose.orientation.w = float(target[3])
    request.target_pose.orientation.x = float(target[4])
    request.target_pose.orientation.y = float(target[5])
    request.target_pose.orientation.z = float(target[6])
    return request


class OnlineRedundancySolver:
    def __init__(self, service, args):
        self.service = service
        self.maximum_step = args.maximum_step
        self.maximum_velocity = args.maximum_velocity
        self.previous_request_joints = rounded_solver_state(INITIAL_RIGHT_JOINTS)
        self.previous_output_joints = None
        self.previous_arm_angle = INITIAL_RIGHT_ARM_ANGLE
        self.previous_source_timestamp = None

    def solve(self, frame, target):
        request = make_ik_request(
            target, self.previous_request_joints, self.previous_arm_angle
        )
        started = time.monotonic()
        response = self.service.call(request)
        latency_us = (time.monotonic() - started) * 1.0e6
        status = str(response.message)
        if not response.success:
            raise RuntimeError(
                "online IK failed at frame {}: {}".format(frame.index, status)
            )
        if status != "redundancy_selector:selected":
            raise RuntimeError(
                "online IK returned non-selected status at frame {}: {}".format(
                    frame.index, status
                )
            )
        source_dt = None
        if self.previous_source_timestamp is not None:
            source_dt = frame.timestamp - self.previous_source_timestamp
        joints, transition = validate_online_solution(
            response.solution,
            self.previous_output_joints,
            source_dt,
            self.maximum_step,
            self.maximum_velocity,
        )
        arm_angle = float(response.new_arm_angle)
        self.previous_request_joints = rounded_solver_state(joints)
        self.previous_output_joints = joints
        self.previous_arm_angle = arm_angle
        self.previous_source_timestamp = frame.timestamp
        return {
            "joints": joints,
            "arm_angle": arm_angle,
            "status": status,
            "latency_us": latency_us,
            "maximum_step": transition.maximum_step,
            "maximum_velocity": transition.maximum_velocity,
        }


def result_row(
    input_path,
    source_summary,
    frame,
    target,
    result,
    solver_start=None,
    publish_time=None,
    publish_lateness=None,
):
    row = {
        "source_file": input_path,
        "source_sha256": source_summary.file_sha256,
        "frame_index": frame.index,
        "source_timestamp": frame.timestamp,
        "target_x": target[0],
        "target_y": target[1],
        "target_z": target[2],
        "target_qw": target[3],
        "target_qx": target[4],
        "target_qy": target[5],
        "target_qz": target[6],
        "ik_success": True,
        "ik_status": result["status"],
        "ik_latency_us": result["latency_us"],
        "arm_angle": result["arm_angle"],
        "maximum_joint_step_rad": result["maximum_step"],
        "maximum_joint_velocity_rad_s": result["maximum_velocity"],
        "solver_start_since_playback_s": solver_start,
        "publish_since_playback_s": publish_time,
        "publish_lateness_s": publish_lateness,
    }
    for index, value in enumerate(result["joints"], start=1):
        row["q{}".format(index)] = value
    return row


def open_output(args):
    output_path = os.path.abspath(args.output or timestamped_output(".csv"))
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if os.path.exists(output_path):
        raise FileExistsError("refusing to overwrite output: {}".format(output_path))
    output_file = open(output_path, "x", encoding="utf-8", newline="")
    writer = csv.DictWriter(output_file, fieldnames=output_fields())
    writer.writeheader()
    return output_path, output_file, writer


def wait_for_ik_service(args):
    rospy.wait_for_service(args.right_ik_service, timeout=args.service_timeout)
    return rospy.ServiceProxy(args.right_ik_service, ArmIK, persistent=True)


def run_ik_only(args, input_path, frames, source_summary, mapper):
    rospy.init_node("right_teleop_online_ik_check", anonymous=True)
    solver = OnlineRedundancySolver(wait_for_ik_service(args), args)
    output_path, output_file, writer = open_output(args)
    maximum_latency = 0.0
    try:
        for frame in frames:
            target = mapper.ik_target(frame.transform)
            result = solver.solve(frame, target)
            maximum_latency = max(maximum_latency, result["latency_us"])
            writer.writerow(
                result_row(input_path, source_summary, frame, target, result)
            )
            if (frame.index + 1) % 25 == 0:
                output_file.flush()
    finally:
        output_file.close()
    rospy.loginfo(
        "Online IK-only check solved %d frames; max latency %.3f us; output %s",
        len(frames),
        maximum_latency,
        output_path,
    )


def lower_services(args):
    names = (
        "/aris_node/feedback_srv",
        "/aris_node/movej_srv",
        "/aris_node/start_teleop_srv",
        "/aris_node/log_srv",
    )
    for name in names:
        rospy.wait_for_service(name, timeout=args.service_timeout)
    return (
        rospy.ServiceProxy(names[0], FeedbackService),
        rospy.ServiceProxy(names[1], MovejService),
        rospy.ServiceProxy(names[2], StartDualTeleOP),
        rospy.ServiceProxy(names[3], LogService),
    )


def run_execute(args, input_path, frames, source_summary, mapper):
    rospy.init_node("right_teleop_online_playback", anonymous=True)
    solver = OnlineRedundancySolver(wait_for_ik_service(args), args)
    output_path, output_file, writer = open_output(args)
    try:
        first_target = mapper.ik_target(frames[0].transform)
        first_result = solver.solve(frames[0], first_target)
        feedback_service, movej_service, teleop_service, log_service = lower_services(
            args
        )
        publisher = rospy.Publisher(
            "/arm_teleop/dual_arm_movej", DualArmMovej, queue_size=20
        )
        before = call_feedback(feedback_service)
    except Exception:
        output_file.close()
        raise
    left_home_error = maximum_error(before["left"], LEFT_HOME_JOINTS)
    if left_home_error > args.left_home_tolerance:
        output_file.close()
        raise RuntimeError(
            "left arm is {:.6f} rad away from the stop-service home pose; "
            "refusing right-only playback".format(left_home_error)
        )
    head_z_rotation = before["others"][4] / 0.8
    if not math.isfinite(head_z_rotation) or abs(head_z_rotation) > math.pi / 4.0 + 1e-6:
        output_file.close()
        raise RuntimeError("current auxiliary-axis target is outside the teleop clamp")
    audit = {
        "source_trajectory": source_summary.__dict__,
        "calculation_output": output_path,
        "started_at": datetime.now().isoformat(),
        "before_movej": before,
        "left_home_max_error_rad": left_home_error,
        "first_frame_from_start_max_delta_rad": maximum_error(
            before["right"], first_result["joints"]
        ),
    }
    teleop_started = False
    log_started = False
    completed = 0
    try:
        call_movej(movej_service, first_result["joints"], args)
        after_movej = call_feedback(feedback_service)
        audit["after_movej"] = after_movej
        first_error = maximum_error(
            after_movej["right"], first_result["joints"]
        )
        audit["first_frame_max_error_rad"] = first_error
        if first_error > args.first_frame_tolerance:
            raise RuntimeError(
                "right arm failed first-frame verification: {:.6f} rad > {:.6f} rad".format(
                    first_error, args.first_frame_tolerance
                )
            )
        connection_deadline = time.monotonic() + args.service_timeout
        while publisher.get_num_connections() < 1:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("lower controller did not subscribe to joint commands")
            time.sleep(0.05)

        set_log(log_service, "start_log")
        log_started = True
        set_teleop(teleop_service, True)
        teleop_started = True
        playback_start = time.monotonic()
        source_start = frames[0].timestamp
        publisher.publish(
            make_message(
                0, first_result["joints"], before["left"], head_z_rotation
            )
        )
        first_publish = time.monotonic()
        writer.writerow(
            result_row(
                input_path,
                source_summary,
                frames[0],
                first_target,
                first_result,
                0.0,
                first_publish - playback_start,
                first_publish - playback_start,
            )
        )
        output_file.flush()
        completed = 1

        for frame in frames[1:]:
            deadline = playback_start + frame.timestamp - source_start
            wait_until(deadline)
            solver_started = time.monotonic()
            target = mapper.ik_target(frame.transform)
            result = solver.solve(frame, target)
            publish_at = time.monotonic()
            lateness = publish_at - deadline
            if lateness > args.maximum_lateness:
                raise RuntimeError(
                    "frame {} is {:.6f} s late; refusing delayed command".format(
                        frame.index, lateness
                    )
                )
            publisher.publish(
                make_message(
                    frame.index,
                    result["joints"],
                    before["left"],
                    head_z_rotation,
                )
            )
            writer.writerow(
                result_row(
                    input_path,
                    source_summary,
                    frame,
                    target,
                    result,
                    solver_started - playback_start,
                    publish_at - playback_start,
                    lateness,
                )
            )
            completed += 1
            if completed % 25 == 0:
                output_file.flush()

        hold_deadline = time.monotonic() + args.hold_seconds
        sequence = len(frames)
        while time.monotonic() < hold_deadline:
            publisher.publish(
                make_message(
                    sequence,
                    solver.previous_output_joints,
                    before["left"],
                    head_z_rotation,
                )
            )
            sequence += 1
            time.sleep(1.0 / 30.0)
        audit["before_stop"] = call_feedback(feedback_service)
    finally:
        output_file.close()
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
        audit["completed_frames"] = completed
        audit["finished_at"] = datetime.now().isoformat()
        audit_path = write_audit(
            args.audit_output or timestamped_output("_audit.json"), audit
        )
        rospy.loginfo("Online playback audit written to %s", audit_path)


def main():
    args = parse_args()
    input_path = os.path.abspath(args.input)
    frames, source_summary = load_teleop_trajectory(input_path)
    mapper = RightArmTrajectoryMapper(frames[0].transform)
    print("Validated raw teleoperation trajectory: {}".format(input_path))
    print(
        "frames={}, duration={:.3f}s, dt=[{:.6f}, {:.6f}]s, sha256={}".format(
            source_summary.frame_count,
            source_summary.duration,
            source_summary.minimum_dt,
            source_summary.maximum_dt,
            source_summary.file_sha256,
        )
    )
    if args.preflight:
        print("Preflight only: no ROS service was called and no command was sent.")
    elif args.ik_only:
        run_ik_only(args, input_path, frames, source_summary, mapper)
    else:
        run_execute(args, input_path, frames, source_summary, mapper)


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logerr("Right-arm online teleoperation playback failed: %s", error)
        raise SystemExit(1)
