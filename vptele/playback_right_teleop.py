#!/usr/bin/env python3

import argparse
import csv
import os
import time
from datetime import datetime

import rospy
from arm_teleop.msg import DualArmMovej
from arm_teleop.srv import ArmIK, ArmIKRequest
from arm_teleop.srv import (
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
    call_movej,
    make_message,
    set_teleop,
    wait_until,
    write_audit,
)


def package_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_input_path():
    configured = os.environ.get("ARM_TELEOP_TRAJECTORY_CSV")
    if configured:
        return os.path.abspath(configured)

    relative_r50 = os.path.join(
        package_root(),
        "..",
        "Arm_kinematics_cal_cpp",
        "examples",
        "redundancy_selector",
        "r50_trigger_demo",
        "R50_trigger_demo.csv",
    )
    candidates = (
        os.path.join(package_root(), "data_log", "R50_trigger_demo.csv"),
        relative_r50,
        "/home/pangu/pangu/src/Arm_kinematics_cal_cpp/examples/"
        "redundancy_selector/r50_trigger_demo/R50_trigger_demo.csv",
    )
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    # Keep the error at the CSV loader, where the missing path is actionable.
    return candidates[0]


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
    parser.add_argument(
        "--ik-method",
        choices=("A1_minimum_jv", "minimum_sufficient_continuity_refined"),
        default="A1_minimum_jv",
        help="Right-arm selector profile requested from the IK service.",
    )
    parser.add_argument("--output", default=None)
    parser.add_argument("--audit-output", default=None)
    parser.add_argument(
        "--right-ik-service", default="/arm_teleop/right_arm_ik_srv"
    )
    parser.add_argument("--service-timeout", type=float, default=15.0)
    parser.add_argument("--movej-vel", type=float, default=0.5)
    parser.add_argument("--movej-acc", type=float, default=5.0)
    parser.add_argument("--movej-jerk", type=float, default=10.0)
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


def make_ik_request(
    target, previous_joints, previous_arm_angle, method="A1_minimum_jv"
):
    request = ArmIKRequest()
    request.method = method
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
    def __init__(self, service, args, initial_joints=None, initial_arm_angle=None):
        self.service = service
        self.ik_method = args.ik_method
        self.maximum_step = args.maximum_step
        self.maximum_velocity = args.maximum_velocity
        self.initial_joints = tuple(
            float(value)
            for value in (initial_joints if initial_joints is not None else INITIAL_RIGHT_JOINTS)
        )
        self.initial_arm_angle = float(
            initial_arm_angle
            if initial_arm_angle is not None
            else INITIAL_RIGHT_ARM_ANGLE
        )
        self.previous_request_joints = rounded_solver_state(self.initial_joints)
        self.previous_output_joints = None
        self.previous_arm_angle = self.initial_arm_angle
        self.previous_source_timestamp = None

    def solve(self, frame, target):
        request = make_ik_request(
            target,
            self.previous_request_joints,
            self.previous_arm_angle,
            self.ik_method,
        )
        started = time.monotonic()
        response = self.service.call(request)
        latency_us = (time.monotonic() - started) * 1.0e6
        status = str(response.message)
        if not response.success:
            raise RuntimeError(
                "online IK failed at frame {}: {}".format(frame.index, status)
            )
        accepted_statuses = {
            "{}:selected".format(self.ik_method),
            "{}:fallback_baseline".format(self.ik_method),
            "{}:hold_previous".format(self.ik_method),
        }
        if status not in accepted_statuses:
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
            "method": self.ik_method,
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


def target_for_frame(frame, mapper):
    if frame.target_pose is not None:
        return frame.target_pose
    if mapper is None or frame.transform is None:
        raise ValueError("trajectory frame has neither a target pose nor a wrist transform")
    return mapper.ik_target(frame.transform)


def run_ik_only(args, input_path, frames, source_summary, mapper):
    rospy.init_node("right_teleop_online_ik_check", anonymous=True)
    solver = OnlineRedundancySolver(
        wait_for_ik_service(args),
        args,
        frames[0].initial_joints,
        frames[0].initial_arm_angle,
    )
    output_path, output_file, writer = open_output(args)
    maximum_latency = 0.0
    try:
        for frame in frames:
            target = target_for_frame(frame, mapper)
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
        "/aris_node/movej_srv",
        "/aris_node/start_teleop_srv",
    )
    for name in names:
        rospy.wait_for_service(name, timeout=args.service_timeout)
    return (
        rospy.ServiceProxy(names[0], MovejService),
        rospy.ServiceProxy(names[1], StartDualTeleOP),
    )


def run_execute(args, input_path, frames, source_summary, mapper):
    rospy.init_node("right_teleop_online_playback", anonymous=True)
    solver = OnlineRedundancySolver(
        wait_for_ik_service(args),
        args,
        frames[0].initial_joints,
        frames[0].initial_arm_angle,
    )
    output_path, output_file, writer = open_output(args)
    publisher = rospy.Publisher(
        "/arm_teleop/dual_arm_movej", DualArmMovej, queue_size=100
    )
    try:
        movej_service, teleop_service = lower_services(args)
    except Exception:
        output_file.close()
        raise
    head_z_rotation = 0.0
    left_hold_joints = tuple(LEFT_HOME_JOINTS)
    audit = {
        "source_trajectory": source_summary.__dict__,
        "calculation_output": output_path,
        "started_at": datetime.now().isoformat(),
        "right_initial_joints": list(solver.initial_joints),
        "left_hold_joints": left_hold_joints,
        "settings": {
            "ik_method": solver.ik_method,
            "movej_vel": args.movej_vel,
            "movej_acc": args.movej_acc,
            "movej_jerk": args.movej_jerk,
            "head_z_rotation": head_z_rotation,
        },
    }
    teleop_started = False
    completed = 0
    try:
        # Preserve main_ros initialization order; only the right-arm IK method differs.
        call_movej(movej_service, solver.initial_joints, args, arm_id=1)
        call_movej(movej_service, LEFT_HOME_JOINTS, args, arm_id=0)
        set_teleop(teleop_service, True)
        teleop_started = True
        playback_start = time.monotonic()
        source_start = frames[0].timestamp

        for frame in frames:
            deadline = playback_start + frame.timestamp - source_start
            wait_until(deadline)
            solver_started = time.monotonic()
            target = target_for_frame(frame, mapper)
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
                    left_hold_joints,
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
                    left_hold_joints,
                    head_z_rotation,
                )
            )
            sequence += 1
            time.sleep(1.0 / 30.0)
    finally:
        output_file.close()
        audit["teleop_started"] = teleop_started
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
    mapper = (
        RightArmTrajectoryMapper(frames[0].transform)
        if frames[0].transform is not None
        else None
    )
    trajectory_kind = "target pose" if frames[0].target_pose is not None else "raw wrist"
    print("Validated {} teleoperation trajectory: {}".format(trajectory_kind, input_path))
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
