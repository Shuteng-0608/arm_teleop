#!/usr/bin/env python3

import argparse
import csv
import os
import time
from datetime import datetime

import rospy
from arm_angle.srv import PredictArmAngle, PredictArmAngleRequest
from arm_teleop.srv import ArmIK, ArmIKRequest

from core.right_arm_trajectory import (
    INITIAL_RIGHT_ARM_ANGLE,
    INITIAL_RIGHT_JOINTS,
    RightArmTrajectoryMapper,
    file_sha256,
    load_right_wrist_frames,
)


BASELINE_CURRENT_ARM_ANGLE = -0.5
BASELINE_COARSE_DEVIATIONS = [0.0, -0.1, -0.2, -0.3, -0.4, -0.5]
BASELINE_FINE_OFFSETS = [0.0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2]
BASELINE_MAX_JOINT_DEVIATION = 0.45


def package_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_input_path():
    return os.path.join(package_root(), "data_log", "circle_engineering.csv")


def default_output_path():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(
        package_root(),
        "data_log",
        "offline_right_ik_compare_{}.csv".format(timestamp),
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Compare the current right-arm IK path with the redundancy "
            "selector without publishing robot commands."
        )
    )
    parser.add_argument("--input", default=default_input_path())
    parser.add_argument("--output", default=None)
    parser.add_argument(
        "--baseline-angle-source",
        choices=("predictor", "previous"),
        default="predictor",
        help=(
            "Use the arm-angle predictor as main_ros does, or center the "
            "baseline search on its own previous valid arm angle."
        ),
    )
    parser.add_argument(
        "--right-ik-service", default="/arm_teleop/right_arm_ik_srv"
    )
    parser.add_argument("--predictor-service", default="/predict_arm_angle")
    parser.add_argument("--service-timeout", type=float, default=15.0)
    parser.add_argument(
        "--selector-warmup",
        type=int,
        default=1,
        help="Unrecorded first-frame selector calls used only to warm caches.",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="Zero processes the complete CSV; positive values are for smoke tests.",
    )
    args = parser.parse_args()
    if args.selector_warmup < 0:
        parser.error("--selector-warmup must be non-negative")
    if args.max_frames < 0:
        parser.error("--max-frames must be non-negative")
    if args.service_timeout <= 0.0:
        parser.error("--service-timeout must be positive")
    return args


def make_ik_request(method, target_pose, previous_joints, arm_angle, deviations):
    request = ArmIKRequest()
    request.method = method
    request.init_joints = list(previous_joints)
    request.current_arm_angle = float(arm_angle)
    request.offset_list = list(deviations)
    request.offset_refer = BASELINE_MAX_JOINT_DEVIATION
    request.target_pose.position.x = float(target_pose[0])
    request.target_pose.position.y = float(target_pose[1])
    request.target_pose.position.z = float(target_pose[2])
    request.target_pose.orientation.w = float(target_pose[3])
    request.target_pose.orientation.x = float(target_pose[4])
    request.target_pose.orientation.y = float(target_pose[5])
    request.target_pose.orientation.z = float(target_pose[6])
    return request


def predictor_request(target_pose):
    request = PredictArmAngleRequest()
    request.arm_side = "right"
    request.pose = list(target_pose)
    return request


def call_with_latency(service, request):
    started = time.perf_counter()
    response = service.call(request)
    latency_us = (time.perf_counter() - started) * 1.0e6
    return response, latency_us


def rounded_state(joints):
    return [round(float(value), 4) for value in joints]


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
        "baseline_angle_source",
        "predictor_success",
        "predictor_arm_angle_rad",
        "predictor_latency_us",
        "baseline_success",
        "baseline_held_previous",
        "baseline_status",
        "baseline_search_count",
        "baseline_latency_us",
        "baseline_arm_angle",
    ]
    fields.extend("baseline_q{}".format(index) for index in range(1, 8))
    fields.extend(
        [
            "redundancy_success",
            "redundancy_held_previous",
            "redundancy_status",
            "redundancy_latency_us",
            "redundancy_arm_angle",
        ]
    )
    fields.extend("redundancy_q{}".format(index) for index in range(1, 8))
    return fields


def wait_for_services(args):
    rospy.wait_for_service(args.right_ik_service, timeout=args.service_timeout)
    right_ik = rospy.ServiceProxy(args.right_ik_service, ArmIK, persistent=True)
    predictor = None
    if args.baseline_angle_source == "predictor":
        rospy.wait_for_service(args.predictor_service, timeout=args.service_timeout)
        predictor = rospy.ServiceProxy(
            args.predictor_service, PredictArmAngle, persistent=True
        )
    return right_ik, predictor


def baseline_deviations(args, baseline_arm_angle, predictor, predictor_pose):
    predictor_success = False
    predictor_angle = float("nan")
    predictor_latency_us = float("nan")
    if args.baseline_angle_source == "predictor":
        response, predictor_latency_us = call_with_latency(
            predictor, predictor_request(predictor_pose)
        )
        predictor_success = bool(response.success)
        predictor_angle = float(response.arm_angle_rad)
        # This reproduces main_ros exactly: current_arm_angle_right is
        # -prediction + 0.5 and the IK request's hot-path angle is -0.5.
        fine_deviation_center = -predictor_angle + 0.5
    else:
        # computeIKWithSearch adds each deviation to its hot-path angle.
        fine_deviation_center = baseline_arm_angle - BASELINE_CURRENT_ARM_ANGLE

    deviations = BASELINE_COARSE_DEVIATIONS + [
        fine_deviation_center + offset for offset in BASELINE_FINE_OFFSETS
    ]
    return (
        deviations,
        predictor_success,
        predictor_angle,
        predictor_latency_us,
    )


def run(args):
    input_path = os.path.abspath(args.input)
    output_path = os.path.abspath(args.output or default_output_path())
    frames = load_right_wrist_frames(input_path)
    if args.max_frames:
        frames = frames[: args.max_frames]
    mapper = RightArmTrajectoryMapper(frames[0].transform)
    source_hash = file_sha256(input_path)

    output_directory = os.path.dirname(output_path)
    os.makedirs(output_directory, exist_ok=True)
    if os.path.exists(output_path):
        raise FileExistsError("refusing to overwrite output: {}".format(output_path))

    rospy.init_node("offline_right_ik_compare", anonymous=True)
    right_ik, predictor = wait_for_services(args)

    baseline_joints = rounded_state(INITIAL_RIGHT_JOINTS)
    redundancy_joints = rounded_state(INITIAL_RIGHT_JOINTS)
    baseline_arm_angle = INITIAL_RIGHT_ARM_ANGLE
    redundancy_arm_angle = INITIAL_RIGHT_ARM_ANGLE

    first_target = mapper.ik_target(frames[0].transform)
    for _ in range(args.selector_warmup):
        warmup_request = make_ik_request(
            "redundancy_selector",
            first_target,
            redundancy_joints,
            redundancy_arm_angle,
            [],
        )
        right_ik.call(warmup_request)

    completed = 0
    with open(output_path, "x", encoding="utf-8", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=output_fields())
        writer.writeheader()

        for frame in frames:
            if rospy.is_shutdown():
                break

            target = mapper.ik_target(frame.transform)
            predictor_pose = mapper.predictor_target(frame.transform)
            (
                deviations,
                predictor_success,
                predicted_angle,
                predictor_latency_us,
            ) = baseline_deviations(
                args,
                baseline_arm_angle,
                predictor,
                predictor_pose,
            )

            baseline_request = make_ik_request(
                "feasible_ref",
                target,
                baseline_joints,
                BASELINE_CURRENT_ARM_ANGLE,
                deviations,
            )
            baseline_response, baseline_latency_us = call_with_latency(
                right_ik, baseline_request
            )
            baseline_success = bool(baseline_response.success)
            baseline_held = not baseline_success
            if baseline_success:
                baseline_output_joints = [
                    float(value) for value in baseline_response.solution
                ]
                baseline_joints = rounded_state(baseline_output_joints)
                baseline_arm_angle = float(baseline_response.new_arm_angle)
            else:
                baseline_output_joints = list(baseline_joints)

            redundancy_request = make_ik_request(
                "redundancy_selector",
                target,
                redundancy_joints,
                redundancy_arm_angle,
                [],
            )
            redundancy_response, redundancy_latency_us = call_with_latency(
                right_ik, redundancy_request
            )
            redundancy_success = bool(redundancy_response.success)
            redundancy_status = str(redundancy_response.message)
            redundancy_held = (
                not redundancy_success or "hold_previous" in redundancy_status
            )
            if redundancy_success:
                redundancy_output_joints = [
                    float(value) for value in redundancy_response.solution
                ]
                redundancy_joints = rounded_state(redundancy_output_joints)
                redundancy_arm_angle = float(
                    redundancy_response.new_arm_angle
                )
            else:
                redundancy_output_joints = list(redundancy_joints)

            row = {
                "source_file": input_path,
                "source_sha256": source_hash,
                "frame_index": frame.index,
                "source_timestamp": frame.timestamp,
                "target_x": target[0],
                "target_y": target[1],
                "target_z": target[2],
                "target_qw": target[3],
                "target_qx": target[4],
                "target_qy": target[5],
                "target_qz": target[6],
                "baseline_angle_source": args.baseline_angle_source,
                "predictor_success": predictor_success,
                "predictor_arm_angle_rad": predicted_angle,
                "predictor_latency_us": predictor_latency_us,
                "baseline_success": baseline_success,
                "baseline_held_previous": baseline_held,
                "baseline_status": str(baseline_response.message),
                "baseline_search_count": baseline_response.search_cnt,
                "baseline_latency_us": baseline_latency_us,
                "baseline_arm_angle": baseline_arm_angle,
                "redundancy_success": redundancy_success,
                "redundancy_held_previous": redundancy_held,
                "redundancy_status": redundancy_status,
                "redundancy_latency_us": redundancy_latency_us,
                "redundancy_arm_angle": redundancy_arm_angle,
            }
            for index, value in enumerate(baseline_output_joints, start=1):
                row["baseline_q{}".format(index)] = value
            for index, value in enumerate(redundancy_output_joints, start=1):
                row["redundancy_q{}".format(index)] = value
            writer.writerow(row)
            completed += 1
            if completed % 25 == 0:
                output_file.flush()

    rospy.loginfo(
        "Offline right-arm IK comparison wrote %d/%d frames to %s",
        completed,
        len(frames),
        output_path,
    )
    return 0 if completed == len(frames) else 2


def main():
    args = parse_args()
    try:
        return run(args)
    except (OSError, ValueError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logerr("Offline right-arm IK comparison failed: %s", error)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
