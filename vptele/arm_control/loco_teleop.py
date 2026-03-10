#!/usr/bin/env python3
"""
loco_teleop.py

Test node: map synthetic/head 2D (x,y) motion to robot base movement
Uses the same action interface as `ArmTeleopROS.woosh_mf` / `woosh_rotate`.

Run as a ROS node for manual testing.
"""
import rospy
import actionlib
import time
import math
import numpy as np
from woosh_msgs.msg import StepControlGoal, StepControl, StepControlAction
from scipy.spatial.transform import Rotation as R

# Optional: VPStreamer to get real head data from AVP
try:
    from core.vp_streamer_avp import VPStreamer
except Exception:
    VPStreamer = None


def woosh_mf(client, distance, speed=0.1, use_avoid=True, timeout=5.0):
    goal = StepControlGoal()
    goal.mode = StepControlGoal.EXCUTE
    goal.useAvoid = use_avoid

    step = StepControl()
    step.executeMode = StepControlGoal.STRAIGHT
    step.data = float(distance)
    step.speed = float(speed)
    goal.stepControl = [step]

    client.send_goal(goal)
    ok = client.wait_for_result(rospy.Duration(timeout))
    return ok


def woosh_rotate(client, angle_deg, mode='normal', speed=2, timeout=5.0):
    goal = StepControlGoal()
    goal.mode = StepControlGoal.EXCUTE
    goal.useAvoid = False

    step = StepControl()
    step.executeMode = StepControlGoal.ROTATE
    if mode == 'wake_up':
        step.data = -(angle_deg - 90.0) / 180.0 * math.pi
    else:
        step.data = angle_deg / 180.0 * math.pi
    step.speed = float(speed)
    goal.stepControl = [step]

    client.send_goal(goal)
    ok = client.wait_for_result(rospy.Duration(timeout))
    return ok


def run_test():
    rospy.init_node('loco_teleop_test', anonymous=True)

    # Params (can be set via rosparam)
    send_threshold = rospy.get_param('~send_threshold', 0.05)  # meters
    send_scale = rospy.get_param('~send_scale', 1.0)
    max_step = rospy.get_param('~max_step', 0.2)
    send_rate = rospy.get_param('~send_rate', 2.0)  # Hz
    rotate_scale_deg = rospy.get_param('~rotate_scale_deg', 1.0)  # degrees per meter of x movement (example)
    timeout = rospy.get_param('~woosh_timeout', 5.0)

    client = actionlib.SimpleActionClient('/cmd_vel_control', StepControlAction)
    rospy.loginfo('等待 /cmd_vel_control action server...')
    # store the initial head orientation for relative rotation calculation
    initial_rotation = None
    # last sent orientation around Z (degrees), used to ignore tiny changes
    last_sent_z_deg = 0.0
    if not client.wait_for_server(rospy.Duration(10.0)):
        rospy.logerr('无法连接到 /cmd_vel_control action server')
        return
    rospy.loginfo('已连接到 /cmd_vel_control')

    use_avp = rospy.get_param('~use_avp', True)
    avp_ip = rospy.get_param('~avp_ip', '192.168.8.145')

    rate = rospy.Rate(send_rate)
    prev_pos = None

    streamer = None
    if use_avp:
        if VPStreamer is None:
            rospy.logerr('VPStreamer not available (core.vp_streamer_avp import failed)')
            return
        try:
            streamer = VPStreamer(avp_ip, record=False)
            rospy.loginfo(f'Connected to AVP streamer at {avp_ip}')
            
            streamer.streamer.configure_video(device="/dev/video6", format="v4l2", size="1280x720", fps=60)
            streamer.streamer.start_webrtc()
        except Exception as e:
            rospy.logerr(f'Failed to init VPStreamer: {e}')
            return

    rospy.loginfo('开始发送测试（按 Ctrl-C 退出）')
    if not use_avp:
        rospy.logerr("use_avp must be True when running loco_teleop with real data")
        return

    # main loop using only real AVP head data
    while not rospy.is_shutdown():
        try:
            head_data = streamer.get_head_data()
            if head_data is None or len(head_data) == 0:
                rate.sleep()
                continue
            current_transform = head_data[0]
            current_position = np.array(current_transform[:3, 3], dtype=float)
            pos2d = np.array([current_position[0], current_position[1]])

            if prev_pos is None:
                prev_pos = pos2d
                rate.sleep()
                continue

            delta = pos2d - prev_pos
            # dx = float(delta[0])
            dy = float(delta[1])

            # forward/backward mapped from y displacement
            if abs(dy) >= send_threshold:
                distance = max(-max_step, min(max_step, dy * send_scale))
                rospy.loginfo(f"sending forward step: {distance:.4f} m (dy={dy:.4f})")
                woosh_mf(client, distance, speed=0.1, use_avoid=True, timeout=timeout)

            # left/right mapped to rotation: use current orientation about Z
            # calculate relative rotation with respect to the first frame
            current_rotation = current_transform[:3, :3]
            if initial_rotation is None:
                initial_rotation = current_rotation.copy()
            rel_rot = current_rotation @ np.linalg.inv(initial_rotation)
            z_rad = R.from_matrix(rel_rot).as_euler('xyz', degrees=False)[2]
            z_deg = math.degrees(z_rad)
            angle_deg = z_deg  # scale if needed
            # clamp to maximum step if provided
            max_rotate = rospy.get_param('~max_rotate_deg', 45.0)  # default max rotation is 45 degrees
            if max_rotate is not None:
                angle_deg = max(-max_rotate, min(max_rotate, angle_deg))
            # ignore tiny orientation changes relative to last sent rotation
            min_rotate_deg = rospy.get_param('~min_rotate_deg', 1.0)  # minimum degrees change to trigger rotate
            should_send = True
            # if abs(z_deg) >= min_rotate_deg:
            #     if last_sent_z_deg is None or abs(z_deg - last_sent_z_deg) >= min_rotate_deg:
            #         should_send = True
            if should_send:
                rospy.loginfo(f"sending rotate (z orientation): {angle_deg:.2f} deg (z_deg={z_deg:.4f})")
                woosh_rotate(client, z_deg - last_sent_z_deg, mode='normal', speed=2, timeout=timeout)
                last_sent_z_deg = z_deg

            prev_pos = pos2d
            rate.sleep()

        except Exception as e:
            rospy.logerr(f"run_test loop error: {e}")
            time.sleep(0.1)


if __name__ == '__main__':
    try:
        run_test()
    except rospy.ROSInterruptException:
        pass
