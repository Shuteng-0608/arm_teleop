#! /usr/bin/env python
import rospy
import rosbag
from arm_teleop.msg import DualArmMovej
from arm_teleop.srv import MovejService, MovejServiceRequest
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
import time
import argparse

def record_arm_movement(bag_path, duration=None):
    """
    录制双臂运动数据
    """
    rospy.init_node("arm_recorder")
    
    bag = rosbag.Bag(bag_path, 'w')
    msg_count = 0
    
    rospy.loginfo("开始录制 /arm_teleop/dual_arm_movej")
    rospy.loginfo("保存到: %s", bag_path)
    
    def callback(msg):
        nonlocal msg_count
        bag.write("/arm_teleop/dual_arm_movej", msg, rospy.Time.now())
        msg_count += 1
        rospy.loginfo("录制消息 %d", msg_count)
    
    rospy.Subscriber("/arm_teleop/dual_arm_movej", DualArmMovej, callback, queue_size=100)
    
    try:
        if duration:
            rospy.sleep(duration)
        else:
            rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("录制中断")
    finally:
        bag.close()
        rospy.loginfo("录制完成，共 %d 条消息", msg_count)

def play_arm_movement(bag_path, rate=1.0):
    """
    播放双臂运动数据
    """
    rospy.init_node("arm_player")
    
    bag = rosbag.Bag(bag_path)
    # ================== Single Arm MoveJ SERVICE ==================
    rospy.wait_for_service('/aris_node/movej_srv')
    pq_movej_service = rospy.ServiceProxy('/aris_node/movej_srv', MovejService)
    pq_request = MovejServiceRequest()
    pq_request.arm_id = 1
    pq_request.target_joints = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]
    pq_request.vel = 0.5
    pq_request.acc = 5.0
    pq_request.jerk = 10.0
    pq_response = pq_movej_service.call(pq_request)


    pq_request = MovejServiceRequest()
    pq_request.arm_id = 0
    pq_request.target_joints = [-0.0433303, 0.141567, 0.0831955, 1.59424, -1.37614, -0.115441, -0.00507801]
    pq_request.vel = 0.5
    pq_request.acc = 5.0
    pq_request.jerk = 10.0
    pq_response = pq_movej_service.call(pq_request)

    # # ============== Start Dual TeleOP Service ==============
    rospy.wait_for_service('/aris_node/start_teleop_srv')
    start_teleop_service = rospy.ServiceProxy('/aris_node/start_teleop_srv', StartDualTeleOP)
    tele_req = StartDualTeleOPRequest()
    tele_req.running_flag = True
    tele_response = start_teleop_service.call(tele_req)
    
    
    # 创建发布者
    publisher = rospy.Publisher('/arm_teleop/dual_arm_movej', DualArmMovej, queue_size=100)
    
    rospy.loginfo("开始播放: %s", bag_path)
    rospy.loginfo("播放速率: %.1fx", rate)
    
    msg_count = 0
    
    for topic, msg, timestamp in bag.read_messages():
        publisher.publish(msg)
        msg_count += 1
        rospy.loginfo("播放消息 %d", msg_count)
        rospy.sleep(1.0 / (rate * 10))  # 控制播放速率
    
    bag.close()
    rospy.loginfo("播放完成，共 %d 条消息", msg_count)
    rospy.sleep(1)  # 确保所有消息发布完成 before stopping teleop
    tele_req = StartDualTeleOPRequest()
    tele_req.running_flag = False
    tele_response = start_teleop_service.call(tele_req)



    # pq_request = MovejServiceRequest()
    # pq_request.arm_id = 1
    # pq_request.target_joints = [5, -5, -3, 0.13, -0.12, 0.4, 1.0]
    # pq_request.vel = 0.5
    # pq_request.acc = 5.0
    # pq_request.jerk = 20.0
    # pq_response = pq_movej_service.call(pq_request)


    # pq_request = MovejServiceRequest()
    # pq_request.arm_id = 0
    # pq_request.target_joints = [5, -5, -3, -0.13, 0.12, -0.4, -1.0]
    # pq_request.vel = 0.5
    # pq_request.acc = 5.0
    # pq_request.jerk = 20.0
    # pq_response = pq_movej_service.call(pq_request)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='录制/播放 DualArmMovej 数据')
    parser.add_argument('mode', choices=['record', 'play'], help='模式: record 或 play')
    parser.add_argument('bag_path', help='bag文件路径')
    parser.add_argument('--duration', type=float, help='录制时长(秒)')
    parser.add_argument('--rate', type=float, default=1.0, help='播放速率')
    
    args = parser.parse_args()
    
    if args.mode == 'record':
        record_arm_movement(args.bag_path, args.duration)
    else:
        play_arm_movement(args.bag_path, args.rate)