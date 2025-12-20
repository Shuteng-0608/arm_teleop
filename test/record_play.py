#! /usr/bin/env python
import rospy
import rosbag
from arm_teleop.msg import DualArmMovej, DualHandTele
from arm_teleop.srv import MovejService, MovejServiceRequest
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
import time
import argparse
import threading
import queue

def record_arm_movement(bag_path, duration=None):
    """
    录制双臂运动数据和双手遥操作数据
    """
    rospy.init_node("arm_recorder")

    bag = rosbag.Bag(bag_path, 'w')
    arm_msg_count = 0
    hand_msg_count = 0

    arm_queue = queue.Queue(maxsize=2000)
    hand_queue = queue.Queue(maxsize=2000)
    stop_event = threading.Event()

    rospy.loginfo("开始录制 /arm_teleop/dual_arm_movej 与 /arm_teleop/dual_hand_tele")
    rospy.loginfo("保存到: %s", bag_path)

    def pick_stamp(msg):
        # Prefer message's own timestamp if present, fallback to now
        return getattr(getattr(msg, 'header', None), 'stamp', rospy.Time.now())

    def writer_worker():
        nonlocal arm_msg_count, hand_msg_count
        last_log_time = rospy.Time.now()
        while not stop_event.is_set() or not arm_queue.empty() or not hand_queue.empty():
            try:
                topic, msg = arm_queue.get_nowait()
                bag.write(topic, msg, pick_stamp(msg))
                arm_msg_count += 1
                arm_queue.task_done()
            except queue.Empty:
                pass
            try:
                topic, msg = hand_queue.get_nowait()
                bag.write(topic, msg, pick_stamp(msg))
                hand_msg_count += 1
                hand_queue.task_done()
            except queue.Empty:
                pass

            # Throttle logs to ~1 Hz
            now = rospy.Time.now()
            if (now - last_log_time).to_sec() >= 1.0:
                rospy.loginfo("录制进度 双臂:%d 双手:%d", arm_msg_count, hand_msg_count)
                last_log_time = now

            rospy.sleep(0.001)

    writer_thread = threading.Thread(target=writer_worker, daemon=True)
    writer_thread.start()

    def arm_callback(msg):
        try:
            arm_queue.put_nowait(("/arm_teleop/dual_arm_movej", msg))
        except queue.Full:
            rospy.logwarn("arm_queue 满，丢弃一条双臂消息")

    def hand_callback(msg):
        try:
            hand_queue.put_nowait(("/arm_teleop/dual_hand_tele", msg))
        except queue.Full:
            rospy.logwarn("hand_queue 满，丢弃一条双手消息")

    rospy.Subscriber("/arm_teleop/dual_arm_movej", DualArmMovej, arm_callback, queue_size=100)
    rospy.Subscriber("/arm_teleop/dual_hand_tele", DualHandTele, hand_callback, queue_size=100)

    try:
        if duration:
            rospy.sleep(duration)
        else:
            rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("录制中断")
    finally:
        stop_event.set()
        writer_thread.join()
        bag.close()
        rospy.loginfo("录制完成，双臂 %d 条，双手 %d 条", arm_msg_count, hand_msg_count)

def play_arm_movement(bag_path, rate=1.0):
    """
    播放双臂运动数据和双手遥操作数据
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
    arm_publisher = rospy.Publisher('/arm_teleop/dual_arm_movej', DualArmMovej, queue_size=100)
    hand_publisher = rospy.Publisher('/arm_teleop/dual_hand_tele', DualHandTele, queue_size=100)

    rospy.loginfo("开始播放: %s", bag_path)
    rospy.loginfo("播放速率: %.1fx", rate)

    arm_msg_count = 0
    hand_msg_count = 0

    arm_queue = queue.Queue()
    hand_queue = queue.Queue()
    rate_obj = rospy.Rate(rate * 10)

    def publisher_worker(msg_queue, publisher, is_arm=True):
        nonlocal arm_msg_count, hand_msg_count
        while not rospy.is_shutdown():
            msg = msg_queue.get()
            if msg is None:
                msg_queue.task_done()
                break
            publisher.publish(msg)
            if is_arm:
                arm_msg_count += 1
                rospy.loginfo("播放双臂消息 %d", arm_msg_count)
            else:
                hand_msg_count += 1
                rospy.loginfo("播放双手消息 %d", hand_msg_count)
            rate_obj.sleep()
            msg_queue.task_done()

    arm_thread = threading.Thread(target=publisher_worker, args=(arm_queue, arm_publisher, True), daemon=True)
    hand_thread = threading.Thread(target=publisher_worker, args=(hand_queue, hand_publisher, False), daemon=True)
    arm_thread.start()
    hand_thread.start()

    start_time = rospy.Time.from_sec(10.0)
    for topic, msg, timestamp in bag.read_messages(start_time=start_time):
        if topic == '/arm_teleop/dual_arm_movej':
            arm_queue.put(msg)
        elif topic == '/arm_teleop/dual_hand_tele':
            hand_queue.put(msg)
        else:
            rospy.logdebug("忽略无关话题: %s", topic)

    # 发送结束标记并等待线程完成
    arm_queue.put(None)
    hand_queue.put(None)
    arm_queue.join()
    hand_queue.join()

    bag.close()
    rospy.loginfo("播放完成，双臂 %d 条，双手 %d 条", arm_msg_count, hand_msg_count)
    rospy.sleep(1)  # 确保所有消息发布完成 before stopping teleop
    tele_req = StartDualTeleOPRequest()
    tele_req.running_flag = False
    tele_response = start_teleop_service.call(tele_req)

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