import os

import rospy
from arm_teleop.srv import ArmIK, ArmIKRequest
from arm_teleop.srv import MovejService, MovejServiceRequest
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
# from pangu_msgs.msg import ArmJoints, DualArmMovej
from arm_teleop.msg import ArmJoints, DualArmMovej
from arm_angle.msg import ArmAngle
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import numpy as np
from threading import Thread
from utils.math_utils import rotation_matrix_to_euler, smooth_values
from utils.logger import get_logger
from utils.filters import OneEuroFilter, PoseFilter7D
from scipy.spatial.transform import Rotation as R
import threading
import csv
from datetime import datetime
from std_srvs.srv import SetBool, SetBoolResponse

from arm_angle.srv import PredictArmAngle, PredictArmAngleRequest

logger = get_logger()

class ArmTeleopROS:
    def __init__(self, vp_streamer, robot_controller, config=None):
        """
        初始化机械臂遥控器
        
        参数:
            vp_streamer: VisionPro数据流对象
            robot_controller: 机械臂控制器对象
            config (dict): 配置参数
        """
        # 基本组件
        self.vp_streamer = vp_streamer
        self.robot_controller = robot_controller
        self.config = config or {}

        if not rospy.core.is_initialized():
            rospy.init_node('arm_teleop', anonymous=True)



        # Arm Angle Prediction Service
        self.arm_angle_subscriber = rospy.ServiceProxy('/predict_arm_angle', PredictArmAngle)
        
        # ================= DUAL ARM DATA PUBLISHER ==================
        self.dual_arm_publisher = rospy.Publisher('/arm_teleop/dual_arm_movej', DualArmMovej, queue_size=100)
        self.publish_rate = rospy.Rate(100)
        self.publisher_thread = None

        self.data_lock = threading.RLock()
        self.sequence = 0
        
        
        # ================== Single Arm MoveJ SERVICE ==================
        # rospy.wait_for_service('/aris_node/movej_srv')
        self.pq_movej_service = rospy.ServiceProxy('/aris_node/movej_srv', MovejService)



        # self.current_arm_angle_right = 0.0
        # self.arm_angle_subscriber = rospy.Subscriber('/arm_angle/info', ArmAngle, self.arm_angle_callback, queue_size=10)
        


        # ================== [RIGHT ARM INIT] ==================
        # rospy.wait_for_service('/arm_teleop/right_arm_ik_srv')
        self.right_ik_service = rospy.ServiceProxy('/arm_teleop/right_arm_ik_srv', ArmIK)
        self.initial_right_robot_pose = [0.3011, -0.3580, 0.2282, 3.1923149, -0.036102, -0.0007987]  # XYZ + 欧拉角 (弧度)
        
        # self.initial_right_robot_pose_aa = [0.3011, -0.3580, 0.2282, 3.1923149, -0.036102, -0.0007987]  # XYZ + 欧拉角 (弧度)
        self.initial_right_robot_pose_aa = [0.26164, -0.35714, 0.06982, 3.1923149, -0.036102, -0.0007987]  # XYZ + 欧拉角 (弧度)
        self.init_right_rotation = R.from_euler("XYZ", 
                                        [self.initial_right_robot_pose[3], 
                                         self.initial_right_robot_pose[4], 
                                         self.initial_right_robot_pose[5]], 
                                        degrees=False)
        self.init_right_rotation_aa = R.from_euler("XYZ", 
                                        [self.initial_right_robot_pose_aa[3], 
                                         self.initial_right_robot_pose_aa[4], 
                                         self.initial_right_robot_pose_aa[5]], 
                                        degrees=False)
        self.last_right_joint_angles = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]
        self.last_right_joint_angles = [round(angle, 4) for angle in self.last_right_joint_angles]
        pq_request = MovejServiceRequest()
        pq_request.arm_id = 1
        pq_request.target_joints = self.last_right_joint_angles
        pq_request.vel = 0.5
        pq_request.acc = 5.0
        pq_request.jerk = 10.0
        pq_response = self.pq_movej_service.call(pq_request)
        self.initial_right_robot_pose_in_quat = self.euler_to_quaternion(self.initial_right_robot_pose)
        self.initial_right_robot_pose_in_quat = [round(angle, 4) for angle in self.initial_right_robot_pose_in_quat]
        self.current_arm_angle_right = -1.2 + 0.5
        # self.current_arm_angle_right = 0
        # self.current_arm_angle_right = 145 * np.pi / 180.0  # 初始臂角，单位为弧度
        logger.info("RIGHT ARM 已连接到逆运动学服务, 初始化完成。")
        rospy.loginfo(f"[RIGHT ARM]机械臂末端初始位置: {[round(x, 4) for x in self.initial_right_robot_pose]}")
        


        # ================== [LEFT ARM INIT] ===================
        # rospy.wait_for_service('/arm_teleop/left_arm_ik_srv')
        self.left_ik_service = rospy.ServiceProxy('/arm_teleop/left_arm_ik_srv', ArmIK)
        self.initial_left_robot_pose = [0.301, -0.358, -0.333, 3.0905722, 0.0360597, -0.0010818]  # XYZ + 欧拉角 (弧度)
        self.initial_left_robot_pose_aa = [0.26164, -0.35714, -0.06994, 3.0905722, 0.0360597, -0.0010818]  # XYZ + 欧拉角 (弧度)

        self.init_left_rotation = R.from_euler("XYZ", 
                                        [self.initial_left_robot_pose[3], 
                                         self.initial_left_robot_pose[4], 
                                         self.initial_left_robot_pose[5]], 
                                        degrees=False)
        self.init_left_rotation_aa = R.from_euler("XYZ", 
                                        [self.initial_left_robot_pose_aa[3], 
                                         self.initial_left_robot_pose_aa[4], 
                                         self.initial_left_robot_pose_aa[5]], 
                                        degrees=False)
        # self.last_left_joint_angles = [-0.0433303, 0.141567, 0.0831955, 1.59424, -1.37614, -0.115441, -0.00507801]
        # self.last_left_joint_angles = [-0.0433303, 0.141567, 0.0831955, 1.59424, -1.37614, -0.115441, -0.00507801]
        self.last_left_joint_angles = [-0.046, 0.2, 0.0, 1.6, -1.32, -0.005, -0.005]

        pq_request = MovejServiceRequest()
        pq_request.arm_id = 0
        pq_request.target_joints = self.last_left_joint_angles
        pq_request.vel = 0.5
        pq_request.acc = 5.0
        pq_request.jerk = 10.0
        pq_response = self.pq_movej_service.call(pq_request)
        self.last_left_joint_angles = [round(angle, 4) for angle in self.last_left_joint_angles]
        self.initial_left_robot_pose_in_quat = self.euler_to_quaternion(self.initial_left_robot_pose)
        self.initial_left_robot_pose_in_quat = [round(angle, 4) for angle in self.initial_left_robot_pose_in_quat]
        self.current_arm_angle_left = -1.2 + 0.5
        # self.current_arm_angle_left = 0
        logger.info("LEFT ARM 已连接到逆运动学服务, 初始化完成。")
        rospy.loginfo(f"[LEFT ARM]机械臂末端初始位置: {[round(x, 4) for x in self.initial_left_robot_pose]}")



        # ============== Start Dual TeleOP Service ==============
        # rospy.wait_for_service('/aris_node/start_teleop_srv')
        self.start_teleop_service = rospy.ServiceProxy('/aris_node/start_teleop_srv', StartDualTeleOP)
        tele_req = StartDualTeleOPRequest()
        tele_req.running_flag = True
        tele_response = self.start_teleop_service.call(tele_req)
        if tele_response.success:
            logger.info("已通知底层控制节点启动双臂遥操作模式")
        else:
            logger.error("通知底层控制节点启动双臂遥操作模式失败")
        
        
        # ================== IK搜索次数记录 ==================
        self.right_ik_cnt = np.nan  # 右臂逆解搜索次数，-1表示未执行
        self.left_ik_cnt = np.nan   # 左臂逆解搜索次数，-1表示未执行
        
        # ================= 安全范围设置 ==================
        # 设置安全操作范围
        self.x_range = self.config.get('x_range')  # 前后
        self.y_range = self.config.get('y_range')  # 左右
        self.z_range = self.config.get('z_range')  # 上下
        
        # 旋转角度安全范围
        self.rx_range = self.config.get('rx_range')  # rx旋转范围
        self.ry_range = self.config.get('ry_range')  # ry旋转范围
        self.rz_range = self.config.get('rz_range')  # rz旋转范围
        
        # 如果配置中没有明确指定旋转范围，但需要限制旋转，可以设置默认值
        # if self.rx_range is None:
        #     rx_init = self.initial_robot_pose[3]
        #     self.rx_range = (rx_init , rx_init)  # 约±10度
        # if self.ry_range is None:    
        #     ry_init = self.initial_robot_pose[4]
        #     self.ry_range = (ry_init , ry_init)  # 约±10度
        # if self.rz_range is None:    
        #     rz_init = self.initial_robot_pose[5]
        #     self.rz_range = (rz_init , rz_init)  # 约±10度
        # logger.info(f"旋转范围已限制: RX: {self.rx_range}, RY: {self.ry_range}, RZ: {self.rz_range}")
        

        # ================== 控制参数 ===================
        self.running = False
        self.update_frequency = self.config.get('update_frequency', 0.01)  # 更新频率 (秒)
        self.control_thread = None # single arm control thread
        self.control_thread_right = None # right arm control thread
        self.control_thread_left = None # left arm control thread
        self.scaling_factor = 1.0 # 手部运动到机械臂运动的缩放因子

        # ============ OneEuroFilter 用于姿态平滑 ============
        self.pose_filter_right = PoseFilter7D(min_cutoff=0.05, beta=0.5)
        self.pose_filter_left = PoseFilter7D(min_cutoff=0.05, beta=0.5)
        # t_now = time.time()
        # self.joints_filter_right = OneEuroFilter(t_now, np.array(self.last_right_joint_angles), min_cutoff=0.001, beta=0.8)
        # self.last_smooth_joints_right = self.last_right_joint_angles.copy()
        # t_now = time.time()
        # self.joints_filter_left = OneEuroFilter(t_now, np.array(self.last_left_joint_angles), min_cutoff=0.001, beta=0.8)
        # self.last_smooth_joints_left = self.last_left_joint_angles.copy()
        
        # ============ 滑动窗口平滑 ============
        # 添加姿态平滑过滤参数
        # self.smoothing_factor = self.config.get('smoothing_factor', 0.5)  # 值越大，平滑效果越强(0-1)
        # rospy.loginfo(f"[RIGHT ARM]初始位姿: {self.initial_right_robot_pose}")
        # self.last_target_pose_right = self.initial_right_robot_pose.copy()
        # self.position_buffer_right = []
        # rospy.loginfo(f"[LEFT ARM]初始位姿: {self.initial_left_robot_pose}")
        # self.last_target_pose_left = self.initial_left_robot_pose.copy()
        # self.position_buffer_left = []
        
        # 添加关节平滑相关参数
        # self.joints_smoothing_factor = self.config.get('smoothing_factor', 0.99)  # 关节平滑系数
        self.joints_smoothing_factor = 0.99
        rospy.loginfo(f"[RIGHT ARM] Last joint angles: {self.last_right_joint_angles}")
        self.last_smooth_joints_right = self.last_right_joint_angles.copy() if self.last_right_joint_angles is not None else None
        self.joints_buffer_right = []
        rospy.loginfo(f"[LEFT ARM] Last joint angles: {self.last_left_joint_angles}")
        self.last_smooth_joints_left = self.last_left_joint_angles.copy() if self.last_left_joint_angles is not None else None
        self.joints_buffer_left = []
        
        self.teleop_active = True  # 默认激活遥操作
        logger.info("遥操作初始化完成，等待校准手部位置...")
        # 校准手部位置
        self.calibrate_right_hand_position()
        self.calibrate_left_hand_position()

        self.lastest_head_z_rotation = 0.0
        # self.calibrate_head_position()

        # self.head_thread = None
        logger.info("启动 头部数据获取线程...")
        # self.head_thread = Thread(target=self.head_loop, daemon=True) 
        # self.head_thread.start()

        # For data logging
        # 【修改】使用 perf_counter 作为基准时间，用于高精度耗时计算
        self._base_perf_time = time.perf_counter()
        
        self.vp_time = 0.0
        self.mapping_time = 0.0
        self.euro_time = 0.0
        self.ik_time = 0.0
        self.filter_time = 0.0
        self.loop_cost_time = 0.0
        self.last_target_pose_right_smooth = [0.0] * 7
        self.last_target_pose_right = [0.0] * 7
        self.last_target_pose_left_smooth = [0.0] * 7
        self.last_target_pose_left = [0.0] * 7
        self.last_target_pose_left_quat = [0.0] * 7
        self.last_target_pose_right_quat = [0.0] * 7

        # 线程锁，防止其他线程更新数据时在此处读取造成数据撕裂
        self.data_lock = threading.Lock()
        
        # 2. 状态控制标志
        self.is_recording = False
        
        # 3. 指定保存路径并生成完整的文件路径
        save_directory = "/home/pangu/pangu/src/arm_teleop/data_log" 
        # /home/pangu/pangu/src/arm_teleop/vptele/arm_control/arm_teleop_ros.py
        
        # 检查文件夹是否存在，如果不存在则自动创建
        os.makedirs(save_directory, exist_ok=True)
        
        # 生成时间戳和完整的文件名
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_filename = os.path.join(save_directory, f"teleop_data_{timestamp_str}.csv") # <--- 核心修改点
        
        # 打开文件
        self.file_handle = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.file_handle)
        
        # 写入 CSV 表头
        self._write_csv_header()
        
        # 注册 rospy 退出时的回调，确保程序结束时文件安全关闭
        rospy.on_shutdown(self._shutdown_hook)

        # 4. 创建 ROS 服务，用于控制记录的启停
        """
        rosservice call /toggle_data_recording "data: true"
        rosservice call /toggle_data_recording "data: false"
        """
        self.record_srv = rospy.Service(
            '/toggle_data_recording', 
            SetBool, 
            self._handle_record_srv
        )
        rospy.loginfo(f"数据记录节点已就绪。文件保存至: {self.csv_filename}")


        self.logging_thread = Thread(target=self.logging_loop, daemon=True)
        self.logging_thread.start()
    

    def logging_loop(self):
        """
        记录遥操作数据的循环函数,将数据保存为csv文件
        """
        rate = rospy.Rate(25)
        
        while not rospy.is_shutdown():
            if self.is_recording:
                # 【修改】使用 time.time() 记录绝对时间，用于后期对齐日志
                abs_timestamp = time.time()
                # 【修改】使用 perf_counter 记录相对时间，用于精确计算耗时
                rel_timestamp = time.perf_counter() - self._base_perf_time
                
                # 【修改】使用锁安全地复制所有当前数据，防止数据撕裂
                with self.data_lock:
                    row_data = [abs_timestamp, rel_timestamp]
                    
                    # 1. 写入位姿数据
                    row_data.extend(self.last_target_pose_right_smooth)
                    row_data.extend(self.last_target_pose_right_quat)
                    row_data.extend(self.last_target_pose_left_smooth)
                    row_data.extend(self.last_target_pose_left_quat)
                    # 28列位姿数据: 右臂平滑(7) + 右臂原始(7) + 左臂平滑(7) + 左臂原始(7)
                    
                    # 2. 写入臂角数据 (标量，用 append)
                    row_data.append(self.current_arm_angle_left)
                    row_data.append(self.current_arm_angle_right)
                    # 30-31列: 左臂臂角 + 右臂臂角
                    # ================== 新增：IK搜索次数 ==================
                    row_data.append(self.left_ik_cnt)
                    row_data.append(self.right_ik_cnt)
                    
                    # 3. 写入关节角度数据 (列表，用 extend)
                    row_data.extend(self.last_left_joint_angles)
                    row_data.extend(self.last_smooth_joints_left)
                    row_data.extend(self.last_right_joint_angles)
                    row_data.extend(self.last_smooth_joints_right)
                    # 32-45列: 左臂原始7个关节角 + 左臂平滑7个关节角 + 右臂原始7个关节角 + 右臂平滑7个关节角

                    # 4. 耗时
                    # 【修改】使用 max(0.0, ...) 防止浮点误差导致负数
                    vp_dur = self.vp_time
                    map_dur = max(0.0, self.mapping_time - self.vp_time)
                    euro_dur = max(0.0, self.euro_time - self.mapping_time)
                    ik_dur = max(0.0, self.ik_time - self.euro_time)
                    filter_dur = max(0.0, self.filter_time - self.ik_time)
                    
                    row_data.append(vp_dur)
                    row_data.append(map_dur)
                    row_data.append(euro_dur)
                    row_data.append(ik_dur)
                    row_data.append(filter_dur)
                    row_data.append(self.loop_cost_time)

                
                # 一次性写入这一行的所有数据
                self.csv_writer.writerow(row_data)
                
            rate.sleep()
    
    def _write_csv_header(self):
        """生成并写入CSV表头"""
        header = ['abs_timestamp', 'rel_timestamp']  # 【修改】增加两列时间戳
        prefixes = ['r_smooth', 'r_raw', 'l_smooth', 'l_raw']
        suffixes = ['px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz']
        
        for p in prefixes:
            for s in suffixes:
                header.append(f"{p}_{s}")
        
        # 2. 新增：臂角表头 (2列)
        header.extend(['arm_angle_left', 'arm_angle_right'])
        # ================== 新增：IK搜索次数列 ==================
        header.extend(['ik_search_cnt_left', 'ik_search_cnt_right'])
        
        # 3. 新增：关节角度表头 (4 * num_joints 列)
        for i in range(7):
            header.append(f"l_joint_{i}")
        for i in range(7):
            header.append(f"l_smooth_joint_{i}")
        for i in range(7):
            header.append(f"r_joint_{i}")
        for i in range(7):
            header.append(f"r_smooth_joint_{i}")
        
        header.append('vp_time')
        header.append('mapping_time')
        header.append('euro_time')
        header.append('ik_time')
        header.append('filter_time')
        header.append('loop_cost_time')




        self.csv_writer.writerow(header)
        self.file_handle.flush() # 确保表头立即写入

    def _handle_record_srv(self, req):
        """ROS 服务回调函数：处理开始/停止记录的请求"""
        if req.data:
            if not self.is_recording:
                self.is_recording = True
                rospy.loginfo("已开始记录遥操作数据...")
                return SetBoolResponse(success=True, message="Recording Started")
            else:
                return SetBoolResponse(success=False, message="Already Recording")
        else:
            if self.is_recording:
                self.is_recording = False
                self.file_handle.flush() # 停止记录时强制刷入硬盘
                rospy.loginfo("已停止记录遥操作数据。")
                return SetBoolResponse(success=True, message="Recording Stopped")
            else:
                return SetBoolResponse(success=False, message="Not Recording Currently")

    def _shutdown_hook(self):
        """节点关闭时触发，安全释放文件资源"""
        self.is_recording = False
        if self.file_handle and not self.file_handle.closed:
            self.file_handle.flush()
            self.file_handle.close()
            rospy.loginfo(f"CSV文件已安全保存: {self.csv_filename}")

    # def arm_angle_callback(self, msg: ArmAngle):
    #     """接收机械臂当前臂角的回调函数"""
    #     self.current_arm_angle_right = msg.right_arm_angle
    #     self.current_arm_angle_left = msg.left_arm_angle
    
    def calibrate_head_position(self):
        """校准头部位置，记录初始位置作为参考点"""
        # 等待获取有效的头部数据
        max_attempts = 10
        attempts = 0
        
        logger.info("开始校准头部位置...")
        
        while attempts < max_attempts:
            head_data = self.vp_streamer.get_head_data()
            if head_data is not None:
                # 记录头部初始位置
                self.initial_head_transform = head_data[0]
                self.initial_head_position = self.initial_head_transform[:3, 3]
                self.initial_head_rotation = self.initial_head_transform[:3, :3]
                self.init_head_rotation_in_euler = rotation_matrix_to_euler(self.initial_head_rotation)
                # self.init_head_pos = head_data
                logger.info(f"已校准头部位置: {self.initial_head_position}")
                logger.info(f"已校准头部姿态: {rotation_matrix_to_euler(self.initial_head_rotation)}")
                return
            
            time.sleep(0.5)
            attempts += 1
            logger.info(f"等待头部数据... {attempts}/{max_attempts}")
            
        logger.info("警告: 无法获取头部位置进行校准！使用默认值。")
        self.initial_head_position = np.array([0, 0, 0])
        self.initial_head_rotation = np.eye(3)  # 单位矩阵作为默认旋转
    
    def get_head_z_rotation(self):
        """
        获取头部相对于初始位置绕Z轴的旋转角度 (弧度)
        返回:
            z_angle: 绕Z轴的旋转角度 (弧度)
        """
        # 获取当前头部数据
        head_data = self.vp_streamer.get_head_data()
        if head_data is None:
            return 0.0
            
        # 获取当前头部旋转矩阵
        current_transform = head_data[0]
        current_rotation = current_transform[:3, :3]
        
        # 计算相对旋转 R_rel = R_curr * R_init^-1
        # 这样得到的是相对于初始姿态的旋转矩阵
        relative_rotation = current_rotation @ np.linalg.inv(self.initial_head_rotation)
        
        # 转换为欧拉角
        # 使用 'xyz' (extrinsic) 顺序，这样 Z 轴旋转是相对于固定坐标系的
        r = R.from_matrix(relative_rotation)
        euler = r.as_euler('xyz', degrees=False)
        # rospy.loginfo(f"头部相对旋转欧拉角: {[round(angle, 4) for angle in euler]}")
        
        return euler[2]
    
    def calibrate_right_hand_position(self):
        """校准手部位置和姿态，记录初始位置作为参考点"""
        # 等待获取有效的手部数据
        max_attempts = 10
        attempts = 0
        
        logger.info("开始校准手部位置...")
        
        while attempts < max_attempts:
            hand_data = self.vp_streamer.get_hand_position(hand='right')
            if hand_data is not None and len(hand_data) > 0:
                # 记录右手腕初始位置和姿态
                self.initial_hand_transform_right = hand_data[0]
                self.initial_hand_position_right = self.initial_hand_transform_right[:3, 3]
                # self.initial_hand_rotation_right = self.initial_hand_transform_right[:3, :3]
                self.initial_hand_rotation_right = np.array([[0.0, 1.0, 0.0],
                                                             [-1.0, 0.0, 0.0],
                                                             [0.0, 0.0, 1.0]])
                logger.info(f"已校准手部位置: {self.initial_hand_position_right}")
                logger.info(f"已校准手部姿态: {rotation_matrix_to_euler(self.initial_hand_rotation_right)}")
                return
            
            time.sleep(0.5)
            attempts += 1
            logger.info(f"等待手部数据... {attempts}/{max_attempts}")
            
        logger.info("警告: 无法获取手部位置进行校准！使用默认值。")
        self.initial_hand_position_right = np.array([0, 0, 0])
        self.initial_hand_rotation_right = np.eye(3)  # 单位矩阵作为默认旋转
    
    def calibrate_left_hand_position(self):
        """校准手部位置和姿态，记录初始位置作为参考点"""
        # 等待获取有效的手部数据
        max_attempts = 10
        attempts = 0
        
        logger.info("开始校准手部位置...")
        
        while attempts < max_attempts:
            hand_data = self.vp_streamer.get_hand_position(hand='left')
            if hand_data is not None and len(hand_data) > 0:
                # 记录右手腕初始位置和姿态
                self.initial_hand_transform_left = hand_data[0]
                self.initial_hand_position_left = self.initial_hand_transform_left[:3, 3]
                # self.initial_hand_rotation_left = self.initial_hand_transform_left[:3, :3]
                self.initial_hand_rotation_left = np.array([[0.0, 1.0, 0.0],
                                                            [1.0, 0.0, 0.0],
                                                            [0.0, 0.0, -1.0]])
                logger.info(f"已校准手部位置: {self.initial_hand_position_left}")
                logger.info(f"已校准手部姿态: {rotation_matrix_to_euler(self.initial_hand_rotation_left)}")
                return
            
            time.sleep(0.5)
            attempts += 1
            logger.info(f"等待手部数据... {attempts}/{max_attempts}")
            
        logger.info("警告: 无法获取手部位置进行校准！使用默认值。")
        self.initial_hand_position_left = np.array([0, 0, 0])
        self.initial_hand_rotation_left = np.eye(3)  # 单位矩阵作为默认旋转

    def map_hand_to_robot(self, hand_transform, hand_side="right"):
        """
        将手部位置和旋转映射到机械臂位置和姿态
        
        参数:
            hand_transform: 4x4 变换矩阵，包含位置和旋转信息
        """
        # 提取手部位置
        hand_position = hand_transform[:3, 3]
        
        # 计算手部位置相对于初始位置的偏移
        if hand_side == 'right':
            hand_offset = hand_position - self.initial_hand_position_right
        elif hand_side == 'left':
            hand_offset = hand_position - self.initial_hand_position_left
            rospy.loginfo(f"左手手腕偏移: {[round(x, 4) for x in hand_offset]}")
        
        # 将偏移应用到机械臂初始位置
        if hand_side == 'right':
            target_position = self.initial_right_robot_pose.copy()
        elif hand_side == 'left':
            target_position = self.initial_left_robot_pose.copy()
        
        # 调整位置偏移方向和缩放
        target_position[0] += hand_offset[1] * 1.5
        target_position[1] += hand_offset[2] * 1.5
        target_position[2] += hand_offset[0] * 1.5
        
        # 从变换矩阵中提取旋转信息，转为欧拉角
        rotation_matrix = hand_transform[:3, :3]
        
        # 计算相对于初始手部姿态的旋转变化
        # 相对旋转 = 当前旋转 × 初始旋转的逆
        if hand_side == 'right':
            relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation_right)
        elif hand_side == 'left':
            relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation_left)
        # rospy.loginfo(f"相对旋转矩阵: \n{relative_rotation}")
        transfrom_matrix = np.array([[0.0, 1.0, 0.0],
                                     [0.0, 0.0, 1.0],
                                     [1.0, 0.0, 0.0]])

        rotation_in_arm = np.dot(transfrom_matrix, relative_rotation) @ transfrom_matrix.T
        if hand_side == 'right':
            relative_rotation = R.from_matrix(rotation_in_arm @ self.init_right_rotation.as_matrix())
        else:
            relative_rotation = R.from_matrix(rotation_in_arm @ self.init_left_rotation.as_matrix())
        [new_rx, new_ry, new_rz] = relative_rotation.as_euler('XYZ')

        
        
        target_position[3] = new_rx
        target_position[4] = new_ry
        target_position[5] = new_rz

        
        # 转换成 numpy 数组
        target_position = np.array(target_position)
        return target_position
    
    def map_hand_to_robot_aa(self, hand_transform, hand_side="right"):
        """
        将手部位置和旋转映射到机械臂位置和姿态
        
        参数:
            hand_transform: 4x4 变换矩阵，包含位置和旋转信息
        """
        # 提取手部位置
        hand_position = hand_transform[:3, 3]
        
        # 计算手部位置相对于初始位置的偏移
        if hand_side == 'right':
            hand_offset = hand_position - self.initial_hand_position_right
        elif hand_side == 'left':
            hand_offset = hand_position - self.initial_hand_position_left
            rospy.loginfo(f"左手手腕偏移: {[round(x, 4) for x in hand_offset]}")
        
        # 将偏移应用到机械臂初始位置
        if hand_side == 'right':
            raw_target_position = self.initial_right_robot_pose_aa.copy()
        elif hand_side == 'left':
            raw_target_position = self.initial_left_robot_pose_aa.copy()

        raw_target_position[0] += hand_offset[1]
        raw_target_position[1] += hand_offset[2]
        raw_target_position[2] += hand_offset[0]


        
        
        
        # 从变换矩阵中提取旋转信息，转为欧拉角
        rotation_matrix = hand_transform[:3, :3]
        
        # 计算相对于初始手部姿态的旋转变化
        # 相对旋转 = 当前旋转 × 初始旋转的逆
        if hand_side == 'right':
            relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation_right)
        elif hand_side == 'left':
            relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation_left)
        # rospy.loginfo(f"相对旋转矩阵: \n{relative_rotation}")
        transfrom_matrix = np.array([[0.0, 1.0, 0.0],
                                     [0.0, 0.0, 1.0],
                                     [1.0, 0.0, 0.0]])

        rotation_in_arm = np.dot(transfrom_matrix, relative_rotation) @ transfrom_matrix.T
        if hand_side == 'right':
            relative_rotation = R.from_matrix(rotation_in_arm @ self.init_right_rotation_aa.as_matrix())
        else:
            relative_rotation = R.from_matrix(rotation_in_arm @ self.init_left_rotation_aa.as_matrix())
        [new_rx, new_ry, new_rz] = relative_rotation.as_euler('XYZ')

        
        
        # target_position[3] = new_rx
        # target_position[4] = new_ry
        # target_position[5] = new_rz

        raw_target_position[3] = new_rx
        raw_target_position[4] = new_ry
        raw_target_position[5] = new_rz
        # 如果想固定末端姿态，可以取消下面三行的注释
        # target_position[3] = self.initial_robot_pose[3]
        # target_position[4] = self.initial_robot_pose[4]
        # target_position[5] = self.initial_robot_pose[5]
        
        # 转换成 numpy 数组
        # target_position = np.array(target_position)
        raw_target_position = np.array(raw_target_position)
        return raw_target_position
        
    
    def euler_to_quaternion(self, euler_pose, rotation_order='XYZ', degrees=False):
        """
        将欧拉角形式的6维位姿转换为四元数形式的7维位姿
        
        参数:
            euler_pose: 欧拉角位姿 [x, y, z, rx, ry, rz]
            rotation_order: 欧拉角的旋转顺序，默认为 'XYZ'
            degrees: 欧拉角的单位，False为弧度(默认)，True为度
        
        返回:
            quaternion_pose: 四元数位姿 [x, y, z, qw, qx, qy, qz]
        """
        # 确保输入是 numpy 数组
        euler_pose = np.array(euler_pose)
        
        # 分离位置和旋转
        position = euler_pose[:3]  # [x, y, z]
        euler_angles = euler_pose[3:]  # [rx, ry, rz]
        
        # 使用 SciPy 创建旋转对象
        rotation = R.from_euler(rotation_order, euler_angles, degrees=degrees)
        
        # 转换为四元数 (SciPy 返回 [x, y, z, w] 格式)
        quat_scipy = rotation.as_quat()  # 返回 [qx, qy, qz, qw]
        
        # 转换为 [qw, qx, qy, qz] 格式
        quaternion = [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]
        
        # 组合位置和四元数
        quaternion_pose = np.concatenate([position, quaternion])
        
        return quaternion_pose
    

    def euler_to_quaternion_aa(self, euler_pose, rotation_order='XYZ', degrees=False):
        """
        将欧拉角形式的6维位姿转换为四元数形式的7维位姿
        
        参数:
            euler_pose: 欧拉角位姿 [x, y, z, rx, ry, rz]
            rotation_order: 欧拉角的旋转顺序，默认为 'XYZ'
            degrees: 欧拉角的单位，False为弧度(默认)，True为度
        
        返回:
            quaternion_pose: 四元数位姿 [x, y, z, qw, qx, qy, qz]
        """
        # 确保输入是 numpy 数组
        euler_pose = np.array(euler_pose)
        
        # 分离位置和旋转
        position = euler_pose[:3]  # [x, y, z]
        euler_angles = euler_pose[3:]  # [rx, ry, rz]
        
        # 使用 SciPy 创建旋转对象
        rotation = R.from_euler(rotation_order, euler_angles, degrees=degrees)
        
        # 转换为四元数 (SciPy 返回 [x, y, z, w] 格式)
        quat_scipy = rotation.as_quat()  # 返回 [qx, qy, qz, qw]
        
        # 转换为 [qw, qx, qy, qz] 格式
        # quaternion = [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]
        
        # 组合位置和四元数
        quaternion_pose = np.concatenate([position, quat_scipy])
        
        return quaternion_pose
    
    def aa_loop(self, arm_side='right'):
        """臂角预测循环"""
        logger.info(f"============ {arm_side}臂角预测 ============")
        while self.running:
            
            try:
                # 获取最新的手部数据
                hand_data = self.vp_streamer.latest
                hand_data = self.vp_streamer.get_hand_position(hand=arm_side)
                # 只有当遥操作激活时才执行控制
                if self.teleop_active:
                    # 提取右手腕的完整变换矩阵
                    hand_transform = hand_data[0]
                    
                    # 映射到机械臂位置和姿态
                    target_pose_aa = self.map_hand_to_robot_aa(hand_transform, arm_side)
                    
                    target_pose_in_quat_aa = self.euler_to_quaternion_aa(target_pose_aa)

                    aa_req = PredictArmAngleRequest()

                    if arm_side == 'right':
                        aa_req.arm_side = arm_side
                        aa_req.pose = target_pose_in_quat_aa
                        aa_result = self.arm_angle_subscriber.call(aa_req)
                        self.current_arm_angle_right = aa_result.arm_angle_rad * -1 + 0.5
                        
                        
                    elif arm_side == 'left':
                        aa_req.arm_side = arm_side
                        aa_req.pose = target_pose_in_quat_aa
                        aa_result = self.arm_angle_subscriber.call(aa_req)
                        self.current_arm_angle_left = aa_result.arm_angle_rad * -1 + 0.5

                
                
                time.sleep(0.03)

            except Exception as e:
                logger.error(f"臂角预测循环出错: {str(e)}", exc_info=True)  # 使用exc_info=True记录完整堆栈
                time.sleep(1)  # 错误恢复等待



    def control_loop(self, arm_side="left", arm_id=0):
        """控制循环，持续更新机械臂位置和姿态"""
        logger.info(f"开始[{arm_side}]机械臂控制循环")
        
        
        # 添加 FPS 计算相关变量
        frame_count = 0
        last_fps_time = time.time()  # 【保留原样】FPS计算用time.time没问题
        max_record_time = 0.0
        recorded_time = 0.0
        # loop_time = 0.0
        # max_loop_time = 0.0
        while self.running:
            # 【修改】使用 perf_counter 作为循环起点，不受系统时间调整影响
            loop_start_time = time.perf_counter()
            try:
                # 增加帧计数
                frame_count += 1
                current_time = time.time()  # 【保留原样】FPS计算
                
                # 每秒计算并显示一次 FPS
                # if current_time - last_fps_time >= 1.0:
                fps = frame_count / (current_time - last_fps_time)
                logger.info(f"[{arm_side}] 遥操作 FPS: {fps:.2f}")
                frame_count = 0
                last_fps_time = current_time
                
                # 获取最新的手部数据
                hand_data = self.vp_streamer.latest
                hand_data = self.vp_streamer.get_hand_position(hand=arm_side)
                # 【修改】使用 perf_counter 计算耗时
                self.vp_time = time.perf_counter() - loop_start_time
                # 只有当遥操作激活时才执行控制
                if self.teleop_active:
                    # 提取右手腕的完整变换矩阵
                    hand_transform = hand_data[0]
                    
                    # 映射到机械臂位置和姿态
                    target_pose = self.map_hand_to_robot(hand_transform, arm_side) # [位置(x,y,z) + 欧拉角(rx,ry,rz)]
                    # target_pose_aa = self.map_hand_to_robot_aa(hand_transform, arm_side)
                    # raw_target_pose = 
                    logger.info(f'{arm_side}目标位置: {[round(x, 4) for x in target_pose]}')
                    
                    # 应用平滑过滤到位置
                    target_pose_in_quat = self.euler_to_quaternion(target_pose) # 转为四元数形式 [x, y, z, qw, qx, qy, qz]
                    # target_pose_in_quat_aa = self.euler_to_quaternion_aa(target_pose_aa)
                    current_timestamp = time.time()  # 【保留原样】滤波器接口可能需要time.time

                    # 【修改】使用 perf_counter 计算耗时
                    self.mapping_time = time.perf_counter() - loop_start_time

                    # aa_req = PredictArmAngleRequest()

                    if arm_side == 'right':
                        # 对[姿态]进行 1Euro 滤波
                        smooth_target_in_quat = self.pose_filter_right.process(target_pose_in_quat, current_timestamp)

                        # smooth_target_in_quat_aa = self.pose_filter_right.process(target_pose_in_quat_aa, current_timestamp)

                        # 【修改】加锁保护共享数据
                        with self.data_lock:
                            self.last_target_pose_right = target_pose.copy()
                            self.last_target_pose_right_quat = target_pose_in_quat.copy()
                            self.last_target_pose_right_smooth = smooth_target_in_quat.copy()

                        # aa_req.arm_side = arm_side
                        # aa_req.pose = target_pose_in_quat_aa
                        # aa_result = self.arm_angle_subscriber.call(aa_req)
                        
                        
                    elif arm_side == 'left':
                        smooth_target_in_quat = self.pose_filter_left.process(target_pose_in_quat, current_timestamp)

                        # smooth_target_in_quat_aa = self.pose_filter_right.process(target_pose_in_quat_aa, current_timestamp)

                        # 【修改】加锁保护共享数据
                        with self.data_lock:
                            self.last_target_pose_left = target_pose.copy()
                            self.last_target_pose_left_quat = target_pose_in_quat.copy()
                            self.last_target_pose_left_smooth = smooth_target_in_quat.copy()

                        # aa_req.arm_side = arm_side
                        # aa_req.pose = target_pose_in_quat_aa
                        # aa_result = self.arm_angle_subscriber.call(aa_req)
                        
                    
                    
                    # Log position only once every 0.1 seconds
                    current_time = time.time()
                    if not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 0.1:
                        # logger.info(f"目标位置: {[round(x, 4) for x in smooth_target]}")
                        self.last_log_time = current_time
                    # 【修改】使用 perf_counter 计算耗时
                    self.euro_time = time.perf_counter() - loop_start_time



                    # 使用逆运动学计算关节角度
                    # TODO: call 逆解服务
                    start_time = time.time()
                    ik_request = ArmIKRequest()
                    # ======================== [feasible method] ========================
                    if arm_side == 'right':
                        ik_request.method = 'feasible_ref'  # 使用组合方法
                        # ik_request.current_arm_angle = aa_result.arm_angle_rad * -1
                        # 【修改】加锁读取臂角
                        with self.data_lock:
                            # ik_request.current_arm_angle = self.current_arm_angle_right
                            ik_request.current_arm_angle = -1 + 0.5
                        logger.info(f"{arm_side}使用的臂角大小为{self.current_arm_angle_right}")
                    elif arm_side == 'left':
                        ik_request.method = 'feasible_ref'  # 使用组合方法
                        # ik_request.current_arm_angle = aa_result.arm_angle_rad * -1
                        # 【修改】加锁读取臂角
                        with self.data_lock:
                            # ik_request.current_arm_angle = self.current_arm_angle_left
                            ik_request.current_arm_angle = -1 + 0.5
                        logger.info(f"{arm_side}使用的臂角大小为{self.current_arm_angle_left}")
                    
                    
                    
                    # ik_request.current_arm_angle = aa_result.arm_angle_rad
                    # rospy.loginfo(f"arm_side: {arm_side}  -  arm_angle: {aa_result}")


                    # 3. 直接在当前进程暴力破解 IK！
                    offset_list1 = [0, -0.1, -0.2, -0.3, -0.4, -0.5]
                    # ⚠️ 注意这里：也必须改为围绕修正后的 `ik_phi` 进行搜索，而不是原来的 `predicted_arm_angle_rad`
                    if arm_side == "right":
                        # 【修改】加锁读取臂角
                        with self.data_lock:
                            offset_list2 = [i + self.current_arm_angle_right for i in [0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2]]
                    else:
                        # 【修改】加锁读取臂角
                        with self.data_lock:
                            offset_list2 = [i + self.current_arm_angle_left for i in [0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2]]
                    search_list = offset_list1 + offset_list2
                    
                    ik_request.offset_list = search_list
                    ik_request.offset_refer = 0.45

                    rospy.loginfo(f"[{arm_side}] 请求逆解服务，目标位姿: {[round(x, 4) for x in smooth_target_in_quat]}")
                    ik_request.target_pose.position.x = smooth_target_in_quat[0]
                    ik_request.target_pose.position.y = smooth_target_in_quat[1]
                    ik_request.target_pose.position.z = smooth_target_in_quat[2]
                    ik_request.target_pose.orientation.w = smooth_target_in_quat[3]
                    ik_request.target_pose.orientation.x = smooth_target_in_quat[4]
                    ik_request.target_pose.orientation.y = smooth_target_in_quat[5]
                    ik_request.target_pose.orientation.z = smooth_target_in_quat[6]
                    if arm_side == 'right':
                        # 【修改】加锁读取关节角
                        with self.data_lock:
                            ik_request.init_joints = self.last_right_joint_angles if self.last_right_joint_angles is not None else []
                        response = self.right_ik_service.call(ik_request)
                        with self.data_lock:
                       # ✅ 核心转换逻辑
                            if response.search_cnt >= 0:
                               self.right_ik_cnt = response.search_cnt  # 成功：使用真实次数
                            else:
                               self.right_ik_cnt = np.nan  # 失败：转为NaN
                    elif arm_side == 'left':
                        # 【修改】加锁读取关节角
                        with self.data_lock:
                            ik_request.init_joints = self.last_left_joint_angles if self.last_left_joint_angles is not None else []
                        response = self.left_ik_service.call(ik_request)
                        with self.data_lock:
                            # ✅ 核心转换逻辑
                            if response.search_cnt >= 0:
                               self.left_ik_cnt = response.search_cnt  # 成功：使用真实次数
                            else:
                               self.left_ik_cnt = np.nan  # 失败：转为NaN
                    success = response.success
                    joint_angles = response.solution
                    recorded_time = time.time() - start_time
                    max_record_time = recorded_time if recorded_time > max_record_time else max_record_time 
                    rospy.loginfo(f"{arm_side}逆解耗时: {time.time() - start_time:.4f} 秒")
                    rospy.loginfo(f"当前最大逆解耗时: {max_record_time:.4f} 秒")
                    # 【修改】使用 perf_counter 计算耗时
                    self.ik_time = time.perf_counter() - loop_start_time
                    

                    
                    if success:
                        # 更新最后使用的关节角度
                        if arm_side == 'right':
                            # 【修改】加锁保护所有共享数据
                            with self.data_lock:
                                self.current_arm_angle_right = response.new_arm_angle
                                self.last_right_joint_angles = [round(angle, 4) for angle in joint_angles]
                        elif arm_side == 'left':
                            # 【修改】加锁保护所有共享数据
                            with self.data_lock:
                                self.current_arm_angle_left = response.new_arm_angle
                                self.last_left_joint_angles = [round(angle, 4) for angle in joint_angles]
                        rospy.loginfo(f"[{arm_side}] 当前臂角: {self.current_arm_angle_right if arm_side == 'right' else self.current_arm_angle_left}")
                        logger.info(f"[{arm_side}] 当前臂角: {self.current_arm_angle_right if arm_side == 'right' else self.current_arm_angle_left}")
                        rospy.loginfo(f"[{arm_side}] 逆解成功，关节角度: {[round(angle, 4) for angle in joint_angles]}")
                        logger.info(f"[{arm_side}] 逆解成功")
                        
                        # 对计算的关节角度进行平滑处理
                        
                        if arm_side == 'right':
                            #     # 对[关节角度]进行 1Euro 滤波
                            #     # if self.last_smooth_joints_right is not None:
                            #     #     smooth_joints = self.joints_filter_right(current_timestamp, np.array(joint_angles))
                            #     #     self.last_smooth_joints_right = list(smooth_joints).copy()
                            # 【修改】加锁保护共享数据
                            with self.data_lock:
                                if self.last_smooth_joints_right is not None:
                                    smooth_joint_angles, self.joints_buffer_right = smooth_values(
                                        joint_angles,
                                        self.last_smooth_joints_right,
                                        self.joints_buffer_right,
                                        self.joints_smoothing_factor
                                    )
                                    self.last_smooth_joints_right = smooth_joint_angles.copy()
                                else:
                                    smooth_joint_angles = joint_angles
                                    self.last_smooth_joints_right = joint_angles.copy()
                        elif arm_side == 'left':
                            #     # if self.last_smooth_joints_left is not None:
                            #     #     smooth_joints = self.joints_filter_left(current_timestamp, np.array(joint_angles))
                            #     #     self.last_smooth_joints_left = list(smooth_joints).copy()
                            # 【修改】加锁保护共享数据
                            with self.data_lock:
                                if self.last_smooth_joints_left is not None:
                                    smooth_joint_angles, self.joints_buffer_left = smooth_values(
                                        joint_angles,
                                        self.last_smooth_joints_left,
                                        self.joints_buffer_left,
                                        self.joints_smoothing_factor
                                    )
                                    self.last_smooth_joints_left = smooth_joint_angles.copy()
                                else:
                                    smooth_joint_angles = joint_angles
                                    self.last_smooth_joints_left = joint_angles.copy()
                        
                        # for i in range(len(joint_angles)):
                        #     if arm_side == "right":
                        #         if abs(joint_angles[i] - self.last_right_joint_angles[i]) > 0.5:
                        #             self.last_right_joint_angles[i] = joint_angles[i]
                        #         self.last_smooth_joints_right = self.last_right_joint_angles.copy()
                        #     elif arm_side == "left":
                        #         if abs(joint_angles[i] - self.last_left_joint_angles[i]) > 0.5:
                        #             self.last_left_joint_angles[i] = joint_angles[i]
                        #         self.last_smooth_joints_left = self.last_left_joint_angles.copy()
                            
                            
                        
                        # if self.config.get('move', True):
                        #     rospy.loginfo(f"[{arm_side}]移动到关节角度位置: {[round(x, 4) for x in smooth_joint_angles]}")
                        #     logger.info(f"[{arm_side}]移动到关节角度位置: {[round(x, 4) for x in smooth_joint_angles]}")

                    else:
                        rospy.logwarn(f"[{arm_side}] 逆解失败，无法控制到位置: {joint_angles}")
                        logger.error(f"[{arm_side}] 逆解失败，无法控制到位置: {joint_angles}")
                    
                    # 【修改】使用 perf_counter 计算耗时
                    self.filter_time = time.perf_counter() - loop_start_time
                    
                
                # 等待一段时间再更新
                # 【修改】使用 perf_counter 计算耗时
                self.loop_cost_time = time.perf_counter() - loop_start_time

                if self.loop_cost_time > 0.03:
                    continue
                else:
                    diff_time = 0.03 - self.loop_cost_time
                    time.sleep(diff_time)
                
            except Exception as e:
                logger.error(f"控制循环出错: {str(e)}", exc_info=True)  # 使用exc_info=True记录完整堆栈
                time.sleep(1)  # 错误恢复等待

    def head_loop(self):
        """持续发布头部数据的循环线程"""
        while not rospy.is_shutdown():
            head_z_rotation = self.get_head_z_rotation()
            # throttle head angle logs to avoid log spam
            if abs(head_z_rotation - self.lastest_head_z_rotation) > 0.05:
                rospy.loginfo(f"更新头部绕Z轴旋转角度: {head_z_rotation}")
                self.lastest_head_z_rotation = head_z_rotation
            if abs(head_z_rotation) <= 0.1:
                self.lastest_head_z_rotation = 0.0
            rospy.sleep(0.01)
        
    
    def publish_loop(self):
        """发布线程"""
        rate = self.publish_rate
        rospy.loginfo(f"以{rate} Hz频率发布双臂关节数据")
        
        while self.running and not rospy.is_shutdown():
            try:
                # 创建双臂消息
                # rospy.loginfo(f"头部绕Z轴旋转角度: {self.get_head_z_rotation()}")
                dual_arm_msg = DualArmMovej()
                # head_z_rotation = self.get_head_z_rotation()
                # rospy.loginfo(f"头部绕Z轴旋转角度: {head_z_rotation}")
                # if abs(head_z_rotation) > 0.1:
                #     rospy.loginfo(f"更新头部绕Z轴旋转角度: {head_z_rotation}")
                #     self.lastest_head_z_rotation = head_z_rotation
                # else:
                #     rospy.loginfo(f"保持头部绕Z轴旋转角度不变: {self.lastest_head_z_rotation}")
                #     dual_arm_msg.head_z_rotation = self.lastest_head_z_rotation
                
                
                # 设置header
                dual_arm_msg.header = Header()
                dual_arm_msg.header.stamp = rospy.Time.now()
                dual_arm_msg.header.frame_id = "pangu_base"
                # 【修改】加锁保护 sequence 和关节角
                with self.data_lock:
                    dual_arm_msg.sequence = self.sequence
                    self.sequence += 1
                    
                    # 更新左右臂数据
                    dual_arm_msg.right_arm.arm_id = 1
                    dual_arm_msg.left_arm.arm_id = 0
                    
                    dual_arm_msg.right_arm.arm_joints = self.last_smooth_joints_right
                    dual_arm_msg.left_arm.arm_joints = self.last_smooth_joints_left
                
                # dual_arm_msg.right_arm.arm_joints = [0,0,0,0,0,0,0]
                # dual_arm_msg.left_arm.arm_joints =  [0.314957,   0.238734,   -0.658534,   1.496385,   -1.000000,   -0.080329,   -0.113492 ]
                # dual_arm_msg.left_arm.arm_joints = [0.555753,   0.369511,   -0.749425,   1.503231,   -0.766683,   -0.061464,   -0.051642]

                # [0.314957,   0.238734,   -0.658534,   1.496385,   -1.000000,   -0.080329,   -0.113492 ]

                # dual_arm_msg.left_arm.arm_joints = [0,0,0,0,0,0,0]

                # dual_arm_msg.head_z_rotation = self.lastest_head_z_rotation
                dual_arm_msg.head_z_rotation = 0.0
                # self.robot_controller.set_dual_arm_positions(self.last_smooth_joints_right,self.last_smooth_joints_left)
                
                
                # 发布数据
                self.dual_arm_publisher.publish(dual_arm_msg)
                
                    
            except Exception as e:
                rospy.logerr("Publish error: %s", str(e))
            rate.sleep()
            # rospy.sleep(1)
    
    
    def start(self):
        """开始遥操作控制"""
        if self.running:
            logger.info("机械臂遥控器已在运行")
            return
            
        logger.info("启动机械臂遥操作控制...")
        self.running = True
        self.control_thread = Thread(target=self.control_loop, name="ArmTeleopThread")
        self.control_thread.daemon = True
        self.control_thread.start()
    
    def multi_start(self):
        """多线程启动遥操作控制"""
        if self.running:
            logger.info("机械臂遥控器已在运行")
            return
        self.running = True
        
        logger.info("启动 [RIGHT ARM] 机械臂遥操作控制线程...")
        self.control_thread_right = Thread(
            target=self.control_loop, 
            kwargs={'arm_side': "right",
                    'arm_id': 1},
            name="RightArmTeleopThread")
        self.control_thread_right.daemon = True
        self.control_thread_right.start()

        logger.info("启动 [RIGHT ARM] 机械臂臂角预测线程...")
        self.aa_thread_right = Thread(
            target=self.aa_loop, 
            kwargs={'arm_side': "right"},
            name="RightArmAAThread")
        self.aa_thread_right.daemon = True
        self.aa_thread_right.start()

        

        logger.info("启动 [LEFT ARM] 机械臂遥操作控制线程...")
        self.control_thread_left = Thread(
            target=self.control_loop, 
            kwargs={'arm_side': "left", 
                    'arm_id': 0},
            name="LeftArmTeleopThread")
        self.control_thread_left.daemon = True
        self.control_thread_left.start()

        logger.info("启动 [LEFTT ARM] 机械臂臂角预测线程...")
        self.aa_thread_left = Thread(
            target=self.aa_loop, 
            kwargs={'arm_side': "left"},
            name="LeftArmAAThread")
        self.aa_thread_left.daemon = True
        self.aa_thread_left.start()

        # logger.info("启动 头部数据获取线程...")
        # self.head_thread = Thread(target=self.head_loop, daemon=True) 
        # self.head_thread.start()

        logger.info("启动 [DUAL AMR] 双臂数据发布线程...")
        self.publisher_thread = Thread(target=self.publish_loop, name="ArmTeleopPublishThread")
        self.publisher_thread.daemon = True
        self.publisher_thread.start()

        





        
    def stop(self):
        """停止遥操作控制"""
        if not self.running:
            return
            
        logger.info("停止机械臂遥操作控制...")
        
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=2)
        if self.control_thread_right:
            self.control_thread_right.join(timeout=2)
        if self.control_thread_left:
            self.control_thread_left.join(timeout=2)
        if self.publisher_thread:
            self.publisher_thread.join(timeout=2)
        logger.info("机械臂遥操作已停止")