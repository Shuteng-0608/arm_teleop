import rospy
from arm_teleop.srv import ArmIK, ArmIKRequest
from arm_teleop.srv import MovejService, MovejServiceRequest
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
# from pangu_msgs.msg import ArmJoints, DualArmMovej
from arm_teleop.msg import ArmJoints, DualArmMovej
from arm_angle.msg import ArmAngle
from arm_teleop.msg import DualTelePQ
from arm_teleop.msg import ArmPQ
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
logger = get_logger()

class ArmTeleopPure:
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
        
        # ================= DUAL ARM DATA PUBLISHER ==================
        self.dual_arm_publisher = rospy.Publisher('/arm_teleop/dual_arm_pq', DualTelePQ, queue_size=100)
        self.publish_rate = rospy.Rate(100)
        self.publisher_thread = None

        self.data_lock = threading.RLock()
        self.sequence = 0
        


        # ================== [RIGHT ARM INIT] ==================
        self.initial_right_robot_pose = [0.3011, -0.3580, 0.2282, 3.1923149, -0.036102, -0.0007987]  # XYZ + 欧拉角 (弧度)
        self.init_right_rotation = R.from_euler("XYZ", 
                                        [self.initial_right_robot_pose[3], 
                                         self.initial_right_robot_pose[4], 
                                         self.initial_right_robot_pose[5]], 
                                        degrees=False)
        self.last_right_joint_angles = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]
        self.last_right_joint_angles = [round(angle, 4) for angle in self.last_right_joint_angles]
        self.last_target_pose_right = self.euler_to_quaternion(self.initial_right_robot_pose).tolist()
        logger.info("RIGHT ARM 已连接到逆运动学服务, 初始化完成。")
        rospy.loginfo(f"[RIGHT ARM]机械臂末端初始位置: {[round(x, 4) for x in self.initial_right_robot_pose]}")


        # ================== [LEFT ARM INIT] ===================
        self.initial_left_robot_pose = [0.301, -0.358, -0.333, 3.0905722, 0.0360597, -0.0010818]  # XYZ + 欧拉角 (弧度)
        self.init_left_rotation = R.from_euler("XYZ", 
                                        [self.initial_left_robot_pose[3], 
                                         self.initial_left_robot_pose[4], 
                                         self.initial_left_robot_pose[5]], 
                                        degrees=False)
        # self.last_left_joint_angles = [-0.0433303, 0.141567, 0.0831955, 1.59424, -1.37614, -0.115441, -0.00507801]
        self.last_left_joint_angles = [-0.0433303, 0.141567, 0.0831955, 1.59424, -1.37614, -0.115441, -0.00507801]
        self.last_left_joint_angles = [round(angle, 4) for angle in self.last_left_joint_angles]
        self.last_target_pose_left = self.euler_to_quaternion(self.initial_left_robot_pose).tolist()
        logger.info("LEFT ARM 已连接到逆运动学服务, 初始化完成。")
        rospy.loginfo(f"[LEFT ARM]机械臂末端初始位置: {[round(x, 4) for x in self.initial_left_robot_pose]}")


        # ================== 控制参数 ===================
        self.running = False
        self.update_frequency = self.config.get('update_frequency', 0.01)  # 更新频率 (秒)
        self.control_thread = None # single arm control thread
        self.control_thread_right = None # right arm control thread
        self.control_thread_left = None # left arm control thread
        self.scaling_factor = 1.0 # 手部运动到机械臂运动的缩放因子

        # ============ OneEuroFilter 用于姿态平滑 ============
        self.pose_filter_right = PoseFilter7D(min_cutoff=0.1, beta=0.8)
        self.pose_filter_left = PoseFilter7D(min_cutoff=0.1, beta=0.5)
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
        self.joints_smoothing_factor = self.config.get('smoothing_factor', 0.5)  # 关节平滑系数
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

        # # self.head_thread = None
        # logger.info("启动 头部数据获取线程...")
        # self.head_thread = Thread(target=self.head_loop, daemon=True) 
        # self.head_thread.start()

    def arm_angle_callback(self, msg: ArmAngle):
        """接收机械臂当前臂角的回调函数"""
        self.current_arm_angle_right = msg.right_arm_angle
        self.current_arm_angle_left = msg.left_arm_angle
    
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
        rospy.loginfo(f"头部相对旋转欧拉角: {[round(angle, 4) for angle in euler]}")
        
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
        # 如果想固定末端姿态，可以取消下面三行的注释
        # target_position[3] = self.initial_robot_pose[3]
        # target_position[4] = self.initial_robot_pose[4]
        # target_position[5] = self.initial_robot_pose[5]
        
        # 转换成 numpy 数组
        target_position = np.array(target_position)
        return target_position
        
    
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
        # quaternion = [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]
        quaternion = [quat_scipy[0], quat_scipy[1], quat_scipy[2], quat_scipy[3]]
        
        # 组合位置和四元数
        quaternion_pose = np.concatenate([position, quaternion])
        
        return quaternion_pose

    def control_loop(self, arm_side="left", arm_id=0):
        """控制循环，持续更新机械臂位置和姿态"""
        logger.info(f"开始[{arm_side}]机械臂控制循环")
        
        
        # 添加 FPS 计算相关变量
        frame_count = 0
        last_fps_time = time.time()
        max_record_time = 0.0
        recorded_time = 0.0
        while self.running:
            loop_start_time = time.time()
            try:
                # 增加帧计数
                frame_count += 1
                current_time = time.time()
                
                # 每秒计算并显示一次 FPS
                # if current_time - last_fps_time >= 1.0:
                fps = frame_count / (current_time - last_fps_time)
                logger.info(f"[{arm_side}] 遥操作 FPS: {fps:.2f}")
                frame_count = 0
                last_fps_time = current_time
                
                # 获取最新的手部数据
                hand_data = self.vp_streamer.latest
                hand_data = self.vp_streamer.get_hand_position(hand=arm_side)
                # 只有当遥操作激活时才执行控制
                if self.teleop_active:
                    # 提取右手腕的完整变换矩阵
                    hand_transform = hand_data[0]
                    
                    # 映射到机械臂位置和姿态
                    target_pose = self.map_hand_to_robot(hand_transform, arm_side)
                    logger.info(f'{arm_side}目标位置: {[round(x, 4) for x in target_pose]}')
                    
                    # 应用平滑过滤到位置
                    target_pose_in_quat = self.euler_to_quaternion(target_pose) # 转为四元数形式 [x, y, z, qx, qy, qz, qw]
                    current_timestamp = time.time()

                    if arm_side == 'right':
                        # 对[姿态]进行 1Euro 滤波
                        smooth_target_in_quat = self.pose_filter_right.process(target_pose_in_quat, current_timestamp)
                        self.last_target_pose_right = smooth_target_in_quat.tolist()
                        # smooth_target, self.position_buffer_right = smooth_values(
                        #     target_pose, 
                        #     self.last_target_pose_right, 
                        #     self.position_buffer_right, 
                        #     self.smoothing_factor
                        # )
                        # smooth_target = [round(angle, 4) for angle in smooth_target]
                        # self.last_target_pose_right = smooth_target.copy()
                    elif arm_side == 'left':
                        smooth_target_in_quat = self.pose_filter_left.process(target_pose_in_quat, current_timestamp)
                        self.last_target_pose_left = smooth_target_in_quat.tolist()
                        # smooth_target, self.position_buffer_left = smooth_values(
                        #     target_pose, 
                        #     self.last_target_pose_left, 
                        #     self.position_buffer_left, 
                        #     self.smoothing_factor
                        # )
                        # smooth_target = [round(angle, 4) for angle in smooth_target]
                        # self.last_target_pose_left = smooth_target.copy()
                    
                    # logger.info(f"平滑位置: {[round(x, 4) for x in smooth_target]}")
                    
                    # debug 限制位置，全部设置为初始值
                    # smooth_target[:3] = self.initial_robot_pose[:3]

                    # debug 限制旋转，全部设置为初始值
                    # smooth_target[3:] = self.initial_robot_pose[3:]
                    
                    # Log position only once every 0.1 seconds
                    current_time = time.time()
                    if not hasattr(self, 'last_log_time') or current_time - self.last_log_time > 0.1:
                        # logger.info(f"目标位置: {[round(x, 4) for x in smooth_target]}")
                        self.last_log_time = current_time
                    
                    
                    
                
                # 等待一段时间再更新
                loop_cost_time = loop_start_time - time.time()
                if loop_cost_time > 0.01:
                    continue
                else:
                    diff_time = 0.01 - loop_cost_time
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
                dual_arm_pq = DualTelePQ()

                dual_arm_pq.header = Header()
                dual_arm_pq.header.stamp = rospy.Time.now()
                dual_arm_pq.header.frame_id = "pangu_base"
                dual_arm_pq.sequence = self.sequence
                self.sequence += 1
                
                # 右臂数据
                dual_arm_pq.right_arm.arm_id = 1
                dual_arm_pq.right_arm.arm_pq = self.last_target_pose_right

                # 左臂数据
                dual_arm_pq.left_arm.arm_id = 0
                dual_arm_pq.left_arm.arm_pq = self.last_target_pose_left
                
                # 发布数据
                self.dual_arm_publisher.publish(dual_arm_pq)
                
                    
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

        logger.info("启动 [LEFT ARM] 机械臂遥操作控制线程...")
        self.control_thread_left = Thread(
            target=self.control_loop, 
            kwargs={'arm_side': "left", 
                    'arm_id': 0},
            name="LeftArmTeleopThread")
        self.control_thread_left.daemon = True
        self.control_thread_left.start()

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
