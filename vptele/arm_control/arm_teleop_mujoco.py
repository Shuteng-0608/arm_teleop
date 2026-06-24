import rospy
from arm_teleop.srv import ArmIK, ArmIKRequest
from std_srvs.srv import Trigger, TriggerResponse

import time
import numpy as np
from threading import Thread
from utils.math_utils import rotation_matrix_to_euler, smooth_values
from utils.logger import get_logger
from utils.filters import OneEuroFilter, PoseFilter7D
from scipy.spatial.transform import Rotation as R
logger = get_logger()

class ArmTeleopMujoco:
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

        # ================== [RIGHT ARM INIT] ==================
        rospy.loginfo("Connecting to ik service......")
        rospy.wait_for_service('/arm_teleop/right_arm_ik_srv')
        self.right_ik_service = rospy.ServiceProxy('/arm_teleop/right_arm_ik_srv', ArmIK)
        self.initial_right_robot_pose = [0.3011, -0.3580, 0.2282, 3.1923149, -0.036102, -0.0007987]  # XYZ + 欧拉角 (弧度)
        self.init_right_rotation = R.from_euler("XYZ", 
                                        [self.initial_right_robot_pose[3], 
                                         self.initial_right_robot_pose[4], 
                                         self.initial_right_robot_pose[5]], 
                                        degrees=False)
        self.last_right_joint_angles = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]        
        self.last_right_joint_angles = [round(angle, 4) for angle in self.last_right_joint_angles]
        self.initial_right_robot_pose_in_quat = self.euler_to_quaternion(self.initial_right_robot_pose)
        self.initial_right_robot_pose_in_quat = [round(angle, 4) for angle in self.initial_right_robot_pose_in_quat]
        self.current_arm_angle_right = 0.0
        logger.info("RIGHT ARM 已连接到逆运动学服务, 初始化完成。")
        rospy.loginfo(f"[RIGHT ARM]机械臂末端初始位置: {[round(x, 4) for x in self.initial_right_robot_pose]}")
        
        

        # ================== 控制参数 ===================
        self.running = False
        self.update_frequency = self.config.get('update_frequency', 0.01)  # 更新频率 (秒)
        self.control_thread = None # single arm control thread
        self.control_thread_right = None # right arm control thread
        self.scaling_factor = float(self.config.get("scaling_factor", 1.1)) # 手部运动到机械臂运动的缩放因子


        # ============ OneEuroFilter ============
        self.pose_filter_right = PoseFilter7D(min_cutoff=0.1, beta=0.1)
        t_now = time.time()
        self.joints_filter_right = OneEuroFilter(t_now, np.array(self.last_right_joint_angles), min_cutoff=0.1, beta=0.1)
        self.last_smooth_joints_right = self.last_right_joint_angles

        # 添加关节平滑相关参数
        self.joints_smoothing_factor = self.config.get('smoothing_factor', 0.5)  # 关节平滑系数
        rospy.loginfo(f"[RIGHT ARM] Last joint angles: {self.last_right_joint_angles}")
        self.last_smooth_joints_right = self.last_right_joint_angles.copy() if self.last_right_joint_angles is not None else None
        self.joints_buffer_right = []

        
        self.teleop_active = True  # 默认激活遥操作
        logger.info("遥操作初始化完成，等待校准手部位置...")
        # 校准手部位置
        self.calibrate_right_hand_position()

        # self.lastest_head_z_rotation = 0.0
        # self.calibrate_head_position()
        # ================== Episode-level ROS services ==================
        self.teleop_service_ns = self.config.get(
            "teleop_service_ns",
            "/arm_teleop_mujoco",
        )

        self.enable_episode_services = bool(
            self.config.get("enable_episode_services", True)
        )

        if self.enable_episode_services:
            self.srv_stop_teleop = rospy.Service(
                f"{self.teleop_service_ns}/stop",
                Trigger,
                self._handle_stop_teleop_service,
            )

            self.srv_recalibrate_teleop = rospy.Service(
                f"{self.teleop_service_ns}/recalibrate",
                Trigger,
                self._handle_recalibrate_teleop_service,
            )

            self.srv_start_teleop = rospy.Service(
                f"{self.teleop_service_ns}/start",
                Trigger,
                self._handle_start_teleop_service,
            )

            logger.info(
                f"ArmTeleop episode services ready under {self.teleop_service_ns}"
            )
    
    def recalibrate_for_new_episode(self):
        """
        Recalibrate hand reference and reset teleoperation internal states.

        This should be called after MuJoCo arm reset and before starting
        a new recording episode.
        """
        logger.info("开始重新标定新 episode 的遥操作参考...")

        # 先禁止控制循环输出
        self.teleop_active = False

        # 重新标定当前 Vision Pro 右手位置和姿态
        self.calibrate_right_hand_position()

        # 同步关节初值到 MuJoCo 的默认初始关节
        if hasattr(self.robot_controller, "initial_arm_joints_external"):
            self.last_right_joint_angles = list(
                self.robot_controller.initial_arm_joints_external
            )
        else:
            self.last_right_joint_angles = [
                -0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005
            ]

        self.last_right_joint_angles = [
            round(float(x), 4) for x in self.last_right_joint_angles
        ]

        self.last_smooth_joints_right = self.last_right_joint_angles.copy()
        self.joints_buffer_right = []

        # 重置臂角搜索参考
        self.current_arm_angle_right = 0.0

        # 重置滤波器，避免上一条 episode 的平滑状态污染下一条
        t_now = time.time()

        self.pose_filter_right = PoseFilter7D(
            min_cutoff=0.1,
            beta=0.1,
        )

        self.joints_filter_right = OneEuroFilter(
            t_now,
            np.array(self.last_right_joint_angles),
            min_cutoff=0.1,
            beta=0.1,
        )

        # 重新允许遥操作逻辑，但此时线程是否运行由 start() 控制
        self.teleop_active = True

        logger.info(
            "新 episode 遥操作参考已重置，初始关节: "
            f"{[round(x, 4) for x in self.last_right_joint_angles]}"
        )
    
    def _handle_stop_teleop_service(self, req):
        """
        Stop arm teleoperation thread.
        """
        try:
            self.teleop_active = False
            self.stop()

            return TriggerResponse(
                success=True,
                message="Arm teleoperation stopped.",
            )

        except Exception as e:
            logger.error(f"停止遥操作失败: {e}", exc_info=True)
            return TriggerResponse(
                success=False,
                message=f"Failed to stop teleoperation: {e}",
            )


    def _handle_recalibrate_teleop_service(self, req):
        """
        Recalibrate hand reference for a new episode.
        Does not start the control thread.
        """
        try:
            if self.running:
                self.stop()

            self.recalibrate_for_new_episode()

            return TriggerResponse(
                success=True,
                message="Arm teleoperation recalibrated for new episode.",
            )

        except Exception as e:
            logger.error(f"重新标定遥操作失败: {e}", exc_info=True)
            return TriggerResponse(
                success=False,
                message=f"Failed to recalibrate teleoperation: {e}",
            )


    def _handle_start_teleop_service(self, req):
        """
        Start arm teleoperation thread.
        """
        try:
            self.teleop_active = True
            self.start()

            return TriggerResponse(
                success=True,
                message="Arm teleoperation started.",
            )

        except Exception as e:
            logger.error(f"启动遥操作失败: {e}", exc_info=True)
            return TriggerResponse(
                success=False,
                message=f"Failed to start teleoperation: {e}",
            )
    
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
                self.initial_hand_rotation_right = self.initial_hand_transform_right[:3, :3]
                # self.initial_hand_rotation_right = np.array([[0.0, 1.0, 0.0],
                #                                              [-1.0, 0.0, 0.0],
                #                                              [0.0, 0.0, 1.0]])
                logger.info(f"已校准手部位置: {self.initial_hand_position_right}")
                logger.info(f"已校准手部姿态: {rotation_matrix_to_euler(self.initial_hand_rotation_right)}")
                return
            
            time.sleep(0.5)
            attempts += 1
            logger.info(f"等待手部数据... {attempts}/{max_attempts}")
            
        logger.info("警告: 无法获取手部位置进行校准！使用默认值。")
        self.initial_hand_position_right = np.array([0, 0, 0])
        self.initial_hand_rotation_right = np.eye(3)  # 单位矩阵作为默认旋转
    

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
        
        
        # 将偏移应用到机械臂初始位置
        if hand_side == 'right':
            target_position = self.initial_right_robot_pose.copy()
        
        s = self.scaling_factor

        target_position[0] += hand_offset[1] * s
        target_position[1] += hand_offset[2] * s
        target_position[2] += hand_offset[0] * s

        
        # 从变换矩阵中提取旋转信息，转为欧拉角
        rotation_matrix = hand_transform[:3, :3]
        
        # 计算相对于初始手部姿态的旋转变化
        # 相对旋转 = 当前旋转 × 初始旋转的逆
        if hand_side == 'right':
            relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation_right)
        
        rospy.loginfo(f"相对旋转矩阵: \n{relative_rotation}")
        transfrom_matrix = np.array([[0.0, 1.0, 0.0],
                                     [0.0, 0.0, 1.0],
                                     [1.0, 0.0, 0.0]])

        rotation_in_arm = np.dot(transfrom_matrix, relative_rotation) @ transfrom_matrix.T
        
        if hand_side == 'right':
            relative_rotation = R.from_matrix(rotation_in_arm @ self.init_right_rotation.as_matrix())
      
        [new_rx, new_ry, new_rz] = relative_rotation.as_euler('XYZ')
        
        target_position[3] = new_rx
        target_position[4] = new_ry
        target_position[5] = new_rz

        
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
        quaternion = [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]
        
        # 组合位置和四元数
        quaternion_pose = np.concatenate([position, quaternion])
        
        return quaternion_pose

    def control_loop(self, arm_side="right", arm_id=1):
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
                if current_time - last_fps_time >= 1.0:
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
                    target_pose = self.map_hand_to_robot(hand_transform, arm_side) # [x, y, z, rx, ry, rz]

                    # 对[姿态]进行 1Euro 滤波
                    target_pose_in_quat = self.euler_to_quaternion(target_pose) # 转为四元数形式 [x, y, z, qw, qx, qy, qz]
                    current_timestamp = time.time()
                    smooth_target_in_quat = self.pose_filter_right.process(target_pose_in_quat, current_timestamp)
                    self.last_target_pose_right = target_pose.copy()
                    
                    # 使用逆运动学计算关节角度
                    start_time = time.time()

                    ik_request = ArmIKRequest()
                    ik_request.method = 'optimal_ref'  # method in ['feasible_ref', 'optimal_ref', 'optimal_std', 'feasible_std']

                    # ===========  处理Arm Angle  ============ #
                    ik_request.current_arm_angle = 0
                    offset_list2 = [i + self.current_arm_angle_right for i in [0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2]]
                    offset_list1 = [0, -0.1,  -0.2, -0.3, -0.4, -0.5]
                    offset_list = offset_list1 + offset_list2
                    ik_request.offset_list = offset_list
                    ik_request.offset_refer = 0.5

                    # ===========  设置目标位姿 =========== #
                    rospy.loginfo(f"[{arm_side}] 请求逆解服务，目标位姿: {[round(x, 4) for x in smooth_target_in_quat]}")
                    ik_request.target_pose.position.x = smooth_target_in_quat[0]
                    ik_request.target_pose.position.y = smooth_target_in_quat[1]
                    ik_request.target_pose.position.z = smooth_target_in_quat[2]
                    ik_request.target_pose.orientation.w = smooth_target_in_quat[3]
                    ik_request.target_pose.orientation.x = smooth_target_in_quat[4]
                    ik_request.target_pose.orientation.y = smooth_target_in_quat[5]
                    ik_request.target_pose.orientation.z = smooth_target_in_quat[6]
                    ik_request.init_joints = self.last_right_joint_angles if self.last_right_joint_angles is not None else []

                    # ===========  调用逆解服务 =========== #
                    response = self.right_ik_service.call(ik_request)

                    success = response.success
                    joint_angles = response.solution
                    
                    if success:
                        self.current_arm_angle_right = response.new_arm_angle
                        self.last_right_joint_angles = [round(angle, 4) for angle in joint_angles]
                        logger.info(f"[{arm_side}] 目标位姿逆解成功，新的臂角: {self.current_arm_angle_right}")
                        rospy.loginfo(f"[{arm_side}] 当前臂角: {self.current_arm_angle_right}")
                        rospy.loginfo(f"[{arm_side}] 逆解成功，关节角度: {[round(angle, 4) for angle in joint_angles]}")
                        logger.info(f"[{arm_side}] 逆解成功")
                        
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
                            
                        if self.config.get('move', True):
                            rospy.loginfo(f"[{arm_side}]移动到关节角度位置: {[round(x, 4) for x in smooth_joint_angles]}")
                            logger.info(f"[{arm_side}]移动到关节角度位置: {[round(x, 4) for x in smooth_joint_angles]}")
                            # for i in range(len(smooth_joint_angles)):
                            #     if i == 1:
                            #         smooth_joint_angles[i] = -1.0 * smooth_joint_angles[i]

                            self.robot_controller.set_arm_positions(smooth_joint_angles)
                            
                    else:
                        rospy.logwarn(f"[{arm_side}] 逆解失败，无法控制到位置: {joint_angles}")
                    
                
                # 等待一段时间再更新
                # loop_cost_time = time.time() - loop_start_time
                # if loop_cost_time > 0.01:
                #     continue
                # else:
                #     diff_time = 0.01 - loop_cost_time
                #     time.sleep(diff_time)
                
                period = float(self.update_frequency)
                loop_cost_time = time.time() - loop_start_time
                sleep_time = period - loop_cost_time

                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    logger.debug(f"[{arm_side}] 控制循环超时: {loop_cost_time:.4f}s")
                
            except Exception as e:
                logger.error(f"控制循环出错: {str(e)}", exc_info=True)  # 使用exc_info=True记录完整堆栈
                time.sleep(1)  # 错误恢复等待
    
    
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
        logger.info("机械臂遥操作已停止")
