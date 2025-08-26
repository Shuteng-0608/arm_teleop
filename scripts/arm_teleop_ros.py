import rospy
from arm_teleop.srv import ArmIK, ArmIKRequest
import time
import numpy as np
from threading import Thread
from utils.math_utils import rotation_matrix_to_euler, smooth_values, clip_to_safe_range, track_continuous_angle
from utils.logger import get_logger
logger = get_logger()

# 导入自适应控制器
from arm_control.adaptive_controller import AdaptiveController

class ArmTeleop:
    def __init__(self, vp_streamer, robot_controller, algo_controller=None, config=None):
        """
        初始化机械臂遥控器
        
        参数:
            vp_streamer: VisionPro数据流对象
            robot_controller: 机械臂控制器对象
            algo_controller: 机械臂算法控制器，用于逆运动学计算
            config (dict): 配置参数
        """
        if not rospy.core.is_initialized():
            rospy.init_node('arm_teleop', anonymous=True)

        self.ik_service = rospy.ServiceProxy('inverse_kinematics_service', ArmIK)
        logger.info("已连接到逆运动学服务")

        # 基本组件
        self.vp_streamer = vp_streamer
        self.robot_controller = robot_controller
        self.algo_controller = algo_controller
        
        # 配置参数，如果没有提供则使用默认值
        self.config = config or {}
        
        # 获取机械臂的初始位置
        self.initial_robot_pose = self.robot_controller.current_position

        
        
        # 初始关节角度，用于逆解计算参考
        self.last_joint_angles = None
        if self.algo_controller:
            # 如果提供了算法控制器，尝试计算当前位姿对应的关节角度
            # success, joint_angles = self.algo_controller.inverse_kinematics(self.initial_robot_pose)
            rospy.wait_for_service('inverse_kinematics_service')
            ik_request = ArmIKRequest()
            # ik_request.target_pose = smooth_target.tolist()  # 目标笛卡尔位姿
            ik_request.initial_joints = self.initial_robot_pose
            response = self.ik_service.call(ik_request)
            success = response.success
            joint_angles = response.joint_angles


            if success:
                self.last_joint_angles = joint_angles
                logger.info(f"初始关节角度: {self.last_joint_angles}")
            else:
                logger.warning("无法获取初始关节角度，使用算法控制器默认值")
                self.last_joint_angles = self.algo_controller.last_joint_angles
        
        # 设置安全操作范围
        self.x_range = self.config.get('x_range')  # 前后
        self.y_range = self.config.get('y_range')  # 左右
        self.z_range = self.config.get('z_range')  # 上下
        
        # 旋转角度安全范围
        self.rx_range = self.config.get('rx_range')  # rx旋转范围
        self.ry_range = self.config.get('ry_range')  # ry旋转范围
        self.rz_range = self.config.get('rz_range')  # rz旋转范围
        
        # 如果配置中没有明确指定旋转范围，但需要限制旋转，可以设置默认值
        if self.rx_range is None:
            rx_init = self.initial_robot_pose[3]
            self.rx_range = (rx_init , rx_init)  # 约±10度
        if self.ry_range is None:    
            ry_init = self.initial_robot_pose[4]
            self.ry_range = (ry_init , ry_init)  # 约±10度
        if self.rz_range is None:    
            rz_init = self.initial_robot_pose[5]
            self.rz_range = (rz_init , rz_init)  # 约±10度
            
        logger.info(f"旋转范围已限制: RX: {self.rx_range}, RY: {self.ry_range}, RZ: {self.rz_range}")
        
        # 控制参数
        self.running = False
        self.update_frequency = self.config.get('update_frequency', 0.01)  # 更新频率 (秒)
        self.control_thread = None
        self.scaling_factor = self.config.get('scaling_factor', 1.0)  # 手部运动到机械臂运动的缩放因子
        
        # 平滑过滤参数
        self.smoothing_factor = self.config.get('smoothing_factor', 0.5)  # 值越大，平滑效果越强(0-1)
        self.last_target_pose = self.initial_robot_pose.copy()
        self.position_buffer = []
        
        # 添加关节平滑相关参数
        self.joints_smoothing_factor = self.config.get('joints_smoothing_factor', 0.5)  # 关节平滑系数
        self.last_smooth_joints = self.last_joint_angles.copy() if self.last_joint_angles is not None else None
        self.joints_buffer = []
        
        # 初始化自适应控制器
        self.use_adaptive_control = self.config.get('use_adaptive_control', False)
        if self.use_adaptive_control:
            adaptive_config = self.config.get('adaptive_config', {})
            adaptive_config['leftright'] = self.config.get('leftright', 'right')
            self.adaptive_controller = AdaptiveController(adaptive_config)
            logger.info("已启用上下文感知自适应控制")
        else:
            self.adaptive_controller = None
            logger.info("未启用自适应控制，使用固定控制参数")
        self.teleop_active = True  # 默认不激活遥操作
        logger.info("遥操作初始化完成，等待手势激活...")
        # 校准手部位置
        self.calibrate_hand_position()
        
    def calibrate_hand_position(self):
        """校准手部位置和姿态，记录初始位置作为参考点"""
        # 等待获取有效的手部数据
        max_attempts = 10
        attempts = 0
        
        logger.info("开始校准手部位置...")
        
        while attempts < max_attempts:
            hand_data = self.vp_streamer.get_hand_position(hand=self.config.get('leftright', 'right'))
            if hand_data is not None and len(hand_data) > 0:
                # 记录右手腕初始位置和姿态
                self.initial_hand_transform = hand_data[0]
                self.initial_hand_position = self.initial_hand_transform[:3, 3]
                self.initial_hand_rotation = self.initial_hand_transform[:3, :3]
                logger.info(f"已校准手部位置: {self.initial_hand_position}")
                logger.info(f"已校准手部姿态: {rotation_matrix_to_euler(self.initial_hand_rotation)}")
                return
            
            time.sleep(0.5)
            attempts += 1
            logger.info(f"等待手部数据... {attempts}/{max_attempts}")
            
        logger.info("警告: 无法获取手部位置进行校准！使用默认值。")
        self.initial_hand_position = np.array([0, 0, 0])
        self.initial_hand_rotation = np.eye(3)  # 单位矩阵作为默认旋转

    def map_hand_to_robot(self, hand_transform):
        """
        将手部位置和旋转映射到机械臂位置和姿态
        
        参数:
            hand_transform: 4x4 变换矩阵，包含位置和旋转信息
        """
        # 提取手部位置
        hand_position = hand_transform[:3, 3]
        
        # 如果启用了自适应控制，更新运动历史并获取自适应参数
        if self.adaptive_controller:
            self.adaptive_controller.update_motion_history(hand_position)
            # 获取当前最新的VisionPro数据，用于手势检测
            hand_data = self.vp_streamer.latest
            adaptive_params = self.adaptive_controller.get_adaptive_parameters(hand_data)
            
            # 动态更新控制参数
            current_scaling = adaptive_params["scaling_factor"]
            current_smoothing = adaptive_params["smoothing_factor"]
            
            # 仅在发生变化时记录日志，避免日志过多
            if hasattr(self, 'last_adaptive_params'):
                if self.last_adaptive_params["mode"] != adaptive_params["mode"]:
                    logger.info(f"自适应模式: {adaptive_params['mode']}, 平滑: {current_smoothing:.2f}, 缩放: {current_scaling:.2f}")
            else:
                logger.info(f"初始自适应模式: {adaptive_params['mode']}, 平滑: {current_smoothing:.2f}, 缩放: {current_scaling:.2f}")
                
            self.last_adaptive_params = adaptive_params
            
            # 更新控制参数
            self.smoothing_factor = current_smoothing
            self.scaling_factor = current_scaling
        else:
            pass
        
        # 计算手部位置相对于初始位置的偏移
        hand_offset = hand_position - self.initial_hand_position
        
        # 将偏移应用到机械臂初始位置
        target_position = self.initial_robot_pose.copy()
        
        # 调整位置偏移方向和缩放
        target_position[0] += hand_offset[0] * self.scaling_factor
        target_position[1] += hand_offset[1] * self.scaling_factor
        target_position[2] += hand_offset[2] * self.scaling_factor
        
        # 从变换矩阵中提取旋转信息，转为欧拉角
        rotation_matrix = hand_transform[:3, :3]
        
        # 计算相对于初始手部姿态的旋转变化
        # 相对旋转 = 当前旋转 × 初始旋转的逆
        relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation)
        
        # 转换为欧拉角
        euler_angles = rotation_matrix_to_euler(relative_rotation)
        
        # 应用旋转到机械臂目标姿态
        # 先计算新的角度值
        if self.config.get('leftright', 'right') == 'left':
            new_rx = self.initial_robot_pose[3] + euler_angles[1]
        else:
            new_rx = self.initial_robot_pose[3] - euler_angles[1]  # rx
        
        if self.config.get('leftright', 'right') == 'left':
            new_ry = self.initial_robot_pose[4] - euler_angles[0]
        else:
            new_ry = self.initial_robot_pose[4] + euler_angles[0]  # ry
        new_rz = self.initial_robot_pose[5] + euler_angles[2]  # rz
        
        # 应用连续角度跟踪，防止角度跳变
        # if hasattr(self, 'last_target_pose'):
        #     new_rx = track_continuous_angle(new_rx, self.last_target_pose[3])
        #     new_ry = track_continuous_angle(new_ry, self.last_target_pose[4])
        #     new_rz = track_continuous_angle(new_rz, self.last_target_pose[5])
        
        target_position[3] = new_rx
        target_position[4] = new_ry
        target_position[5] = new_rz
        
        # 转换成 numpy 数组
        target_position = np.array(target_position)
        
        # 确保在安全范围内
        return clip_to_safe_range(
            target_position, 
            self.x_range, 
            self.y_range, 
            self.z_range,
            self.rx_range,
            self.ry_range,
            self.rz_range
        )

    def control_loop(self):
        """控制循环，持续更新机械臂位置和姿态"""
        logger.info("开始机械臂控制循环，等待手势激活遥操作...")
        
        
        # 添加 FPS 计算相关变量
        frame_count = 0
        last_fps_time = time.time()
        while self.running:
            try:
                # 增加帧计数
                frame_count += 1
                current_time = time.time()
                
                # 每秒计算并显示一次 FPS
                if current_time - last_fps_time >= 1.0:
                    fps = frame_count / (current_time - last_fps_time)
                    logger.info(f"遥操作 FPS: {fps:.2f}")
                    frame_count = 0
                    last_fps_time = current_time
                
                # 获取最新的手部数据
                hand_data = self.vp_streamer.latest
                if hand_data is not None and len(hand_data) > 0:
                    # 检查手势控制
                    if self.use_adaptive_control and self.adaptive_controller:
                        gesture = self.adaptive_controller.process_mode_gesture(hand_data)
                        # logger.info(f"当前手势是：{gesture}")
                        
                        # 处理手势控制
                        if gesture == "start_teleop" and not self.teleop_active:
                            self.teleop_active = True
                            logger.info("检测到开始手势，遥操作已激活！")
                            time.sleep(self.update_frequency)
                            continue  # 跳过本次循环，避免立即移动
                        
                        elif gesture == "stop_teleop" and self.teleop_active:
                            self.teleop_active = False
                            logger.info("检测到停止手势，遥操作已停止！")
                            time.sleep(self.update_frequency)
                            continue  # 跳过本次循环
                    
                hand_data = self.vp_streamer.get_hand_position(hand=self.config.get('leftright', 'right'))
                # 只有当遥操作激活时才执行控制
                if self.teleop_active:
                    # 提取右手腕的完整变换矩阵
                    hand_transform = hand_data[0]
                    
                    # 映射到机械臂位置和姿态
                    target_pose = self.map_hand_to_robot(hand_transform) # TODO
                    # logger.info(f'目标位置: {[round(x, 4) for x in target_pose]}')
                    
                    # 应用平滑过滤到位置
                    smooth_target, self.position_buffer = smooth_values(
                        target_pose, 
                        self.last_target_pose, 
                        self.position_buffer, 
                        self.smoothing_factor
                    )
                    self.last_target_pose = smooth_target.copy()
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
                    
                    # 使用逆运动学计算关节角度
                    if self.algo_controller:
                        # 使用目标笛卡尔位姿和当前关节角度作为参考计算逆解
                        # success, joint_angles = self.algo_controller.inverse_kinematics(
                        # success, joint_angles = self.algo_controller.enhanced_inverse_kinematics(
                        #     smooth_target, 
                        #     initial_joints=self.last_joint_angles
                        # )
                        # TODO: call 逆解服务
                        ik_request = ArmIKRequest()
                        ik_request.target_pose = smooth_target.tolist()  # 目标笛卡尔位姿
                        ik_request.initial_joints = self.last_joint_angles.tolist() if self.last_joint_angles is not None else []
                        response = self.ik_service.call(ik_request)
                        success = response.success
                        joint_angles = response.joint_angles


                        # 转成numpy数组
                        joint_angles = np.array(joint_angles)
                        
                        if success:
                            # 更新最后使用的关节角度
                            self.last_joint_angles = joint_angles
                            
                            # 对计算的关节角度进行平滑处理
                            if self.last_smooth_joints is not None:
                                smooth_joint_angles, self.joints_buffer = smooth_values(
                                    joint_angles,
                                    self.last_smooth_joints,
                                    self.joints_buffer,
                                    self.joints_smoothing_factor
                                )
                                # 更新上一次的平滑关节角度
                                self.last_smooth_joints = smooth_joint_angles.copy()
                            else:
                                smooth_joint_angles = joint_angles
                                self.last_smooth_joints = joint_angles.copy()
                            
                            # 使用平滑后的关节角度控制机械臂移动
                            logger.debug(f"关节角度控制: {[round(angle, 4) for angle in smooth_joint_angles]}")
                            if self.config.get('move', True):
                                # 如果配置中允许移动，则使用关节控制
                                self.robot_controller.movej_follow(smooth_joint_angles)
                        else:
                            logger.warning(f"逆解失败，无法控制到位置: {smooth_target}")
                    else:
                        # 如果没有提供算法控制器，回退到使用笛卡尔位姿控制
                        logger.debug(f"使用笛卡尔控制 (algo_controller不可用): {smooth_target}")
                        if self.config.get('move', True):
                            # 如果配置中允许移动，则使用笛卡尔控制
                            self.robot_controller.movep_follow(smooth_target)
                
                # 等待一段时间再更新
                time.sleep(self.update_frequency)
                
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
        logger.info("机械臂遥操作已停止")
