import rospy
from arm_teleop.srv import ArmIK, ArmIKRequest
from arm_teleop.srv import PqMovejService, PqMovejServiceRequest
from geometry_msgs.msg import Pose, Point, Quaternion
import time
import numpy as np
from threading import Thread
from utils.math_utils import rotation_matrix_to_euler, smooth_values, clip_to_safe_range, track_continuous_angle
from utils.logger import get_logger
from scipy.spatial.transform import Rotation as R
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
        if not rospy.core.is_initialized():
            rospy.init_node('arm_teleop', anonymous=True)
        
        rospy.wait_for_service('/arm_teleop/arm_ik_srv')
        self.ik_service = rospy.ServiceProxy('/arm_teleop/arm_ik_srv', ArmIK)
        # rospy.wait_for_service('/aris_node/pq_movej_srv')
        # self.pq_movej_service = rospy.ServiceProxy('/aris_node/pq_movej_srv', PqMovejService)

        logger.info("已连接到逆运动学服务")

        # 基本组件
        self.vp_streamer = vp_streamer
        self.robot_controller = robot_controller
        self.current_arm_angle = 0.1
        
        # 配置参数，如果没有提供则使用默认值
        self.config = config or {}
        
        # 获取机械臂的初始位置
        # self.initial_robot_pose = self.robot_controller.get_current_pose()
        # self.initial_robot_pose = [0.009, -0.668, 0.1583, -1.5707963, 1.5707963, 0]  # XYZ + 欧拉角 (弧度)
        # self.last_joint_angles = [0, 0, 0, 0, 0, 0, 0]  # 初始关节角度
        self.initial_robot_pose = [0.3011, -0.3580, 0.2282, 3.1923149, -0.036102, -0.0007987]  # XYZ + 欧拉角 (弧度)3.1923149, -0.036102, -0.0007987
        self.last_joint_angles = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]
        self.last_joint_angles = [round(angle, 4) for angle in self.last_joint_angles]
        self.initial_robot_pose_in_quat = self.euler_to_quaternion(self.initial_robot_pose)
        self.initial_robot_pose_in_quat = [round(angle, 4) for angle in self.initial_robot_pose_in_quat]
        

        
        rospy.loginfo(f"机械臂末端初始位置: {[round(x, 4) for x in self.initial_robot_pose]}")
        # 初始关节角度，用于逆解计算参考
        logger.info("正在获取初始关节角度用于逆运动学计算...")
        # self.last_joint_angles = None
        # ik_request = ArmIKRequest()
        # ik_request.method = 'comb'  # 使用组合方法
        # ik_request.target_pose.position.x = self.initial_robot_pose_in_quat[0]
        # ik_request.target_pose.position.y = self.initial_robot_pose_in_quat[1]
        # ik_request.target_pose.position.z = self.initial_robot_pose_in_quat[2]
        # ik_request.target_pose.orientation.w = self.initial_robot_pose_in_quat[3]
        # ik_request.target_pose.orientation.x = self.initial_robot_pose_in_quat[4]
        # ik_request.target_pose.orientation.y = self.initial_robot_pose_in_quat[5]
        # ik_request.target_pose.orientation.z = self.initial_robot_pose_in_quat[6]
        # ik_request.init_joints = self.last_joint_angles if self.last_joint_angles is not None else [0,0,0,0,0,0,0]
        # rospy.loginfo(f"请求逆解服务，目标位姿: {ik_request.target_pose}, 初始关节: {ik_request.init_joints}")
        # resp = self.ik_service.call(ik_request)
        # success = resp.success
        # joint_angles = resp.solution
        # # rospy.loginfo(f"逆解响应情况: {success}, 关节角度: {[round(angle, 4) for angle in joint_angles]}")
        


        # if success:
        #     self.last_joint_angles = [round(angle, 4) for angle in joint_angles]
        #     rospy.loginfo(f"初始关节角度: {self.last_joint_angles}")
        # else:
        #     rospy.logwarn("解算失败，使用Mujoco模型默认值")
        #     self.last_joint_angles = [0,0,0,0,0,0,0]
        
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
        
        # 控制参数
        self.running = False
        self.update_frequency = self.config.get('update_frequency', 0.01)  # 更新频率 (秒)
        self.control_thread = None
        # self.scaling_factor = self.config.get('scaling_factor', 1.0)  # 手部运动到机械臂运动的缩放因子
        self.scaling_factor = 1.0
        
        # 平滑过滤参数
        self.smoothing_factor = self.config.get('smoothing_factor', 0.5)  # 值越大，平滑效果越强(0-1)
        rospy.loginfo(f"机器人初始位姿: {self.initial_robot_pose}")
        self.last_target_pose = self.initial_robot_pose.copy()
        self.position_buffer = []
        
        # 添加关节平滑相关参数
        self.joints_smoothing_factor = self.config.get('joints_smoothing_factor', 0.5)  # 关节平滑系数
        rospy.loginfo(f"Last joint angles: {self.last_joint_angles}")
        self.last_smooth_joints = self.last_joint_angles.copy() if self.last_joint_angles is not None else None
        self.joints_buffer = []
        
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
        
        # 计算手部位置相对于初始位置的偏移
        hand_offset = hand_position - self.initial_hand_position
        # rospy.loginfo(f"手部偏移: {[round(x, 4) for x in hand_offset]}")
        
        # 将偏移应用到机械臂初始位置
        target_position = self.initial_robot_pose.copy()
        # rospy.loginfo(f"上一次机械臂位置: {[round(x, 4) for x in target_position]}")
        
        # 调整位置偏移方向和缩放
        # target_position[0] += hand_offset[0] * self.scaling_factor
        # target_position[1] += hand_offset[1] * self.scaling_factor
        # target_position[2] += hand_offset[2] * self.scaling_factor
        target_position[0] += hand_offset[1] * 1.5
        target_position[1] += hand_offset[2] * 1.5
        target_position[2] += hand_offset[0] * 1.5
        # rospy.loginfo(f"位置调整后: {[round(x, 4) for x in target_position]}")

        
        # 从变换矩阵中提取旋转信息，转为欧拉角
        rotation_matrix = hand_transform[:3, :3]
        
        # 计算相对于初始手部姿态的旋转变化
        # 相对旋转 = 当前旋转 × 初始旋转的逆
        relative_rotation = rotation_matrix @ np.linalg.inv(self.initial_hand_rotation)
        # rospy.loginfo(f"相对旋转矩阵: \n{relative_rotation}")
        
        # 转换为欧拉角
        euler_angles = rotation_matrix_to_euler(relative_rotation)
        
        # 应用旋转到机械臂目标姿态
        # 先计算新的角度值
        # if self.config.get('leftright', 'right') == 'left':
        #     new_rx = self.initial_robot_pose[3] + euler_angles[1]
        # else:
        #     new_rx = self.initial_robot_pose[3] - euler_angles[1]  # rx
        
        # if self.config.get('leftright', 'right') == 'left':
        #     new_ry = self.initial_robot_pose[4] - euler_angles[0]
        # else:
        #     new_ry = self.initial_robot_pose[4] + euler_angles[0]  # ry
        # new_rz = self.initial_robot_pose[5] + euler_angles[2]  # rz
        if self.config.get('leftright', 'right') == 'left':
            new_rx = self.initial_robot_pose[3] - euler_angles[1]
        else:
            new_rx = self.initial_robot_pose[3] + euler_angles[1]  # rx
        
        if self.config.get('leftright', 'right') == 'left':
            new_ry = self.initial_robot_pose[4] - euler_angles[0]
        else:
            new_ry = self.initial_robot_pose[4] + euler_angles[0]  # ry
        new_rz = self.initial_robot_pose[5] - euler_angles[2]  # rz
        
        
        # 应用连续角度跟踪，防止角度跳变
        # if hasattr(self, 'last_target_pose'):
        #     new_rx = track_continuous_angle(new_rx, self.last_target_pose[3])
        #     new_ry = track_continuous_angle(new_ry, self.last_target_pose[4])
        #     new_rz = track_continuous_angle(new_rz, self.last_target_pose[5])
        
        target_position[3] = new_rx
        target_position[4] = new_ry
        target_position[5] = new_rz
        # target_position[3] = self.initial_robot_pose[3]
        # target_position[4] = self.initial_robot_pose[4]
        # target_position[5] = self.initial_robot_pose[5]
        
        # 转换成 numpy 数组
        target_position = np.array(target_position)
        return target_position
        
        # 确保在安全范围内
        # return clip_to_safe_range(
        #     target_position, 
        #     self.x_range, 
        #     self.y_range, 
        #     self.z_range,
        #     self.rx_range,
        #     self.ry_range,
        #     self.rz_range
        # )
    def quat_to_euler(self, initial_pose_quat):
        # initial_pose_quat = self.robot_controller.get_current_pose()
    
        # 提取四元数并转换为 SciPy 格式 [x, y, z, w]
        quat = initial_pose_quat[3:]  # [qw, qx, qy, qz]
        quat_scipy = [quat[1], quat[2], quat[3], quat[0]]  # 转换为 [x, y, z, w]
        
        # 创建旋转对象并转换为欧拉角
        rotation = R.from_quat(quat_scipy)
        euler_angles = rotation.as_euler('xyz')  # 默认返回弧度
        
        # 组合位姿
        self.initial_robot_pose = np.concatenate([initial_pose_quat[:3], euler_angles])
    
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

    def control_loop(self):
        """控制循环，持续更新机械臂位置和姿态"""
        logger.info("开始机械臂控制循环，等待手势激活遥操作...")
        
        
        # 添加 FPS 计算相关变量
        frame_count = 0
        last_fps_time = time.time()
        max_record_time = 0.0
        recorded_time = 0.0
        while self.running:
            try:
                # 增加帧计数
                frame_count += 1
                current_time = time.time()
                
                # 每秒计算并显示一次 FPS
                # if current_time - last_fps_time >= 1.0:
                fps = frame_count / (current_time - last_fps_time)
                logger.info(f"遥操作 FPS: {fps:.2f}")
                frame_count = 0
                last_fps_time = current_time
                
                # 获取最新的手部数据
                hand_data = self.vp_streamer.latest
                    
                hand_data = self.vp_streamer.get_hand_position(hand=self.config.get('leftright', 'right'))
                # 只有当遥操作激活时才执行控制
                if self.teleop_active:
                    # 提取右手腕的完整变换矩阵
                    hand_transform = hand_data[0]
                    
                    # 映射到机械臂位置和姿态
                    target_pose = self.map_hand_to_robot(hand_transform) # TODO
                    logger.info(f'目标位置: {[round(x, 4) for x in target_pose]}')
                    
                    # 应用平滑过滤到位置
                    smooth_target, self.position_buffer = smooth_values(
                        target_pose, 
                        self.last_target_pose, 
                        self.position_buffer, 
                        self.smoothing_factor
                    )
                    smooth_target = [round(angle, 4) for angle in smooth_target]
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
                    # for angle in [0, 0.1, -0.1, 0.5, -0.5]:
                    # TODO: call 逆解服务
                    start_time = time.time()
                    ik_request = ArmIKRequest()
                    ik_request.method = 'feasible'  # 使用组合方法
                    ik_request.current_arm_angle = self.current_arm_angle
                    ik_request.offset_list = [0, 0.05, -0.05, 0.1, -0.1, 0.15, -0.15, 0.2, -0.2, 0.3, -0.3, 0.4, -0.4, 0.5, -0.5]
                    ik_request.offset_refer = 0.5
                    rospy.loginfo(f"请求逆解服务，目标位姿: {[round(x, 4) for x in smooth_target]}")
                    smooth_target_in_quat = self.euler_to_quaternion(smooth_target)
                    ik_request.target_pose.position.x = smooth_target_in_quat[0]
                    ik_request.target_pose.position.y = smooth_target_in_quat[1]
                    ik_request.target_pose.position.z = smooth_target_in_quat[2]
                    ik_request.target_pose.orientation.w = smooth_target_in_quat[3]
                    ik_request.target_pose.orientation.x = smooth_target_in_quat[4]
                    ik_request.target_pose.orientation.y = smooth_target_in_quat[5]
                    ik_request.target_pose.orientation.z = smooth_target_in_quat[6]
                    ik_request.init_joints = self.last_joint_angles if self.last_joint_angles is not None else []
                    response = self.ik_service.call(ik_request)
                    success = response.success
                    joint_angles = response.solution
                    recorded_time = time.time() - start_time
                    max_record_time = recorded_time if recorded_time > max_record_time else max_record_time 
                    print(f"逆解耗时: {time.time() - start_time:.4f} 秒")
                    print(f"当前最大逆解耗时: {max_record_time:.4f} 秒")
                    

                    # yanzheng
                    # offset = [0.0]*7
                    # offset = [abs(response.solution[i] - self.last_joint_angles[i]) for i in range(7)]
                    # if max(offset) > 1.0:
                    #     rospy.logwarn(f"逆解结果跳变过大，忽略本次结果，偏移量: {[round(x, 4) for x in offset]}")
                    #     success = False
                    # else:
                    #     self.current_arm_angle = response.arm_angle
                    #     break
                    
                    if success:
                        # 更新最后使用的关节角度
                        self.current_arm_angle = response.new_arm_angle
                        rospy.loginfo(f"当前臂角: {self.current_arm_angle}")
                        self.last_joint_angles = [round(angle, 4) for angle in joint_angles]
                        rospy.loginfo(f"逆解成功，关节角度: {[round(angle, 4) for angle in joint_angles]}")
                        
                        # 对计算的关节角度进行平滑处理
                        if self.last_smooth_joints is not None:
                            smooth_joint_angles, self.joints_buffer = smooth_values(
                                joint_angles,
                                self.last_smooth_joints,
                                self.joints_buffer,
                                self.joints_smoothing_factor
                            )
                            # rospy.loginfo(f"平滑关节角度: {[round(angle, 4) for angle in smooth_joint_angles]}")
                            # 更新上一次的平滑关节角度
                            self.last_smooth_joints = smooth_joint_angles.copy()
                        else:
                            smooth_joint_angles = joint_angles
                            self.last_smooth_joints = joint_angles.copy()
                        
                        # 使用平滑后的关节角度控制机械臂移动
                        # logger.debug(f"关节角度控制: {[round(angle, 4) for angle in smooth_joint_angles]}")
                        if self.config.get('move', True):
                            rospy.loginfo(f"移动到关节角度位置: {[round(x, 4) for x in smooth_joint_angles]}")
                            # self.robot_controller.set_arm_positions(smooth_joint_angles + [0.0])
                            # pq_request = PqMovejServiceRequest()
                            # pq_request.arm_id = 1
                            # pq_request.target_pq = smooth_joint_angles
                            # pq_response = self.pq_movej_service.call(pq_request)
                            # if pq_response.response:
                            #     rospy.loginfo("机械臂移动命令已发送")
                            
                    else:
                        rospy.logwarn(f"逆解失败，无法控制到位置: {smooth_target}")
                    
                
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
