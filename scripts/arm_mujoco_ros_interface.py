#!/usr/bin/env python3
import rospy
import numpy as np
import mujoco
import mujoco.viewer
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import time

class ArmMujocoRosInterface:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('arm_mujoco_ros_interface', anonymous=True)
        
        # 加载您的 MuJoCo 模型
        model_path = 'src/arm_teleop/model/Arm_right_15.xml'  # 请替换为实际路径
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # 定义所有关节名称（根据您的模型）
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7',
            'joint_hand', 'joint_thumb_1', 'joint_thumb_2', 'joint_index_1', 'joint_index_2',
            'joint_middle_1', 'joint_middle_2', 'joint_ring_1', 'joint_ring_2',
            'joint_little_1', 'joint_little_2'
        ]
        
        # 创建关节状态消息
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        self.joint_state_msg.position = [0.0] * len(self.joint_names)
        self.joint_state_msg.velocity = [0.0] * len(self.joint_names)
        self.joint_state_msg.effort = [0.0] * len(self.joint_names)
        
        # ROS 发布者：发布关节状态
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        
        # ROS 订阅者：订阅控制指令
        rospy.Subscriber('/arm/joint_trajectory', JointTrajectory, self.joint_trajectory_callback)
        
        # 可选：单独关节控制
        rospy.Subscriber('/arm/joint_commands', Float64MultiArray, self.joint_commands_callback)
        
        # 设置控制参数
        self.kp = 100.0  # 比例增益
        self.kd = 10.0   # 微分增益
        
        # 目标关节位置
        self.target_positions = np.zeros(len(self.joint_names))
        
        # 设置发布频率 (Hz)
        self.control_rate = 100  # 匹配 MuJoCo 的仿真步长
        self.model.opt.timestep = 0.001  # 仿真步长：0.001 秒
        self.publish_rate = 50   # ROS 消息发布频率
        self.control_interval = int(self.control_rate / self.publish_rate)
        self.control_counter = 0
        
        # 可视化
        self.viewer = None
        if rospy.get_param("~visualize", False):
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            rospy.loginfo("MuJoCo visualization enabled")
        
        rospy.loginfo("Arm MuJoCo-ROS interface initialized")

    def joint_trajectory_callback(self, msg):
        # 处理轨迹消息
        if not msg.points:
            return
            
        # 简单示例：只取轨迹的第一个点
        point = msg.points[0]
        if len(point.positions) == len(self.target_positions):
            self.target_positions = np.array(point.positions)
        else:
            rospy.logwarn("Received trajectory with incorrect number of joints")

    def joint_commands_callback(self, msg):
        # 处理直接关节位置命令
        if len(msg.data) == len(self.target_positions):
            self.target_positions = np.array(msg.data)
        else:
            rospy.logwarn("Received joint commands with incorrect number of joints")

    def run(self):
        rate = rospy.Rate(self.control_rate)
        
        rospy.loginfo("Starting MuJoCo simulation loop")
        
        while not rospy.is_shutdown():
            # 1. 应用 PD 控制
            for i, joint_name in enumerate(self.joint_names):
                # 获取关节在 MuJoCo 中的索引
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id == -1:
                    rospy.logwarn(f"Joint {joint_name} not found in model")
                    continue
                
                # 获取关节的 qpos 地址
                qpos_addr = self.model.jnt_qposadr[joint_id]
                
                # 获取当前位置和速度
                current_pos = self.data.qpos[qpos_addr]
                current_vel = self.data.qvel[qpos_addr] if qpos_addr < len(self.data.qvel) else 0.0
                
                # 计算误差和控制力
                error = self.target_positions[i] - current_pos
                control_force = self.kp * error - self.kd * current_vel
                
                # 构造执行器名称（假设执行器名称是 "motor_" + 关节名称）
                actuator_name = f"motor_{joint_name}"
                
                # 查找执行器ID
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                
                if actuator_id != -1:
                    self.data.ctrl[actuator_id] = control_force
                else:
                    # 如果找不到执行器，尝试使用关节ID直接施加力
                    rospy.logwarn_once(f"Actuator {actuator_name} not found, using qfrc_applied instead")
                    self.data.qfrc_applied[joint_id] = control_force
            
            # 2. 推进仿真
            # 在每个控制循环中推进多个仿真步
            steps_per_control = int(1.0 / (self.control_rate * self.model.opt.timestep))
            for _ in range(steps_per_control):
                # 应用控制（可以在这里更新控制力）
                mujoco.mj_step(self.model, self.data)
            # mujoco.mj_step(self.model, self.data)
            
            # 3. 更新可视化
            if self.viewer is not None:
                self.viewer.sync()
            
            # 4. 发布关节状态（以较低频率）
            self.control_counter += 1
            if self.control_counter >= self.control_interval:
                self.control_counter = 0
                
                # 更新关节状态消息
                current_time = rospy.Time.now()
                self.joint_state_msg.header.stamp = current_time
                
                for i, joint_name in enumerate(self.joint_names):
                    joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                    if joint_id == -1:
                        continue
                    
                    qpos_addr = self.model.jnt_qposadr[joint_id]
                    
                    self.joint_state_msg.position[i] = self.data.qpos[qpos_addr]
                    if qpos_addr < len(self.data.qvel):
                        self.joint_state_msg.velocity[i] = self.data.qvel[qpos_addr]
                    # 注意：effort 需要从传感器或执行器获取，这里简化处理
                
                # 发布消息
                self.joint_state_pub.publish(self.joint_state_msg)
            
            # 5. 保持循环频率
            rate.sleep()

if __name__ == '__main__':
    try:
        interface = ArmMujocoRosInterface()
        interface.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Arm MuJoCo-ROS interface shutdown")