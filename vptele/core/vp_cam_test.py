from avp_stream import VisionProStreamer
import time
import math
import numpy as np

def matrix_to_euler_angles(matrix):
    """从4x4齐次变换矩阵中提取欧拉角（弧度）"""
    # 提取旋转矩阵部分（3x3）
    R = matrix[:3, :3]
    
    # 计算欧拉角 (ZYX顺序)
    sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        x = math.atan2(R[2,1], R[2,2])  # 绕X轴旋转 (俯仰)
        y = math.atan2(-R[2,0], sy)     # 绕Y轴旋转 (倾斜)
        z = math.atan2(R[1,0], R[0,0])  # 绕Z轴旋转 (偏航)
    else:
        x = math.atan2(-R[1,2], R[1,1]) # 绕X轴旋转
        y = math.atan2(-R[2,0], sy)     # 绕Y轴旋转
        z = 0                           # 绕Z轴旋转
    
    return np.array([x, y, z])

def extract_robot_control_info(pose_matrix):
    """
    从齐次变换矩阵中提取机器人控制信息
    返回: [x_position, y_position, z_position, roll_x, pitch_y, yaw_z]
    """
    # 提取位置信息 (平移部分)
    position = pose_matrix[:3, 3]
    x_pos, y_pos, z_pos = position
    
    # 提取欧拉角 (旋转部分)
    euler_angles = matrix_to_euler_angles(pose_matrix)
    roll_x, pitch_y, yaw_z = euler_angles
    
    return {
        'position': {
            'x': x_pos,
            'y': y_pos,  # 前进方向
            'z': z_pos   # 上下方向
        },
        'rotation': {
            'roll_x': roll_x,   # 绕X轴旋转 (机器人头部俯仰)
            'pitch_y': pitch_y, # 绕Y轴旋转 (机器人头部倾斜)
            'yaw_z': yaw_z      # 绕Z轴旋转 (机器人头部偏航)
        }
    }

def matrix_inverse(matrix):
    """计算4x4齐次变换矩阵的逆矩阵"""
    R = matrix[:3, :3]  # 旋转矩阵
    t = matrix[:3, 3]   # 平移向量
    
    # 逆旋转矩阵是转置
    R_inv = R.T
    # 逆平移向量
    t_inv = -R_inv @ t
    
    # 构造逆矩阵
    inv_matrix = np.eye(4)
    inv_matrix[:3, :3] = R_inv
    inv_matrix[:3, 3] = t_inv
    
    return inv_matrix

def relative_transform(current_matrix, initial_matrix):
    """计算相对于初始姿态的变换矩阵"""
    initial_inv = matrix_inverse(initial_matrix)
    return initial_inv @ current_matrix

# 主程序
avp_ip = "192.168.8.145"  # Vision Pro IP (shown in the app)
s = VisionProStreamer(ip=avp_ip)

# Configure video streaming from robot camera 
# "/dev/video6" is the color camera on the head
s.configure_video(device="/dev/video6", format="v4l2", size="640x400", fps=30)
s.start_webrtc()

print("正在等待获取初始姿态数据...")
initial_pose_set = False
initial_pose_matrix = None
initial_time = time.time()

while True:
    r = s.get_latest()
    head_pose_raw = r['head']
    
    # 将列表转换为numpy数组以便处理
    current_pose_matrix = np.array(head_pose_raw).reshape(4, 4)
    
    if not initial_pose_set:
        # 设置初始姿态
        initial_pose_matrix = current_pose_matrix.copy()
        initial_pose_set = True
        
        # 提取初始姿态信息
        initial_info = extract_robot_control_info(initial_pose_matrix)
        
        print(f"初始姿态已设置 (耗时: {time.time()-initial_time:.2f}s)")
        print(f"初始位置 -> X: {initial_info['position']['x']:.3f}, "
              f"Y: {initial_info['position']['y']:.3f}, "
              f"Z: {initial_info['position']['z']:.3f}")
        print(f"初始旋转 -> Roll_X: {math.degrees(initial_info['rotation']['roll_x']):.2f}°, "
              f"Pitch_Y: {math.degrees(initial_info['rotation']['pitch_y']):.2f}°, "
              f"Yaw_Z: {math.degrees(initial_info['rotation']['yaw_z']):.2f}°")
        print("-" * 80)
        continue
    
    # 计算相对于初始姿态的变换
    relative_matrix = relative_transform(current_pose_matrix, initial_pose_matrix)
    
    # 从相对变换矩阵中提取变化信息
    relative_info = extract_robot_control_info(relative_matrix)
    
    # 打印相对变化信息
    rel_pos = relative_info['position']
    rel_rot = relative_info['rotation']
    
    print(f"时间戳: {time.time():.2f}")
    print(f"相对位置变化 -> ΔX(向右): {rel_pos['x']:.3f}, ΔY(前进): {rel_pos['y']:.3f}, ΔZ(上下): {rel_pos['z']:.3f}")
    print(f"相对旋转变化 -> ΔRoll_X(头部俯仰): {math.degrees(rel_rot['roll_x']):.2f}°, "
          f"ΔPitch_Y(头部倾斜): {math.degrees(rel_rot['pitch_y']):.2f}°, "
          f"ΔYaw_Z(头部偏航): {math.degrees(rel_rot['yaw_z']):.2f}°")
    
    # 根据题目要求，提取关键控制信息（相对于初始姿态的变化）
    forward_change = rel_pos['y']  # Y方向变化为机器人底盘前进变化
    head_yaw_change = rel_rot['yaw_z']      # 绕Z轴旋转变化为机器人头部偏航变化
    head_pitch_change = rel_rot['roll_x']   # 绕X轴旋转变化为机器人头部俯仰变化
    
    print(f"机器人控制指令 -> 底盘前进变化: {forward_change:.3f}, "
          f"头部偏航变化: {math.degrees(head_yaw_change):.2f}°, "
          f"头部俯仰变化: {math.degrees(head_pitch_change):.2f}°")
    
    print("-" * 80)
    
    time.sleep(1.0)
