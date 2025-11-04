import numpy as np
import math
from utils.logger import get_logger

logger = get_logger()

def rotation_matrix_to_euler(R):
    """
    将旋转矩阵转换为欧拉角 (rx, ry, rz)
    
    参数:
        R: 3x3 旋转矩阵
        
    返回:
        list: 欧拉角 [rx, ry, rz]
    """
    # 确保是有效的旋转矩阵
    if abs(np.linalg.det(R) - 1.0) > 1e-6:
        # 如果不是正交矩阵，进行调整
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
    
    # 从旋转矩阵提取欧拉角 (rx, ry, rz)
    # 使用 ZYX 欧拉角顺序 (常见于机器人控制)
    if abs(R[2, 0]) != 1:
        theta1 = -math.asin(R[2, 0])
        theta2 = math.pi - theta1
        
        psi1 = math.atan2(R[2, 1] / math.cos(theta1), R[2, 2] / math.cos(theta1))
        psi2 = math.atan2(R[2, 1] / math.cos(theta2), R[2, 2] / math.cos(theta2))
        
        phi1 = math.atan2(R[1, 0] / math.cos(theta1), R[0, 0] / math.cos(theta1))
        phi2 = math.atan2(R[1, 0] / math.cos(theta2), R[0, 0] / math.cos(theta2))
        
        # 通常选择第一组解
        rx = psi1
        ry = theta1
        rz = phi1
    else:
        # 万向锁情况 (Gimbal lock)
        phi = 0  # 任意值
        if R[2, 0] == -1:
            theta = math.pi/2
            psi = phi + math.atan2(R[0, 1], R[0, 2])
        else:
            theta = -math.pi/2
            psi = -phi + math.atan2(-R[0, 1], -R[0, 2])
            
        rx = psi
        ry = theta
        rz = phi
    
    return [rx, ry, rz]

def clip_to_safe_range(pose, x_range, y_range, z_range, rx_range=None, ry_range=None, rz_range=None):
    """
    确保位置和姿态在安全范围内
    
    参数:
        pose: 位置和姿态列表 [x, y, z, rx, ry, rz]
        x_range: x坐标安全范围 (min, max)
        y_range: y坐标安全范围 (min, max)
        z_range: z坐标安全范围 (min, max)
        rx_range: rx旋转安全范围 (min, max)，如果为None则仅限制在-pi到pi之间
        ry_range: ry旋转安全范围 (min, max)，如果为None则仅限制在-pi到pi之间
        rz_range: rz旋转安全范围 (min, max)，如果为None则仅限制在-pi到pi之间
        
    返回:
        list: 限制在安全范围内的位置和姿态
    """
    # 限制位置
    pose[0] = max(x_range[0], min(x_range[1], pose[0]))
    pose[1] = max(y_range[0], min(y_range[1], pose[1]))
    pose[2] = max(z_range[0], min(z_range[1], pose[2]))
    
    # 首先将旋转角度标准化到-pi到pi之间
    # for i in range(3, 6):
    #     while pose[i] > math.pi:
    #         pose[i] -= 2 * math.pi
    #     while pose[i] < -math.pi:
    #         pose[i] += 2 * math.pi
    
    # 然后根据提供的旋转范围进一步限制
    # rx限制
    if rx_range is not None:
        if pose[3] > 0:
            pose[3] = max(rx_range[0], min(rx_range[1], pose[3]))
        else:
            pose[3] = max(-rx_range[1], min(-rx_range[0], pose[3]))
    
    # ry限制
    if ry_range is not None:
        pose[4] = max(ry_range[0], min(ry_range[1], pose[4]))
    
    # rz限制
    if rz_range is not None:
        pose[5] = max(rz_range[0], min(rz_range[1], pose[5]))
            
    return pose

# def smooth_values(new_values, last_values, buffer=None, smoothing_factor=0.5):
#     """
#     应用平滑过滤，减少抖动，可用于位置、关节角度等任何数值序列
    
#     参数:
#         new_values: 新的数值序列
#         last_values: 上一个数值序列
#         buffer: 数据缓冲区，用于更复杂的滤波
#         smoothing_factor: 平滑系数，值越大平滑效果越强
        
#     返回:
#         tuple: (平滑处理后的数值序列, 更新后的缓冲区)
#     """
#     # 如果缓冲区未初始化，创建一个新的缓冲区
#     if buffer is None:
#         buffer = []
            
#     # 更新缓冲区
#     if len(buffer) == 0:
#         buffer = [new_values.copy() for _ in range(3)]
#     else:
#         buffer.pop(0)
#         buffer.append(new_values.copy())
    
#     # 始终使用动态权重
#     smooth_result = last_values.copy()
    
#     # 根据smoothing_factor动态调整权重
#     # smoothing_factor越大，历史数据权重越高
#     if len(buffer) >= 3:
#         # 使用动态权重代替固定权重
#         weight_new = 1 - smoothing_factor         # 最新数据权重
#         weight_mid = smoothing_factor * 0.4       # 中间数据权重
#         weight_old = smoothing_factor * 0.6       # 最旧数据权重
        
#         for i in range(len(new_values)):
#             smooth_result[i] = (weight_old * buffer[0][i] + 
#                                weight_mid * buffer[1][i] + 
#                                weight_new * buffer[2][i])
#     else:
#         # 保持原有的指数滤波逻辑
#         for i in range(len(new_values)):
#             smooth_result[i] = smoothing_factor * last_values[i] + (1 - smoothing_factor) * new_values[i]
    
#     return smooth_result, buffer

def smooth_values(new_values, last_values, buffer=None, smoothing_factor=0.5, history_length=3):
    # history 30 每秒发30， 3 每秒发 40 在有log情况下
    # history 30 每秒发70， 3 每秒发 70 在无log情况下
    """
    应用平滑过滤，减少抖动，可用于位置、关节角度等任何数值序列
    
    参数:
        new_values: 新的数值序列
        last_values: 上一个数值序列
        buffer: 数据缓冲区，可以比history_length大
        smoothing_factor: 平滑系数，值越大平滑效果越强
        history_length: 计算平滑值时使用的历史数据点数量
         
    返回:
        tuple: (平滑处理后的数值序列, 更新后的缓冲区)
    """
    # 如果缓冲区未初始化，创建一个新的缓冲区
    if buffer is None:
        buffer = []
            
    # 更新缓冲区
    buffer.append(np.array(new_values))
    
    # 保持缓冲区不超过一个合理的最大值
    max_buffer_size = 100
    if len(buffer) > max_buffer_size:
        buffer = buffer[-max_buffer_size:]
    
    # 如果只有一个数据点，直接返回它
    if len(buffer) == 1:
        return buffer[0], buffer
    
    # 确定实际使用的历史长度
    actual_history = min(len(buffer), history_length)
    
    # 获取最近的历史数据
    history_data = buffer[-actual_history:]
    
    # 计算指数衰减权重 - 越新的数据权重越大
    weights = []
    weight_sum = 0
    
    # 首先计算未归一化的权重
    for i in range(actual_history):
        # i=0是最老的数据，i=actual_history-1是最新的数据
        # 权重随着数据越新而增加
        weight = (1 - smoothing_factor) ** (actual_history - 1 - i)
        weights.append(weight)
        weight_sum += weight
    
    # 归一化权重使总和为1
    normalized_weights = [w / weight_sum for w in weights]
    # logger.info(f"平滑系数: {smoothing_factor}, 历史长度: {history_length}")
    # logger.info(f"平滑权重: {normalized_weights}")
    # logger.info(f"历史数据: {history_data}")
    
    # 应用加权平均
    smooth_result = np.zeros_like(new_values, dtype=float)
    for i in range(actual_history):
        smooth_result += normalized_weights[i] * history_data[i]
    
    return smooth_result, buffer

# 保留原函数作为兼容性包装器
def smooth_position(new_position, last_position, position_buffer=None, smoothing_factor=0.5):
    """
    应用平滑过滤，减少抖动
    
    参数:
        new_position: 新的目标位置
        last_position: 上一个目标位置
        position_buffer: 位置缓冲区，用于更复杂的滤波
        smoothing_factor: 平滑系数，值越大平滑效果越强
        
    返回:
        list: 平滑处理后的位置
    """
    result, _ = smooth_values(new_position, last_position, position_buffer, smoothing_factor)
    return result


def track_continuous_angle(new_angle, last_angle):
    """
    跟踪角度变化，确保连续性，解决角度环绕问题
    
    参数:
        new_angle: 当前计算的角度
        last_angle: 上一次使用的角度
        
    返回:
        float: 调整后的连续角度
    """
    # 标准化角度到[-pi, pi]范围
    while new_angle > math.pi:
        new_angle -= 2 * math.pi
    while new_angle < -math.pi:
        new_angle += 2 * math.pi
    
    # 检测角度是否发生了跳变(通常跳变幅度接近2π)
    if abs(new_angle - last_angle) > 5.0:  # 如果角度变化太大(接近π)
        diff = (new_angle - last_angle) % (2 * math.pi)
        if diff > math.pi:
            diff -= 2 * math.pi
        # 调整新角度，保持连续性
        new_angle = last_angle + diff
    elif abs(new_angle - last_angle) > 1.5:  # 较小但仍可能是跳变的情况
        if new_angle * last_angle < 0 and abs(abs(new_angle) + abs(last_angle) - 2*math.pi) < 0.5:
            # 这可能是一个从+π到-π或从-π到+π的跳变
            if new_angle < 0:  # 从+π跳到-π
                new_angle += 2 * math.pi
            else:  # 从-π跳到+π
                new_angle -= 2 * math.pi
    
    return new_angle