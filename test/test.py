import numpy as np
from scipy.spatial.transform import Rotation as R
def euler_to_quaternion(euler_pose, rotation_order='XYZ', degrees=False):
        """
        将欧拉角形式的6维位姿转换为四元数形式的7维位姿
        
        参数:
            euler_pose: 欧拉角位姿 [x, y, z, rx, ry, rz]
            rotation_order: 欧拉角的旋转顺序，默认为 'xyz'
            degrees: 欧拉角的单位，False为弧度(默认)，True为度
        
        返回:
            quaternion_pose: 四元数位姿 [x, y, z, qw, qx, qy, qz]
        """
        # 确保输入是 numpy 数组
        euler_pose = np.array(euler_pose)
        
        # 分离位置和旋转
        position = euler_pose[:3]  # [x, y, z]
        euler_angles = euler_pose[3:]  # [rx, ry, rz]
        print(f"输入欧拉角位姿: {euler_angles}")

        
        # 使用 SciPy 创建旋转对象
        rotation = R.from_euler(rotation_order, euler_angles, degrees=degrees)
        print(f"旋转矩阵: \n{rotation.as_matrix()}")
        
        # 转换为四元数 (SciPy 返回 [x, y, z, w] 格式)
        quat_scipy = rotation.as_quat()  # 返回 [qx, qy, qz, qw]
        
        # 转换为 [qw, qx, qy, qz] 格式
        quaternion = [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]
        print(f"四元数: {quaternion}")
        
        # 组合位置和四元数
        quaternion_pose = np.concatenate([position, quaternion])
        
        return quaternion_pose
def euler_xyz_list_to_rotation_matrix(euler_angles):
    """
    将XYZ顺序的欧拉角列表(弧度制)转换为旋转矩阵
    
    参数:
    euler_angles: 包含三个欧拉角的列表 [alpha, beta, gamma]
                  alpha: 绕X轴的旋转角度(弧度)
                  beta:  绕Y轴的旋转角度(弧度)
                  gamma: 绕Z轴的旋转角度(弧度)
    
    返回:
    3x3旋转矩阵
    """
    if len(euler_angles) != 3:
        raise ValueError("欧拉角列表必须包含3个元素")
    
    X, Y, Z = euler_angles  # 明确使用大写X,Y,Z表示绕对应轴的旋转
    
    # 预计算三角函数值
    cx, sx = np.cos(X), np.sin(X)  # X轴
    cy, sy = np.cos(Y), np.sin(Y)  # Y轴
    cz, sz = np.cos(Z), np.sin(Z)  # Z轴
    
    # 计算XYZ顺序的旋转矩阵: R = Rz(Z) * Ry(Y) * Rx(X)
    R = np.array([
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy, sx * cy, cx * cy]
    ])
    
    return R


euler_angles = [3.1923149, -0.036102, -0.0007987]  # [alpha, beta, gamma] 弧度
# R = euler_xyz_list_to_rotation_matrix(euler_angles)
rotation = R.from_euler("XYZ", euler_angles, degrees=False)
print("单个欧拉角列表转换结果:")
# print(rotation)
print(rotation.as_matrix())
print(euler_xyz_list_to_rotation_matrix(euler_angles))

# 示例2: 验证旋转矩阵性质
# print(f"\n旋转矩阵行列式: {np.linalg.det(R):.6f} (应该接近1)")
# print(f"R * R^T 是否接近单位矩阵: {np.allclose(R @ R.T, np.eye(3))}")

# print(f"测试欧拉角到四元数的转换: {euler_to_quaternion([0.301, -0.358, -0.333, 3.0905722, 0.0360597, -0.0010818])}")