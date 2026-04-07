# import numpy as np
# from scipy.spatial.transform import Rotation as R
# def euler_to_quaternion(euler_pose, rotation_order='XYZ', degrees=False):
#         """
#         将欧拉角形式的6维位姿转换为四元数形式的7维位姿
        
#         参数:
#             euler_pose: 欧拉角位姿 [x, y, z, rx, ry, rz]
#             rotation_order: 欧拉角的旋转顺序，默认为 'xyz'
#             degrees: 欧拉角的单位，False为弧度(默认)，True为度
        
#         返回:
#             quaternion_pose: 四元数位姿 [x, y, z, qw, qx, qy, qz]
#         """
#         # 确保输入是 numpy 数组
#         euler_pose = np.array(euler_pose)
        
#         # 分离位置和旋转
#         position = euler_pose[:3]  # [x, y, z]
#         euler_angles = euler_pose[3:]  # [rx, ry, rz]
#         print(f"输入欧拉角位姿: {euler_angles}")

        
#         # 使用 SciPy 创建旋转对象
#         rotation = R.from_euler(rotation_order, euler_angles, degrees=degrees)
#         print(f"旋转矩阵: \n{rotation.as_matrix()}")
        
#         # 转换为四元数 (SciPy 返回 [x, y, z, w] 格式)
#         quat_scipy = rotation.as_quat()  # 返回 [qx, qy, qz, qw]
        
#         # 转换为 [qw, qx, qy, qz] 格式
#         quaternion = [quat_scipy[3], quat_scipy[0], quat_scipy[1], quat_scipy[2]]
#         print(f"四元数: {quaternion}")
        
#         # 组合位置和四元数
#         quaternion_pose = np.concatenate([position, quaternion])
        
#         return quaternion_pose
# def euler_xyz_list_to_rotation_matrix(euler_angles):
#     """
#     将XYZ顺序的欧拉角列表(弧度制)转换为旋转矩阵
    
#     参数:
#     euler_angles: 包含三个欧拉角的列表 [alpha, beta, gamma]
#                   alpha: 绕X轴的旋转角度(弧度)
#                   beta:  绕Y轴的旋转角度(弧度)
#                   gamma: 绕Z轴的旋转角度(弧度)
    
#     返回:
#     3x3旋转矩阵
#     """
#     if len(euler_angles) != 3:
#         raise ValueError("欧拉角列表必须包含3个元素")
    
#     X, Y, Z = euler_angles  # 明确使用大写X,Y,Z表示绕对应轴的旋转
    
#     # 预计算三角函数值
#     cx, sx = np.cos(X), np.sin(X)  # X轴
#     cy, sy = np.cos(Y), np.sin(Y)  # Y轴
#     cz, sz = np.cos(Z), np.sin(Z)  # Z轴
    
#     # 计算XYZ顺序的旋转矩阵: R = Rz(Z) * Ry(Y) * Rx(X)
#     R = np.array([
#         [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
#         [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
#         [-sy, sx * cy, cx * cy]
#     ])
    
#     return R


# euler_angles = [3.1923149, -0.036102, -0.0007987]  # [alpha, beta, gamma] 弧度
# # R = euler_xyz_list_to_rotation_matrix(euler_angles)
# rotation = R.from_euler("XYZ", euler_angles, degrees=False)
# print("单个欧拉角列表转换结果:")
# # print(rotation)
# print(rotation.as_matrix())
# print(euler_xyz_list_to_rotation_matrix(euler_angles))


# #   [Left]FK验证 - 期望位姿:  0.955659  0.227893  0.186496  0.346824
# #  0.270326   -0.9301 -0.248672 -0.307363
# #  0.116789  0.288061 -0.950464 -0.331754
# #         0         0         0         1
# #   [Left]FK验证 - FK位姿:   0.948652 -0.0576655   -0.31102   0.278812
# #  -0.126664   0.831722   -0.54055    -0.3866
# #   0.289853   0.552189   0.781711  -0.271155
# #          0          0          0          1


# # 示例2: 验证旋转矩阵性质
# # print(f"\n旋转矩阵行列式: {np.linalg.det(R):.6f} (应该接近1)")
# # print(f"R * R^T 是否接近单位矩阵: {np.allclose(R @ R.T, np.eye(3))}")

# # print(f"测试欧拉角到四元数的转换: {euler_to_quaternion([0.301, -0.358, -0.333, 3.0905722, 0.0360597, -0.0010818])}")
import numpy as np
from scipy.spatial.transform import Rotation

def rotation_matrix_to_euler_xyz(matrix_4x4):
    """
    将4x4齐次变换矩阵的旋转部分转换为XYZ顺序欧拉角（弧度）
    
    参数:
        matrix_4x4: 4x4齐次变换矩阵
    
    返回:
        euler_angles_xyz: XYZ顺序的欧拉角（弧度）
    """
    # 提取旋转矩阵（前3x3部分）
    rotation_matrix = matrix_4x4[:3, :3]
    
    # 使用scipy将旋转矩阵转换为XYZ顺序欧拉角
    rot = Rotation.from_matrix(rotation_matrix)
    euler_angles_xyz = rot.as_euler('XYZ', degrees=False)
    
    return euler_angles_xyz

# 示例使用
if __name__ == "__main__":
    # 第一个矩阵（期望位姿）
    expected_matrix = np.array([
        [0.955659, 0.227893, 0.186496, 0.346824],
        [0.270326, -0.9301, -0.248672, -0.307363],
        [0.116789, 0.288061, -0.950464, -0.331754],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    # 第二个矩阵（FK位姿）
    fk_matrix = np.array([
        [0.948652, -0.0576655, -0.31102, 0.278812],
        [-0.126664, 0.831722, -0.54055, -0.3866],
        [0.289853, 0.552189, 0.781711, -0.271155],
        [0.0, 0.0, 0.0, 1.0]
    ])
    
    # 转换为欧拉角
    expected_euler = rotation_matrix_to_euler_xyz(expected_matrix)
    fk_euler = rotation_matrix_to_euler_xyz(fk_matrix)
    
    print("期望位姿的欧拉角 (XYZ顺序, 弧度):", expected_euler)
    print("FK位姿的欧拉角 (XYZ顺序, 弧度):", fk_euler)
    
    # 转换为角度制
    expected_euler_deg = np.degrees(expected_euler)
    fk_euler_deg = np.degrees(fk_euler)
    
    print("\n期望位姿的欧拉角 (XYZ顺序, 角度):", expected_euler_deg)
    print("FK位姿的欧拉角 (XYZ顺序, 角度):", fk_euler_deg)

    print("\n期望的位置(XYZ):" , expected_matrix[0][3],expected_matrix[1][3],expected_matrix[2][3])
    print("FK的位置(XYZ):" , fk_matrix[0][3],fk_matrix[1][3],fk_matrix[2][3])