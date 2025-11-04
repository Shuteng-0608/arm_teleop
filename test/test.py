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

print(f"测试欧拉角到四元数的转换: {euler_to_quaternion([0.2018, -0.368, -0.0086, 3.2767, -0.1549, -0.656])}")