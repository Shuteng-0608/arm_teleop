import numpy as np
from teleop.open_television.television import TeleVision
from teleop.open_television.constants import *
from teleop.utils.mat_tool import mat_update, fast_mat_inv

print("I am in thomas hole")

"""
(basis) OpenXR Convention : y up, z back, x right. 
(basis) 新Robot Convention : x right, y front, z up.  
(旧Robot Convention 是: x front, y left, z up)
p.s. Vuer's all raw data follows OpenXR Convention, WORLD coordinate.

under (basis) 新Robot Convention, wrist's initial pose convention:

    # (Left Wrist) XR/AppleVisionPro Convention:
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from index toward pinky.
        - the z-axis pointing from palm toward back of the hand.

    # (Right Wrist) XR/AppleVisionPro Convention:
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from pinky toward index.
        - the z-axis pointing from palm toward back of the hand.
  
    # (Left Wrist URDF) Unitree Convention:
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from palm toward back of the hand.
        - the z-axis pointing from pinky toward index.

    # (Right Wrist URDF) Unitree Convention:
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from back of the hand toward palm. 
        - the z-axis pointing from pinky toward index.
"""

class TeleVisionWrapper:
    def __init__(self, binocular, img_shape, img_shm_name):
        self.tv = TeleVision(binocular, img_shape, img_shm_name)
        
        # 新的坐标系转换矩阵：从OpenXR坐标系(y up, z back, x right)到新Robot坐标系(x right, y front, z up)
        # OpenXR的x轴直接对应新Robot的x轴
        # OpenXR的-z轴对应新Robot的y轴（前向）
        # OpenXR的y轴对应新Robot的z轴（上方）
        self.T_new_robot_openxr = np.array([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])

    def get_data(self):
        # --------------------------------wrist-------------------------------------

        # TeleVision obtains a basis coordinate that is OpenXR Convention
        head_vuer_mat, head_flag = mat_update(const_head_vuer_mat, self.tv.head_matrix.copy())
        left_wrist_vuer_mat, left_wrist_flag = mat_update(const_left_wrist_vuer_mat, self.tv.left_hand.copy())
        right_wrist_vuer_mat, right_wrist_flag = mat_update(const_right_wrist_vuer_mat, self.tv.right_hand.copy())

        # 使用新的坐标系转换矩阵
        # 将OpenXR坐标系下的数据转换为新Robot坐标系
        head_mat = self.T_new_robot_openxr @ head_vuer_mat @ fast_mat_inv(self.T_new_robot_openxr)
        left_wrist_mat = self.T_new_robot_openxr @ left_wrist_vuer_mat @ fast_mat_inv(self.T_new_robot_openxr)
        right_wrist_mat = self.T_new_robot_openxr @ right_wrist_vuer_mat @ fast_mat_inv(self.T_new_robot_openxr)

        # 根据新的坐标系，调整Unitree手腕的变换
        # 注意：需要重新定义这些变换，以下是示例，具体值可能需要根据实际调整
        new_T_to_unitree_left_wrist = np.array([[1, 0, 0, 0],
                                              [0, 0, -1, 0],
                                              [0, 1, 0, 0],
                                              [0, 0, 0, 1]])
        
        new_T_to_unitree_right_wrist = np.array([[1, 0, 0, 0],
                                               [0, 0, 1, 0],
                                               [0, -1, 0, 0],
                                               [0, 0, 0, 1]])

        unitree_left_wrist = left_wrist_mat @ (new_T_to_unitree_left_wrist if left_wrist_flag else np.eye(4))
        unitree_right_wrist = right_wrist_mat @ (new_T_to_unitree_right_wrist if right_wrist_flag else np.eye(4))

        # Transfer from WORLD to HEAD coordinate (translation only).
        unitree_left_wrist[0:3, 3] = unitree_left_wrist[0:3, 3] - head_mat[0:3, 3]
        unitree_right_wrist[0:3, 3] = unitree_right_wrist[0:3, 3] - head_mat[0:3, 3]

        # --------------------------------hand-------------------------------------

        # Homogeneous, [xyz] to [xyz1]
        left_hand_vuer_mat = np.concatenate([self.tv.left_landmarks.copy().T, np.ones((1, self.tv.left_landmarks.shape[0]))])
        right_hand_vuer_mat = np.concatenate([self.tv.right_landmarks.copy().T, np.ones((1, self.tv.right_landmarks.shape[0]))])

        # 使用新的坐标系转换矩阵
        left_hand_mat = self.T_new_robot_openxr @ left_hand_vuer_mat
        right_hand_mat = self.T_new_robot_openxr @ right_hand_vuer_mat

        # Transfer from WORLD to WRIST coordinate. (this process under new Robot Convention)
        left_hand_mat_wb = fast_mat_inv(left_wrist_mat) @ left_hand_mat
        right_hand_mat_wb = fast_mat_inv(right_wrist_mat) @ right_hand_mat

        # 根据新坐标系调整手部变换
        new_T_to_unitree_hand = np.array([[0, 0, 1, 0],
                                        [-1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, 0, 1]])
            
        unitree_left_hand = (new_T_to_unitree_hand @ left_hand_mat_wb)[0:3, :].T
        unitree_right_hand = (new_T_to_unitree_hand @ right_hand_mat_wb)[0:3, :].T

        # --------------------------------offset-------------------------------------

        head_rmat = head_mat[:3, :3]
        
        # 根据新坐标系调整偏移量
        # 在新坐标系下，可能需要不同的偏移值
        # 示例：假设前向轴变了，但高度轴(z)不变
        unitree_left_wrist[0, 3] += 0.15  # x轴(右)偏移
        unitree_right_wrist[0, 3] += 0.15
        unitree_left_wrist[2, 3] += 0.45  # z轴(上)偏移
        unitree_right_wrist[2, 3] += 0.45

        return head_rmat, unitree_left_wrist, unitree_right_wrist, unitree_left_hand, unitree_right_hand