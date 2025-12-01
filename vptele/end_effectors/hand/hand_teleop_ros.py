import numpy as np
from end_effectors.end_effector_base import EndEffectorBase
# from end_effectors.hand.hand_controller_mujoco import HandControllerMujoco
from utils.logger import get_logger
import rospy
from aiui.srv import DH5SetPosition, DH5SetPositionRequest
import time
logger = get_logger()

class HandTeleopROS(EndEffectorBase):
    """灵巧手遥控模块"""
    
    def __init__(self, vp_streamer, robot_controller, config=None):
        """
        初始化灵巧手遥控模块
        
        参数:
            vp_streamer: VisionPro数据流对象
            robot_controller: 机械臂控制器对象
            config (dict): 配置参数
        """
        super().__init__(vp_streamer, robot_controller, config)
        # self.hand_controller = self.robot_controller
        # self.max_hand_range = self.config.get('max_hand_range', 1.57) # TODO RIGHT_HAND
        self.max_hand_range = 1.57
        # 手指位置限制范围 [弯曲, 伸直]，根据实际灵巧手调整
        self.latest_hand_pos_right = [920, 1770, 1707, 1730, 1730, 980]
        # self.position_limits_right = [
        #     [30, 930],
        #     [10, 1771],
        #     [30, 1707],
        #     [30, 1731],
        #     [30, 1731],
        #     [30, 981],
        # ]
        self.position_limits_right = [
            [30, 920],
            [10, 1770],
            [30, 1707],
            [30, 1730],
            [30, 1730],
            [30, 980],
        ]
        self.latest_hand_pos_left = [851, 1683, 1699, 1681, 1683, 867]
        # self.position_limits_left = [
        #     [30, 966],
        #     [30, 1725],
        #     [30, 1686],
        #     [30, 1725],
        #     [30, 1732],
        #     [30, 838],
        # ]
        self.position_limits_left = [ 
            [30, 851],
            [10, 1683],
            [30, 1699],
            [30, 1681],
            [10, 1683],
            [30, 867],
        ]
        self.smoothing_factor = self.config.get('smoothing_factor', 0.6)
        self.last_hand_pos = [0, 0, 0, 0, 0, 0]
        if not rospy.core.is_initialized():
            rospy.init_node('hand_teleop', anonymous=True)
        
        rospy.wait_for_service('/dh5/set_all_position')
        self.dh5_service = rospy.ServiceProxy('/dh5/set_all_position', DH5SetPosition)
        self.initialize()
        
    def initialize(self):
        """初始化灵巧手控制参数和关节映射"""
        # 定义计算弯曲程度所需的关节索引 (指尖, 远端关节, 中间关节, 近端关节)
        self.pinky_joints = (24, 23, 22, 21)     # 小拇指
        self.ring_joints = (19, 18, 17, 16)      # 无名指
        self.middle_joints = (14, 13, 12, 11)    # 中指
        self.index_joints = (9, 8, 7, 6)         # 食指
        self.thumb_joints = (4, 3, 2, 1)         # 大拇指
        
        # 大拇指旋转需要用到的关节
        self.thumb_rot_joints = (4, 1, 0, 5)  # 大拇指尖, 大拇指根部, 手腕, 食指根部
        
    def get_joint_position(self, hand_data, joint_idx, hand_side="right"):
        """
        从手部数据中获取指定关节的位置
        
        参数:
            hand_data: VisionPro提供的手部数据
            joint_idx: 关节索引
            
        返回:
            np.array: 关节的3D位置
        """
        if hand_side == "right":
            if hand_data is None or "right_fingers" not in hand_data:
                return None
                
            # 关节位置在变换矩阵的第4列前3行
            return hand_data["right_fingers"][joint_idx, :3, 3]
        elif hand_side == "left":
            if hand_data is None or "left_fingers" not in hand_data:
                return None
                
            return hand_data["left_fingers"][joint_idx, :3, 3]
        
    def calculate_finger_bend_by_angle(self, hand_data, tip_idx, distal_idx, middle_idx, proximal_idx, hand_side="right"):
        """
        通过关节角度计算手指弯曲程度
        
        参数:
            hand_data: 手部数据
            tip_idx: 指尖关节索引
            distal_idx: 远端指节关节索引
            middle_idx: 中间指节关节索引
            proximal_idx: 近端指节关节索引
            
        返回:
            float: 弯曲程度 (0-1)，0表示完全伸直，1表示完全弯曲
        """
        if hand_side == "right":
            if hand_data is None or "right_fingers" not in hand_data:
                return 0
                
            # 获取各关节位置
            tip_pos = self.get_joint_position(hand_data, tip_idx, hand_side="right")
            distal_pos = self.get_joint_position(hand_data, distal_idx, hand_side="right")
            middle_pos = self.get_joint_position(hand_data, middle_idx, hand_side="right")
            proximal_pos = self.get_joint_position(hand_data, proximal_idx, hand_side="right")
        elif hand_side == "left":
            if hand_data is None or "left_fingers" not in hand_data:
                return 0
                
            # 获取各关节位置
            tip_pos = self.get_joint_position(hand_data, tip_idx, hand_side="left")
            distal_pos = self.get_joint_position(hand_data, distal_idx, hand_side="left")
            middle_pos = self.get_joint_position(hand_data, middle_idx, hand_side="left")
            proximal_pos = self.get_joint_position(hand_data, proximal_idx, hand_side="left")
        
        if tip_pos is None or distal_pos is None or middle_pos is None or proximal_pos is None:
            return 0
            
        # 计算关节向量
        vec1 = tip_pos - distal_pos
        vec2 = distal_pos - middle_pos
        vec3 = middle_pos - proximal_pos
        
        # 将向量标准化
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
        vec3 = vec3 / np.linalg.norm(vec3)
        
        # 计算各关节角度（使用点积求夹角）
        angle1 = np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))
        angle2 = np.arccos(np.clip(np.dot(vec2, vec3), -1.0, 1.0))
        
        # 手指完全伸直时这些角度接近0，完全弯曲时接近π/2或更大
        # 计算平均弯曲角度并归一化到0-1范围
        max_angle = np.pi  # 最大可能角度（180度）
        avg_angle = (angle1 + angle2) / 2
        bend = avg_angle / (max_angle/2)  # 归一化到0-1
        
        # 限制在0-1范围内
        bend = max(0.0, min(1.0, bend))
        
        return bend
    
    def calculate_thumb_bend_by_angle(self, hand_data, hand_side="right"):
        """
        针对大拇指特殊结构的弯曲度计算
        
        参数:
            hand_data: 手部数据
            
        返回:
            float: 弯曲程度 (0-1)，0表示完全伸直，1表示完全弯曲
        """
        if hand_side == "right":
            if hand_data is None or "right_fingers" not in hand_data:
                return 0
                
            # 获取大拇指关节位置
            tip_pos = self.get_joint_position(hand_data, self.thumb_joints[0], hand_side="right")  # 拇指尖(4)
            distal_pos = self.get_joint_position(hand_data, self.thumb_joints[1], hand_side="right")  # 远端指节(3)
            proximal_pos = self.get_joint_position(hand_data, self.thumb_joints[2], hand_side="right")  # 近端指节(2)
            metacarpal_pos = self.get_joint_position(hand_data, self.thumb_joints[3], hand_side="right")  # 掌骨(1)
            wrist_pos = self.get_joint_position(hand_data, 0, hand_side="right")  # 手腕
        elif hand_side == "left":
            if hand_data is None or "left_fingers" not in hand_data:
                return 0
                
            # 获取大拇指关节位置
            tip_pos = self.get_joint_position(hand_data, self.thumb_joints[0], hand_side="left")  # 拇指尖(4)
            distal_pos = self.get_joint_position(hand_data, self.thumb_joints[1], hand_side="left")  # 远端指节(3)
            proximal_pos = self.get_joint_position(hand_data, self.thumb_joints[2], hand_side="left")  # 近端指节(2)
            metacarpal_pos = self.get_joint_position(hand_data, self.thumb_joints[3], hand_side="left")  # 掌骨(1)
            wrist_pos = self.get_joint_position(hand_data, 0, hand_side="left")  # 手腕
        
        if tip_pos is None or distal_pos is None or proximal_pos is None or metacarpal_pos is None or wrist_pos is None:
            return 0
            
        # 计算两个关节角度
        vec1 = tip_pos - distal_pos
        vec2 = distal_pos - proximal_pos
        vec3 = proximal_pos - metacarpal_pos
        vec4 = metacarpal_pos - wrist_pos
        
        # 归一化向量
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
        vec3 = vec3 / np.linalg.norm(vec3)
        vec4 = vec4 / np.linalg.norm(vec4)
        
        # 计算关节角度
        angle1 = np.arccos(np.clip(np.dot(vec1, vec2), -1.0, 1.0))  # 指尖-远端
        angle2 = np.arccos(np.clip(np.dot(vec2, vec3), -1.0, 1.0))  # 远端-近端
        angle3 = np.arccos(np.clip(np.dot(vec3, vec4), -1.0, 1.0))  # 近端-掌骨
        
        # 计算总体弯曲程度，大拇指弯曲时三个关节都会有贡献
        # 大拇指内收角度对弯曲贡献更大
        weighted_angle = (angle1 * 0.05 + angle2 * 0.9 + angle3 * 0.05)
        
        # 归一化到0-1，大拇指完全弯曲时总角度约为π
        max_weighted_angle = np.pi * 0.3  # 根据权重调整
        bend = weighted_angle / max_weighted_angle
        
        # 增强大拇指弯曲灵敏度
        bend = pow(bend, 0.8)  # 使用幂函数增强响应
        
        # 限制在0-1范围内
        bend = max(0.0, min(1, bend))
        
        return bend
    
    def calculate_thumb_rotation(self, hand_data, hand_side="right"):
        """
        计算大拇指旋转角度
        
        参数:
            hand_data: 手部数据
            
        返回:
            float: 旋转程度 (0-1)，0表示大拇指与其他手指在同一平面，1表示形成夹爪状态
        """
        if hand_side == "right":
            if hand_data is None or "right_fingers" not in hand_data:
                return 0
                
            # 获取大拇指尖、大拇指根部、手腕和食指根部的位置
            thumb_tip = self.get_joint_position(hand_data, self.thumb_rot_joints[0], hand_side="right")  # 大拇指尖
            thumb_base = self.get_joint_position(hand_data, self.thumb_rot_joints[1], hand_side="right")  # 大拇指根部
            wrist = self.get_joint_position(hand_data, self.thumb_rot_joints[2], hand_side="right")  # 手腕
            index_base = self.get_joint_position(hand_data, self.thumb_rot_joints[3], hand_side="right")  # 食指根部
        elif hand_side == "left":
            if hand_data is None or "left_fingers" not in hand_data:
                return 0
                
            # 获取大拇指尖、大拇指根部、手腕和食指根部的位置
            thumb_tip = self.get_joint_position(hand_data, self.thumb_rot_joints[0], hand_side="left")  # 大拇指尖
            thumb_base = self.get_joint_position(hand_data, self.thumb_rot_joints[1], hand_side="left")  # 大拇指根部
            wrist = self.get_joint_position(hand_data, self.thumb_rot_joints[2], hand_side="left")  # 手腕
            index_base = self.get_joint_position(hand_data, self.thumb_rot_joints[3], hand_side="left")  # 食指根部
        
        if thumb_tip is None or thumb_base is None or wrist is None or index_base is None:
            return 0
        
        # 创建手掌坐标系
        # Z轴：从手腕指向手掌中心
        palm_center = (thumb_base + index_base) / 2  # 手掌中心近似
        z_axis = palm_center - wrist
        z_axis = z_axis / np.linalg.norm(z_axis)
        
        # X轴：从拇指根部指向小指根部的方向（横向）
        # 这里用食指根部近似代替小指根部，垂直于Z轴
        # 根据左右手调整横向轴方向
        # x_temp = index_base - thumb_base
        if hand_side == "right":
            x_temp = index_base - thumb_base  # 右手：从拇指指向食指
        else:
            x_temp = thumb_base - index_base  # 左手：从食指指向拇指（镜像对称）

        x_axis = x_temp - np.dot(x_temp, z_axis) * z_axis  # 确保垂直于Z轴
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Y轴：垂直于X和Z轴（手掌法向量）
        # y_axis = np.cross(z_axis, x_axis)
        if hand_side == "right":
            y_axis = np.cross(z_axis, x_axis)  # 右手坐标系
        else:
            y_axis = np.cross(x_axis, z_axis)  # 左手坐标系（镜像）
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        # 计算大拇指向量
        thumb_vec = thumb_tip - thumb_base
        thumb_vec = thumb_vec / np.linalg.norm(thumb_vec)
        
        # 计算大拇指在Y-Z平面上的投影（拇指内收程度）
        # 拇指在Y轴的投影越大，表示越靠近夹爪状态
        thumb_y_proj = np.dot(thumb_vec, y_axis)
        
        # 将投影映射到0-1范围
        # 通常大拇指内收时Y轴投影会从负值（完全伸开）变为正值（夹爪状态）
        if hand_side == "right":
            # 右手：负值表示伸展，正值表示内收
            rotation = (thumb_y_proj + 0.7) / 1.4
        else:
            # 左手：符号可能相反，需要测试调整
            rotation = (-thumb_y_proj + 0.7) / 1.4  # 尝试取反
        # rotation = (thumb_y_proj + 0.7) / 1.4
        
        # 限制在0-1范围内
        rotation = max(0.0, min(1.0, rotation))
        
        return rotation
        
    def process_vp_data(self, hand_data, hand_side="right"):
        """
        将VisionPro手部数据映射到灵巧手控制值
        
        参数:
            hand_data: VisionPro提供的手部数据
            
        返回:
            list: 6个元素的灵巧手控制值数组
        """
        if hand_side == "right":
            if hand_data is None or "right_fingers" not in hand_data:
                return self.latest_hand_pos_right
        elif hand_side == "left":
            if hand_data is None or "left_fingers" not in hand_data:
                return self.latest_hand_pos_left
            
        # 使用角度法计算手指弯曲程度 (0-1范围)
        pinky_bend = self.calculate_finger_bend_by_angle(hand_data, *self.pinky_joints, hand_side=hand_side)
        ring_bend = self.calculate_finger_bend_by_angle(hand_data, *self.ring_joints, hand_side=hand_side)
        middle_bend = self.calculate_finger_bend_by_angle(hand_data, *self.middle_joints, hand_side=hand_side)
        index_bend = self.calculate_finger_bend_by_angle(hand_data, *self.index_joints, hand_side=hand_side)
        # 对大拇指使用专门的计算方法
        thumb_bend = self.calculate_thumb_bend_by_angle(hand_data, hand_side=hand_side)
        thumb_rot = self.calculate_thumb_rotation(hand_data, hand_side=hand_side)
        
        # 打印弯曲程度，用于调试
        # logger.info(f"小拇指: {pinky_bend:.2f}, 无名指: {ring_bend:.2f}, 中指: {middle_bend:.2f}, "
            #   f"食指: {index_bend:.2f}, 大拇指弯曲: {thumb_bend:.2f}, 大拇指旋转: {thumb_rot:.2f}")
        
        # 转换为灵巧手控制值范围 (0-65535)
        # RIGHT_HAND
        # pinky_value = pinky_bend * self.max_hand_range
        # ring_value = ring_bend * self.max_hand_range
        # middle_value = middle_bend * self.max_hand_range
        # index_value = index_bend * self.max_hand_range
        # thumb_bend_value = thumb_bend
        # thumb_rot_value = thumb_rot * (1.57)
        
        # 按照灵巧手控制顺序排列
        # hand_pos = [thumb_rot_value, thumb_bend_value, index_value, middle_value, ring_value,
        #             pinky_value]
        if hand_side == "right":
            hand_pos = self.map_finger_values_to_limits(
                thumb_bend, index_bend, middle_bend, ring_bend, pinky_bend, thumb_rot,
                self.position_limits_right
            )
        elif hand_side == "left":
            hand_pos = self.map_finger_values_to_limits(
                thumb_bend, index_bend, middle_bend, ring_bend, pinky_bend, thumb_rot,
                self.position_limits_left
            )
        # hand_pos = self.map_finger_values_to_limits(
        #     thumb_bend, index_bend, middle_bend, ring_bend, pinky_bend, thumb_rot,
        #     self.position_limits_right
        # )
        
        
                    
        return hand_pos
    
    def map_finger_values_to_limits(self, thumb_bend, index_bend, middle_bend, ring_bend, pinky_bend, thumb_rot, position_limits):
        """
        将0-1范围的手指弯曲值映射到指定的位置限制范围
        
        参数:
            thumb_bend, index_bend, middle_bend, ring_bend, pinky_bend, thumb_rot: 0-1范围内的值
            position_limits: 位置限制列表，包含6个范围的[min, max]
        
        返回:
            映射后的整数值列表
        """
        # 计算各手指的映射值
        # thumb_bend_value = thumb_bend * position_limits[0][1] + (1 - thumb_bend) * position_limits[0][0]
        # index_value = index_bend * position_limits[1][1] + (1 - index_bend) * position_limits[1][0]
        # middle_value = middle_bend * position_limits[2][1] + (1 - middle_bend) * position_limits[2][0]
        # ring_value = ring_bend * position_limits[3][1] + (1 - ring_bend) * position_limits[3][0]
        # pinky_value = pinky_bend * position_limits[4][1] + (1 - pinky_bend) * position_limits[4][0]
        # thumb_rot_value = thumb_rot * position_limits[5][1] + (1 - thumb_rot) * position_limits[5][0]
        thumb_bend_value = (1 - thumb_bend) * position_limits[0][1] + thumb_bend * position_limits[0][0]
        index_value = (1 - index_bend) * position_limits[1][1] + index_bend * position_limits[1][0]
        middle_value = (1 - middle_bend) * position_limits[2][1] + middle_bend * position_limits[2][0]
        ring_value = (1 - ring_bend) * position_limits[3][1] + ring_bend * position_limits[3][0]
        pinky_value = (1 - pinky_bend) * position_limits[4][1] + pinky_bend * position_limits[4][0]
        thumb_rot_value = (1 - thumb_rot) * position_limits[5][1] + thumb_rot * position_limits[5][0]
        
        # 转换为整数
        mapped_values = [
            int(round(thumb_bend_value)),
            int(round(index_value)),
            int(round(middle_value)),
            int(round(ring_value)),
            int(round(pinky_value)),
            int(round(thumb_rot_value))
        ]
        
        return mapped_values
    
    def smooth_hand_position(self, new_pos):
        """
        应用平滑过滤，减少抖动
        
        参数:
            new_pos: 新的手部位置数据
            
        返回:
            list: 平滑处理后的手部位置
        """
        # 如果是首次调用，直接返回新位置
        if all(x == 0 for x in self.last_hand_pos):
            self.last_hand_pos = new_pos.copy()
            return new_pos
            
        smooth_pos = []
        for i in range(len(new_pos)):
            # 应用平滑系数
            value = self.smoothing_factor * self.last_hand_pos[i] + (1 - self.smoothing_factor) * new_pos[i]
            smooth_pos.append(value)
            
        self.last_hand_pos = smooth_pos.copy()
        return [int(x) for x in smooth_pos]  # 确保返回整数值
    
    def update(self):
        """更新灵巧手状态"""
        # 获取最新的手部数据
        # update_start_time = time.time()
        # data_get_start_time = time.time()
        hand_data = self.vp_streamer.latest
        # print(f"[hand_teleop] 获取手部数据耗时: {time.time() - data_get_start_time:.4f}秒")
        
        if hand_data is not None and "right_fingers" in hand_data:
            # 将手部数据映射到灵巧手控制值
            # process_start_time = time.time()
            # hand_pos = self.process_vp_data(hand_data)
            self.latest_hand_pos_right = self.process_vp_data(hand_data, hand_side="right")
            # print(f"[hand_teleop] 处理手部数据耗时: {time.time() - process_start_time:.4f}秒")
            
            # 应用平滑过滤
            # smooth_hand_pos = self.smooth_hand_position(hand_pos)
            
            # 控制灵巧手
            # self.robot_controller.set_hand_positions(hand_pos)
            
            # dh5_start_time = time.time()
        if hand_data is not None and "left_fingers" in hand_data:
            self.latest_hand_pos_left = self.process_vp_data(hand_data, hand_side="left")
        rospy.sleep(0.1)

        try: 
            dh5_req = DH5SetPositionRequest()
            dh5_req.hand_type = 'both'
            dh5_req.hand_mode = 'hand'
            dh5_req.right_position_list = self.latest_hand_pos_right
            dh5_req.left_position_list = self.latest_hand_pos_left
            self.dh5_service(dh5_req)
            
        except rospy.ServiceException as e:
            logger.error(f"调用灵巧手服务失败: {e}")
        # logger.info(f"[hand_teleop] 调用灵巧手服务耗时: {time.time() - dh5_start_time:.4f}秒")
        
        # print(f"设置灵巧手位置: {hand_pos}")
        # logger.info(f"[hand_teleop] 灵巧手位置更新耗时: {time.time() - update_start_time:.4f}秒")
