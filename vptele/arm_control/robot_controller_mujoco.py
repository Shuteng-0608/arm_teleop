import numpy as np
import mujoco
import mujoco.viewer
import time
from typing import List, Optional, Dict, Any
import threading
import queue
from scipy.spatial.transform import Rotation

class RobotControllerMuJoCo:
    def __init__(self, model_path: str, config: Optional[dict] = None):
        """
        初始化 MuJoCo 仿真器 - 修复阻尼问题
        """
        self.config = config or {}

        
        # 加载模型
        try:
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            print(f"成功加载模型: {model_path}")
        except Exception as e:
            print(f"加载模型失败: {e}")
            raise RuntimeError(f"无法加载模型: {model_path}")
        
        # 获取关节信息
        self.joint_names = self._get_joint_names()
        print(f"模型包含 {len(self.joint_names)} 个关节: {self.joint_names}")
        
        # 初始化18个关节目标位置
        self.target_joints = [0.0] * len(self.joint_names)
        # self.target_joints[7] = 3.14
        # 预计算执行器映射
        self.actuator_map = self._create_actuator_map()
        
        # 夹爪控制相关
        self.gripper_joints = self.config.get('gripper_joints', [
            "joint_thumb_1", "joint_thumb_2",
            "joint_index_1", "joint_index_2", 
            "joint_middle_1", "joint_middle_2",
            "joint_ring_1", "joint_ring_2",
            "joint_little_1", "joint_little_2"
        ])
        
        # 控制频率
        self.control_rate = self.config.get('control_rate', 100)
        
        # 线程同步和控制
        self.running = False
        self.viewer_running = False
        self.lock = threading.Lock()

        
        print("MuJoCo 仿真器初始化完成")
        print("启动多线程仿真...")
        self.set_arm_positions([-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005])
        self.start_simulation()
    

    def _get_joint_names(self) -> List[str]:
        """获取模型中所有关节的名称"""
        joint_names = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                joint_names.append(name)
        return joint_names
    

    def _create_actuator_map(self) -> dict:
        """创建关节到执行器的映射"""
        actuator_map = {}
        for joint_name in self.joint_names:
            actuator_name = f"motor_{joint_name}"
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            if actuator_id != -1:
                actuator_map[joint_name] = actuator_id
                print(f"关节 '{joint_name}' 映射到执行器 ID: {actuator_id}")
            else:
                actuator_map[joint_name] = None
                print(f"警告: 未找到关节 '{joint_name}' 的执行器")
        return actuator_map       
    

    def get_current_joints(self) -> List[float]:
        """获取当前关节角度"""
        joint_angles = []
        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id != -1:
                qpos_addr = self.model.jnt_qposadr[joint_id]
                if qpos_addr < len(self.data.qpos):
                    joint_angles.append(round(float(self.data.qpos[qpos_addr]), 3))
                else:
                    joint_angles.append(0.0)
            else:
                joint_angles.append(0.0)
        return joint_angles
    

    def update_positions(self, target_joints: List[float]):
        """直接设置关节位置（无控制力计算）"""
        if len(target_joints) != len(self.joint_names):
            print(f"错误: 目标关节数({len(target_joints)})与模型关节数({len(self.joint_names)})不匹配")
            return
        # print(f"更新关节位置: {[f'{x:.3f}' for x in target_joints]}")
        with self.lock:
            for i, joint_name in enumerate(self.joint_names):
                joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                if joint_id != -1:
                    qpos_addr = self.model.jnt_qposadr[joint_id]
                    if qpos_addr < len(self.data.qpos):
                        self.data.qpos[qpos_addr] = target_joints[i]


    def set_arm_positions(self, arm_target_joints: List[float]):
        """
        设置机械臂目标关节位置（前8个关节）
        
        参数:
            arm_target_joints: 机械臂关节目标位置列表，包含8个值
        """
        if len(arm_target_joints) != 7:
            print(f"错误: 机械臂目标关节数({len(arm_target_joints)})应为7个")
            return
        multiplier = [-1, 1, 1, -1, 1, 1, 1]
        arm_target_joints = [target_joints * multiplier for target_joints, multiplier in zip(arm_target_joints, multiplier)]
        
        with self.lock:
            # 更新前7个关节位置，保持后10个手指关节不变
            if len(self.target_joints) == 18:
                self.target_joints[:7] = arm_target_joints.copy()
        
        print(f"设置机械臂关节位置: {[f'{x:.3f}' for x in arm_target_joints]}")


    def set_hand_positions(self, hand_target_joints: List[float]):
        """
        设置灵巧手目标位置（后10个关节）
        
        参数:
            hand_target_joints: 手部关节目标位置列表，包含10个值
        """
        # new_target_joints = [x for x in hand_target_joints for _ in range(2)]
        new_target_joints = []

        # 遍历原列表的每个元素
        for i, value in enumerate(hand_target_joints):
            if i < 2:  # 前两个元素只添加一次
                new_target_joints.append(value)
            else:  # 从第三个元素开始，每个元素添加两次
                new_target_joints.extend([value, value])
        multiplier = [1, 1, -1, 1, -1, 1, -1, 1, -1, 1]
        new_target_joints = [target_joints * multiplier for target_joints, multiplier in zip(new_target_joints, multiplier)]
        if len(new_target_joints) != 10:
            print(f"错误: 手部目标关节数({len(new_target_joints)})应为10个")
            return
        
        with self.lock:
            # 更新后10个手指关节位置，保持前8个机械臂关节不变
            if len(self.target_joints) == 18:
                self.target_joints[8:18] = new_target_joints.copy()
        
        print(f"设置手部关节位置: {[f'{x:.3f}' for x in new_target_joints]}")
    
    
    def visualization_thread(self):
        """可视化线程函数"""
        print("启动可视化仿真线程...")
        # 重置数据
        mujoco.mj_resetData(self.model, self.data)
        
        # 先运行几步确保稳定
        print("初始化仿真...")
        for i in range(10):
            self.update_positions(self.target_joints)
            mujoco.mj_step(self.model, self.data)
            time.sleep(0.01)
        
        # 启动可视化
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                print("可视化已启动，按ESC退出")
                self.viewer_running = True
                # 主仿真循环
                step_count = 0
                while viewer.is_running and self.running:
                    # 位置控制
                    self.update_positions(self.target_joints)
                    # 推进仿真
                    mujoco.mj_step(self.model, self.data)
                    # 更新可视化
                    viewer.sync()
                    step_count += 1
                    # 控制仿真速度
                    time.sleep(0.001)
                
                self.viewer_running = False
                print("可视化窗口关闭，仿真结束")
                
        except Exception as e:
            print(f"可视化仿真错误: {e}")
            import traceback
            traceback.print_exc()
            self.viewer_running = False
    
    
    def start_simulation(self):
        """启动仿真"""
        self.running = True
        # 创建并启动可视化线程
        vis_thread = threading.Thread(target=self.visualization_thread, daemon=True)
        vis_thread.start()
        # 等待可视化启动
        time.sleep(2.0)
        print("仿真已启动，按Ctrl+C停止...")
        
    
    def disconnect(self):
        """停止仿真"""
        self.running = False
        print("仿真停止")
    


# 示例使用
if __name__ == "__main__":
    # 使用更高的控制参数以克服阻尼
    config = {
        'kp': 500.0,        # 大幅增加比例增益
        'kd': 50.0,         # 大幅增加微分增益
        'max_force': 50.0, # 大幅增加最大控制力
        'control_rate': 100,
    }
    
    try:
        print("创建仿真器...")
        # model_path = "/home/stw/pangu/src/arm_teleop/model/Arm_simplified.xml"
        model_path = "/home/pangu/pangu/src/arm_teleop/model/right_arm_stable.xml"
        simulator = RobotControllerMuJoCo(model_path, config)
        time.sleep(2.0)
        
        
        
        # 等待仿真结束
        while simulator.running:
            time.sleep(1.0)
            
        print("仿真完成")
    
    except Exception as e:
        print(f"仿真器错误: {e}")
        import traceback
        traceback.print_exc()