import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class RobotVisualizer:
    """机械臂位置和姿态可视化工具"""
    
    def __init__(self):
        """初始化可视化器"""
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.position_history = []
        self.max_history = 100  # 历史轨迹点数量
        
    def update(self, position, orientation=None):
        """
        更新机械臂位置和姿态可视化
        
        参数:
            position: [x, y, z]
            orientation: [rx, ry, rz] 或 旋转矩阵
        """
        # 记录位置历史
        self.position_history.append(position[:3])
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)
        
        # 清除当前图像
        self.ax.clear()
        
        # 绘制历史轨迹
        history = np.array(self.position_history)
        self.ax.plot(history[:, 0], history[:, 1], history[:, 2], 'b-', alpha=0.5)
        
        # 绘制当前位置
        self.ax.scatter(position[0], position[1], position[2], c='r', marker='o', s=100)
        
        # 如果提供了姿态，绘制坐标轴箭头
        if orientation is not None:
            self._draw_orientation(position[:3], orientation)
        
        # 设置图表属性
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        self.ax.set_title('机械臂位置和姿态可视化')
        
        # 设置固定的坐标轴范围，使图像更稳定
        self.ax.set_xlim([-0.6, -0.2])
        self.ax.set_ylim([-0.3, 0.3])
        self.ax.set_zlim([0.2, 0.6])
        
        plt.draw()
        plt.pause(0.01)
        
    def _draw_orientation(self, position, orientation):
        """绘制姿态箭头"""
        if isinstance(orientation, list) and len(orientation) == 3:
            # 如果是欧拉角，转换为旋转矩阵
            rx, ry, rz = orientation
            # 这里简化处理，实际应用中应使用旋转矩阵转换函数
            # 绘制简单的方向指示
            arrow_length = 0.1
            self.ax.quiver(position[0], position[1], position[2], 
                        arrow_length, 0, 0, color='r')
            self.ax.quiver(position[0], position[1], position[2], 
                        0, arrow_length, 0, color='g')
            self.ax.quiver(position[0], position[1], position[2], 
                        0, 0, arrow_length, color='b')
        elif isinstance(orientation, np.ndarray) and orientation.shape == (3, 3):
            # 如果是旋转矩阵
            arrow_length = 0.1
            x_axis = orientation[:, 0] * arrow_length
            y_axis = orientation[:, 1] * arrow_length
            z_axis = orientation[:, 2] * arrow_length
            
            self.ax.quiver(position[0], position[1], position[2], 
                        x_axis[0], x_axis[1], x_axis[2], color='r')
            self.ax.quiver(position[0], position[1], position[2], 
                        y_axis[0], y_axis[1], y_axis[2], color='g')
            self.ax.quiver(position[0], position[1], position[2], 
                        z_axis[0], z_axis[1], z_axis[2], color='b')
