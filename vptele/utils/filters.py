import numpy as np
import time

class LowPassFilter:
    """
    基础低通滤波器，作为 1 Euro Filter 的内部组件。
    """
    def __init__(self, alpha):
        self.__set_alpha(alpha)
        self.y = None
        self.s = None

    def __set_alpha(self, alpha):
        alpha = float(alpha)
        if alpha <= 0 or alpha > 1.0:
            raise ValueError("alpha must be in (0, 1]")
        self.alpha = alpha

    def filter(self, value, alpha=None):
        if alpha is not None:
            self.__set_alpha(alpha)
        
        if self.y is None:
            s = value
        else:
            s = self.alpha * value + (1.0 - self.alpha) * self.s
        
        self.y = value
        self.s = s
        return s

class OneEuroFilter:
    """
    1 Euro Filter 实现。
    适用于平滑实时数据，能在低速时去抖动，高速时低延迟。
    """
    def __init__(self, t0, x0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        """
        初始化参数:
        :param t0: 初始时间戳
        :param x0: 初始值 (可以是 float 或 numpy array)
        :param min_cutoff: 最小截止频率 (Hz)。值越小，静止时越平滑，延迟越高。
        :param beta: 速度系数。值越大，运动时跟随越紧（延迟越低），但可能引入噪音。
        :param d_cutoff: 导数(速度)的截止频率 (Hz)，通常设为 1.0 即可。
        """
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        
        self.x_prev = np.array(x0, dtype=float)
        self.dx_prev = np.zeros_like(self.x_prev)
        self.t_prev = float(t0)
        self.dx_filter = LowPassFilter(self.alpha(self.d_cutoff, 1.0)) # dt 初始化值不重要

    def alpha(self, cutoff, dt):
        """计算平滑系数 alpha"""
        if dt <= 0: return 1.0
        r = 2 * np.pi * cutoff * dt
        return r / (r + 1)

    def __call__(self, t, x):
        """
        执行滤波
        :param t: 当前时间戳 (秒)
        :param x: 当前数值 (可以是标量或numpy数组)
        :return: 平滑后的数值
        """
        x = np.array(x, dtype=float)
        dt = t - self.t_prev
        
        # 避免时间倒流或重复帧导致的除零
        if dt <= 0:
            return self.x_prev

        # 1. 计算信号变化率（速度），并低通滤波
        dx = (x - self.x_prev) / dt
        dx_smoothed = self.dx_filter.filter(dx, self.alpha(self.d_cutoff, dt))

        # 2. 动态计算截止频率 (cutoff = min_cutoff + beta * |speed|)
        # 速度越快 -> cutoff 越高 -> 滤波越弱 -> 延迟越低
        cutoff = self.min_cutoff + self.beta * np.abs(dx_smoothed)

        # 3. 计算当前的平滑系数 alpha
        a = self.alpha(cutoff, dt)

        # 4. 执行滤波 (支持 numpy广播)
        x_filtered = a * x + (1.0 - a) * self.x_prev

        # 更新历史
        self.x_prev = x_filtered
        self.t_prev = t
        
        return x_filtered

class PoseFilter7D:
    """
    专门针对 7维位姿 [x, y, z, q1, q2, q3, q4] 的封装类。
    包含：
    1. 1 Euro Filter 平滑
    2. 四元数双倍覆盖检测 (Antipodal Check)
    3. 四元数归一化 (Normalization)
    """
    def __init__(self, min_cutoff=0.1, beta=0.05):
        """
        :param min_cutoff: 静态平滑力度 (建议 0.1~1.0)
        :param beta: 动态响应速度 (建议 0.01~0.1)
        """
        self.first_frame = True
        self.one_euro = None 
        
        self.cfg_min_cutoff = min_cutoff
        self.cfg_beta = beta
        
    def process(self, raw_pose, timestamp=None):
        """
        :param raw_pose: 7维数组/列表 [pos_x, pos_y, pos_z, qw, qx, qy, qz] (顺序要保持一致即可)
        :param timestamp: 当前时间戳，不传则自动获取
        :return: 平滑后的 numpy 数组
        """
        if timestamp is None:
            timestamp = time.time()
            
        raw_pose = np.array(raw_pose, dtype=float)

        # 第一帧初始化
        if self.first_frame:
            self.one_euro = OneEuroFilter(timestamp, raw_pose, 
                                          min_cutoff=self.cfg_min_cutoff, 
                                          beta=self.cfg_beta)
            self.first_frame = False
            return raw_pose

        # --- 四元数双倍覆盖处理 (Antipodal Check) ---
        # 假设后4位是四元数
        prev_q = self.one_euro.x_prev[3:] 
        curr_q = raw_pose[3:]             
        
        # 如果点积为负，说明到了对侧，需要翻转符号以保持连续性
        if np.dot(prev_q, curr_q) < 0:
            raw_pose[3:] = -curr_q 

        # --- 执行 1 Euro Filter ---
        smooth_pose = self.one_euro(timestamp, raw_pose)

        # --- 四元数归一化 ---
        # 线性滤波会破坏四元数的单位模长特性，必须归一化
        q_smooth = smooth_pose[3:]
        q_norm = np.linalg.norm(q_smooth)
        if q_norm > 0:
            q_smooth = q_smooth / q_norm
        smooth_pose[3:] = q_smooth

        return smooth_pose
