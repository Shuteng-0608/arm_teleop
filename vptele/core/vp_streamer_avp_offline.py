import sys
import os
import time
import threading
import numpy as np
import csv

try:
    from utils.logger import get_logger
except ImportError:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from utils.logger import get_logger
    
logger = get_logger()

class VPStreamer:
    """VisionPro离线数据流回放类 (基于 CSV 录制文件)"""
    
    def __init__(self, csv_path, loop=False):
        """
        初始化 CSV 回放数据流
        
        参数:
            csv_path (str): CSV 记录文件的绝对或相对路径
            loop (bool): 播放完毕后是否循环播放
        """
        self.csv_path = csv_path
        self.loop = loop
        self.latest = None
        self.running = False
        self.frames = []
        
        # 1. 预加载并解析 CSV 数据
        self._load_csv_data()
        
        # 2. 自动启动回放线程 (与原版逻辑保持一致)
        self.start()

    def _load_csv_data(self):
        """将 CSV 中的扁平数据还原为 4x4 矩阵的帧列表"""
        logger.info(f"正在加载离线 CSV 数据: {self.csv_path}")
        try:
            with open(self.csv_path, 'r', encoding='utf-8') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    frame = {'timestamp': float(row['timestamp'])}
                    
                    # 还原右手的 4x4 矩阵
                    rmat = np.zeros((4, 4))
                    lmat = np.zeros((4, 4))
                    for i in range(4):
                        for j in range(4):
                            rmat[i, j] = float(row[f'right_raw_matrix_{i}{j}'])
                            lmat[i, j] = float(row[f'left_raw_matrix_{i}{j}'])
                            
                    frame['right_matrix'] = rmat
                    frame['left_matrix'] = lmat
                    self.frames.append(frame)
                    
            logger.info(f"✅ 成功加载 {len(self.frames)} 帧轨迹数据。")
        except Exception as e:
            logger.error(f"❌ 加载 CSV 文件失败: {e}")
            sys.exit(1)

    def start(self):
        """启动后台回放线程"""
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._update_loop)
            self.thread.daemon = True
            self.thread.start()
            logger.info("▶️ CSV 轨迹回放线程已启动")

    def stop(self):
        """停止数据回放"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        logger.info("⏹️ CSV 轨迹回放已停止")

    def _update_loop(self):
        """后台循环：模拟真实时间间隔抛出数据"""
        while self.running:
            num_frames = len(self.frames)
            for idx in range(num_frames):
                if not self.running:
                    break
                
                frame = self.frames[idx]
                
                # 更新 self.latest 字典，严格对齐原版数据结构 
                # (原版控制器通过 hand_data[0] 提取矩阵，因此这里包装在列表里)
                self.latest = {
                    "right_wrist": [frame['right_matrix']],
                    "left_wrist": [frame['left_matrix']]
                }
                
                # 计算并休眠到下一帧的时间，模拟 20Hz 的真实发送频率
                if idx < num_frames - 1:
                    time_to_wait = self.frames[idx+1]['timestamp'] - frame['timestamp']
                    if time_to_wait > 0:
                        time.sleep(time_to_wait)
                        
            if not self.loop:
                logger.info("🏁 CSV 播放结束。")
                break

    def get_hand_position(self, hand="right"):
        """获取手腕数据 (兼容原版接口)"""
        data = self.latest
        if data is None:
            return None
            
        key = f"{hand}_wrist"
        if key in data:
            return data[key]
        return None
    
    def get_fingers_data(self, hand="right"):
        """离线模式未记录手指数据，始终返回None"""
        return None
    
    def get_head_data(self):
        """离线模式未记录头部数据，始终返回None"""
        return None

if __name__ == "__main__":
    # 测试离线回放类
    csv_test_path = r"/home/pangu/pangu/src/arm_teleop/data_log/vp_raw_record_20260630_215724.csv" # 请替换为你实际的 CSV 文件名
    streamer = VPStreamer(csv_path=csv_test_path, loop=True)
    
    time.sleep(1)
    
    print("开始测试读取离线双臂数据流... (按 Ctrl+C 停止)")
    try:
        while True:
            # 同时获取左右手数据
            hand_data_r = streamer.get_hand_position("right")
            hand_data_l = streamer.get_hand_position("left")
            
            if hand_data_r is not None and hand_data_l is not None:
                # 提取 4x4 矩阵中的平移向量 (X, Y, Z) 用于显示
                pos_r = hand_data_r[0][:3, 3]
                pos_l = hand_data_l[0][:3, 3]
                
                # 在同一行动态刷新打印左右手坐标
                sys.stdout.write(f"\r👉 右手XYZ: [{pos_r[0]:>6.3f}, {pos_r[1]:>6.3f}, {pos_r[2]:>6.3f}]  |  👈 左手XYZ: [{pos_l[0]:>6.3f}, {pos_l[1]:>6.3f}, {pos_l[2]:>6.3f}]")
                sys.stdout.flush()
                
            time.sleep(0.033) # 模拟 30Hz 刷新率
            
    except KeyboardInterrupt:
        print("\n测试结束。")
        streamer.stop()