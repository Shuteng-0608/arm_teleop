import sys
import os
import requests
import time
import numpy as np

from utils.logger import get_logger
logger = get_logger()

class VPStreamer:
    """VisionPro数据流处理类，通过HTTP API从服务器获取数据"""
    
    def __init__(self, ip="localhost", port=5301, record=False):
        """
        初始化VisionPro数据流客户端
        
        参数:
            ip (str): 数据服务器的IP地址，默认为localhost
            port (int): 数据服务器端口，默认为5301
            record (bool): 是否录制数据（目前仅保留参数，未实现）
        """
        # self.server_url = f"http://{ip}:{port}"
        self.server_url = "http://localhost:5301"
        self.record = record
        self._latest_data = None
        self._last_fetch_time = 0
        self._fetch_interval = 1/300  # 30Hz
        
        try:
            # 测试连接
            response = requests.get(f"{self.server_url}/health", timeout=5)
            response.raise_for_status()
            logger.info(f"成功连接到VisionPro数据服务器: {self.server_url}")
        except Exception as e:
            logger.error(f"连接VisionPro数据服务器失败: {e}")
            raise
    
    @property
    def latest(self):
        """
        获取最新的数据帧，使用适当的频率从服务器获取
        
        返回:
            dict: 包含头部和手部位置数据的字典
        """
        current_time = time.time()
        
        # 限制请求频率，避免过于频繁的API调用
        if current_time - self._last_fetch_time >= self._fetch_interval:
            try:
                response = requests.get(f"{self.server_url}/api/vision_data", timeout=1)
                response.raise_for_status()
                self._latest_data = response.json()
                self._last_fetch_time = current_time
                
                # 将列表数据转换为NumPy数组，保持与原始API兼容
                if self._latest_data.get('head_rmat') is not None:
                    self._latest_data['head_rmat'] = np.array(self._latest_data['head_rmat'])
                if self._latest_data.get('left_wrist') is not None:
                    self._latest_data['left_wrist'] = np.array(self._latest_data['left_wrist'])
                if self._latest_data.get('right_wrist') is not None:
                    self._latest_data['right_wrist'] = np.array(self._latest_data['right_wrist'])
                if self._latest_data.get('left_hand') is not None:
                    self._latest_data['left_fingers'] = np.array(self._latest_data['left_hand']) 
                if self._latest_data.get('right_hand') is not None:
                    self._latest_data['right_fingers'] = np.array(self._latest_data['right_hand'])
                
            except Exception as e:
                logger.warning(f"获取VisionPro数据失败: {e}")
                
        return self._latest_data
    
    def get_hand_position(self, hand="right"):
        """
        获取手部位置数据
        
        参数:
            hand (str): 'right' 或 'left'，指定获取哪只手的数据
            
        返回:
            numpy.ndarray: 手部位置数据，如果不可用则返回None
        """
        data = self.latest
        if data is None:
            return None
            
        key = f"{hand}_wrist"
        if key in data:
            return [data[key]]
        return None
    
    def get_fingers_data(self, hand="right"):
        """
        获取手指数据
        
        参数:
            hand (str): 'right' 或 'left'，指定获取哪只手的数据
            
        返回:
            numpy.ndarray: 手指关节位置数据，如果不可用则返回None
        """
        data = self.latest
        if data is None:
            return None
            
        key = f"{hand}_fingers"
        if key in data:
            return data[key]
        return None

    def get_head_rotation(self):
        """
        获取头部旋转矩阵
        
        返回:
            numpy.ndarray: 头部旋转矩阵，如果不可用则返回None
        """
        data = self.latest
        if data is None or 'head_rmat' not in data:
            return None
        
        return data['head_rmat']