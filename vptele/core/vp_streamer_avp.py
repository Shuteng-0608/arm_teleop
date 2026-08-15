import sys
import os
from typing import Any, Optional

import numpy as np
try:
    from utils.logger import get_logger
except ImportError:
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from utils.logger import get_logger
logger = get_logger()
import time
# 将上级目录添加到模块搜索路径中，以便导入avp_stream
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

try:
    from avp_stream import VisionProStreamer as AVPStreamer
except ImportError as exc:
    AVPStreamer = None
    _AVP_IMPORT_ERROR = exc
else:
    _AVP_IMPORT_ERROR = None



class VPStreamer:
    """VisionPro数据流处理类，扩展原始的VisionProStreamer"""
    
    def __init__(self, ip, record=False):
        """
        初始化VisionPro数据流
        
        参数:
            ip (str): VisionPro的IP地址
            record (bool): 是否录制数据
        """
        if AVPStreamer is None:
            raise RuntimeError(
                "avp-stream is not installed in the active Python environment"
            ) from _AVP_IMPORT_ERROR
        try:
            self.streamer = AVPStreamer(ip, record=record)
            self._video_stream_started = False
            self._closed = False
            logger.info(f"成功连接到VisionPro: {ip}")
        except Exception as e:
            logger.exception(f"连接VisionPro失败: {e}")
            raise

    def start_video_stream(self, width, height, fps, port=9999):
        """Start the optional mono WebRTC stream used for the operator CCTV."""
        if self._video_stream_started:
            return True

        width = int(width)
        height = int(height)
        fps = int(round(float(fps)))
        port = int(port)
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError("VisionPro视频尺寸和帧率必须为正数")
        if not 1 <= port <= 65535:
            raise ValueError("VisionPro视频端口必须在1到65535之间")

        try:
            self.streamer.configure_video(
                device=None,
                format=None,
                size=f"{width}x{height}",
                fps=fps,
                stereo=False,
            )
            started = bool(
                self.streamer.start_webrtc(port=port, blocking=False)
            )
        except Exception as exc:
            logger.exception(f"VisionPro CCTV WebRTC启动失败: {exc}")
            return False

        self._video_stream_started = started
        if started:
            logger.info(
                f"VisionPro CCTV WebRTC已启动: {width}x{height} "
                f"@ {fps} FPS, port={port}"
            )
        else:
            logger.error("VisionPro CCTV WebRTC启动失败，原遥操作功能继续运行")
        return started

    def update_video_frame(self, frame_bgr):
        """Publish one BGR frame without queueing it in the control process."""
        if not self._video_stream_started or self._closed:
            return False
        self.streamer.update_frame(frame_bgr)
        return True

    def is_video_connected(self):
        """Return whether a WebRTC video client is currently connected."""
        if not self._video_stream_started or self._closed:
            return False
        try:
            return bool(self.streamer.is_connected())
        except Exception:
            return False

    def close(self):
        """Release tracking and optional WebRTC resources exactly once."""
        if self._closed:
            return
        self._closed = True
        self._video_stream_started = False
        try:
            self.streamer.cleanup()
        except Exception as exc:
            logger.exception(f"关闭VisionPro数据流失败: {exc}")
    
    @property
    def latest(self):
        """获取最新的数据帧"""
        getter = getattr(self.streamer, "get_latest", None)
        if callable(getter):
            return getter()
        return getattr(self.streamer, "latest", None)

    def get_latest(self):
        """Return the newest frame for both avp-stream 1.x and 2.x."""
        return self.latest

    @staticmethod
    def _tracking_value(data: Any, key: str) -> Optional[Any]:
        """Read one legacy key from either a dict or TrackingData object."""
        if data is None:
            return None
        getter = getattr(data, "get", None)
        if callable(getter):
            value = getter(key, None)
            if value is not None:
                return value

        if key == "head":
            return getattr(data, "head", None)

        side, _, part = key.partition("_")
        hand = getattr(data, side, None)
        if hand is None:
            return None
        if part == "wrist":
            value = getattr(hand, "wrist", None)
            if value is not None:
                value = np.asarray(value)
                return value[np.newaxis] if value.shape == (4, 4) else value
        return None
    
    def get_hand_position(self, hand="right"):
        """
        获取手部位置数据
        
        参数:
            hand (str): 'right' 或 'left'，指定获取哪只手的数据
            
        返回:
            dict: 手部位置数据，如果不可用则返回None
        """
        data = self.latest
        if data is None:
            return None
            
        key = f"{hand}_wrist"
        return self._tracking_value(data, key)
    
    def get_fingers_data(self, hand="right"):
        """
        获取手指数据
        
        参数:
            hand (str): 'right' 或 'left'，指定获取哪只手的数据
            
        返回:
            np.ndarray: 手指关节位置数据，如果不可用则返回None
        """
        data = self.latest
        if data is None:
            return None
            
        key = f"{hand}_fingers"
        return self._tracking_value(data, key)
    
    def get_head_data(self):
        """
        获取头部位置数据
        
        返回:
            dict: 头部位置数据，如果不可用则返回None
        """
        data = self.latest
        if data is None:
            return None
            
        key = "head"
        value = self._tracking_value(data, key)
        if value is not None and np.asarray(value).shape == (4, 4):
            return np.asarray(value)[np.newaxis]
        return value
    
if __name__ == "__main__":
    # 测试VPStreamer类
    streamer = VPStreamer(ip="192.168.1.112", record=False)
    time.sleep(2)
    while True:
        print("获取右手位置...")
        right_hand_data = streamer.get_hand_position(hand="right")
        if right_hand_data is not None:
            print(f"右手位置: {right_hand_data}")
        else:
            print("未获取到右手位置数据")
        
        time.sleep(3)
