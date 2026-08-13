import sys
import os
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
except ImportError:
    logger.error("错误: 无法导入avp_stream模块，请确保它已正确安装")
    sys.exit(1)

class VPStreamer:
    """VisionPro数据流处理类，扩展原始的VisionProStreamer"""
    
    def __init__(self, ip, record=False, video_enabled=True):
        """
        初始化VisionPro数据流
        
        参数:
            ip (str): VisionPro的IP地址
            record (bool): 是否录制数据
            video_enabled (bool): 是否将本地相机视频通过WebRTC发送到VisionPro
        """
        try:
            self.streamer = AVPStreamer(ip, record=record)
            if video_enabled:
                # self.streamer.configure_video(device="/dev/video6", format="v4l2", size="640x400", fps=30) # head color camera
                self.streamer.configure_video(device="/dev/video14", format="v4l2", size="640x400", fps=30) # breast color camera
                self.streamer.start_webrtc()
            logger.info(f"成功连接到VisionPro: {ip}")
        except Exception as e:
            logger.error(f"连接VisionPro失败: {e}")
            raise
    
    @property
    def latest(self):
        """获取最新的数据帧"""
        return self.streamer.latest
    
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
        if key in data:
            # logger.info(f"{hand}手腕位置: {data[key]}")
            return data[key]
        return None
    
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
        if key in data:
            return data[key]
        return None
    
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
        if key in data:
            return data[key]
        return None
    
if __name__ == "__main__":
    # 测试VPStreamer类
    streamer = VPStreamer("192.168.8.145",False)
    # streamer = VPStreamer("10.32.205.7",False)
    time.sleep(1)
    while True:
        head_data = streamer.get_head_data()
        if head_data is not None:
            print(f"头部位置: {head_data}")
        else:
            print("未获取到头部位置数据")
        # hand_data = streamer.get_hand_position("right")
        # if hand_data is not None:
        #     print(f"右手腕位置: {hand_data}")
        # else:
        #     print("未获取到右手腕位置数据")
        
        # fingers_data = streamer.get_fingers_data("right")
        # if fingers_data is not None:
        #     print(f"右手指关节位置: {fingers_data}")
        # else:
        #     print("未获取到右手指关节位置数据")
        
        time.sleep(3)
