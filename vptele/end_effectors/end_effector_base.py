from abc import ABC, abstractmethod
from threading import Thread
import time
from utils.logger import get_logger
logger = get_logger()

class EndEffectorBase(ABC):
    """所有末端执行器必须实现的接口"""
    
    def __init__(self, vp_streamer, robot_controller, config=None):
        """
        初始化末端执行器基类
        
        参数:
            vp_streamer: VisionPro数据流对象
            robot_controller: 机械臂控制器对象
            config (dict): 配置参数
        """
        self.vp_streamer = vp_streamer
        self.robot_controller = robot_controller
        self.config = config or {}
        self.running = False
        self.control_thread = None
        self.update_frequency = self.config.get('update_frequency', 0.01)  # 更新频率 (秒)
        
    @abstractmethod
    def initialize(self):
        """初始化末端执行器"""
        pass
        
    @abstractmethod
    def process_vp_data(self, vp_data):
        """
        处理VisionPro数据
        
        参数:
            vp_data: VisionPro提供的数据
            
        返回:
            任意: 处理后的控制数据
        """
        pass
        
    @abstractmethod
    def update(self):
        """更新末端执行器状态"""
        pass
        
    def control_loop(self):
        """控制循环，持续更新末端执行器状态"""
        while self.running:
            try:
                self.update()
                time.sleep(self.update_frequency)
            except Exception as e:
                logger.info(f"末端执行器控制循环出错: {str(e)}")
                time.sleep(1)  # 错误恢复等待
    
    def start(self):
        """开始控制"""
        if self.running:
            logger.info(f"{self.__class__.__name__} 已在运行")
            return
            
        logger.info(f"启动 {self.__class__.__name__}...")
        self.running = True
        self.control_thread = Thread(target=self.control_loop, name=f"{self.__class__.__name__}Thread")
        self.control_thread.daemon = True
        self.control_thread.start()
        
    def stop(self):
        """停止控制"""
        if not self.running:
            return
            
        logger.info(f"停止 {self.__class__.__name__}...")
        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=2)
        logger.info(f"{self.__class__.__name__} 已停止")
