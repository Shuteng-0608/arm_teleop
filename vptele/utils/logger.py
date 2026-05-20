import os
import logging
import datetime
from logging.handlers import RotatingFileHandler

# 日志级别定义
LOG_LEVELS = {
    'debug': logging.DEBUG,
    'info': logging.INFO,
    'warning': logging.WARNING,
    'error': logging.ERROR,
    'critical': logging.CRITICAL
}

# 创建一个自定义的Logger类，用于跟踪调用栈
class CallerAwareLogger(logging.Logger):
    def findCaller(self, stack_info=False, stacklevel=1):
        """
        重写findCaller方法，跳过无关的堆栈帧
        """
        # 增加堆栈级别，跳过日志处理相关的堆栈帧
        return super().findCaller(stack_info, stacklevel + 2)

# 注册自定义的Logger类
logging.setLoggerClass(CallerAwareLogger)

class Logger:
    _instances = {}  # 修改为字典，允许不同前缀的多个实例
    
    @staticmethod
    def get_instance(prefix=''):
        """单例模式获取日志实例，允许不同前缀有不同实例"""
        if prefix not in Logger._instances:
            Logger(prefix)
        return Logger._instances[prefix]
    
    def __init__(self, prefix=''):
        """初始化日志系统"""
        if prefix in Logger._instances:
            raise Exception(f"Logger类是单例模式，请使用get_instance('{prefix}')获取实例")
        
        Logger._instances[prefix] = self
        self.prefix = prefix
        self.logger_name = f'vptele{f"_{prefix}" if prefix else ""}'
        self.logger = logging.getLogger(self.logger_name)
        self.logger.setLevel(logging.DEBUG)  # 设置最低日志级别
        
        # 确保日志不会传递到根记录器
        self.logger.propagate = False
        
        # 清除可能存在的任何处理器
        if self.logger.handlers:
            for handler in self.logger.handlers:
                self.logger.removeHandler(handler)
        
        self.handlers = {}  # 用于存储不同的处理器
        
        # 创建日志目录
        self.log_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs')
        os.makedirs(self.log_dir, exist_ok=True)
    
    def setup_handlers(self, console_level='info', file_level='debug', 
                     max_file_size=100*1024*1024, backup_count=5):
        """设置日志处理器"""
        # 转换日志级别字符串为对应的常量
        if isinstance(console_level, str):
            console_level = LOG_LEVELS.get(console_level.lower(), logging.INFO)
        
        if isinstance(file_level, str):
            file_level = LOG_LEVELS.get(file_level.lower(), logging.DEBUG)
            
        # 创建控制台处理器
        self.add_console_handler(console_level)
        
        # 创建日期子文件夹
        current_date = datetime.datetime.now().strftime('%Y-%m-%d')
        date_log_dir = os.path.join(self.log_dir, current_date)
        os.makedirs(date_log_dir, exist_ok=True)
        
        # 创建文件处理器
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        # 在日志文件名中添加前缀
        prefix_str = f"{self.prefix}_" if self.prefix else ""
        log_file = os.path.join(date_log_dir, f'vptele_{prefix_str}{timestamp}.log')
        self.add_file_handler(log_file, file_level, max_file_size, backup_count)
    
    def add_console_handler(self, level=logging.INFO):
        """
        添加控制台日志处理器
        
        参数:
            level: 日志级别，默认INFO
        """
        if 'console' not in self.handlers:
            # 创建控制台处理器
            console_handler = logging.StreamHandler()
            console_handler.setLevel(level)
            
            # 设置格式 - 简洁格式，不包含文件路径和行号
            # 在日志消息中添加前缀标识
            prefix_str = f"[{self.prefix}] " if self.prefix else ""
            formatter = logging.Formatter(
                f'%(asctime)s [%(levelname)s] {prefix_str}%(message)s',
                datefmt='%H:%M:%S'
            )
            console_handler.setFormatter(formatter)
            
            # 添加到日志器
            self.logger.addHandler(console_handler)
            self.handlers['console'] = console_handler
        else:
            self.handlers['console'].setLevel(level)
    
    def add_file_handler(self, log_file, level=logging.DEBUG, max_size=100*1024*1024, backup_count=5):
        """
        添加文件日志处理器
        
        参数:
            log_file: 日志文件路径
            level: 日志级别，默认DEBUG
            max_size: 单个日志文件最大大小，默认100MB
            backup_count: 保留的日志文件数量，默认5个
        """
        if 'file' not in self.handlers:
            # 创建文件处理器
            file_handler = RotatingFileHandler(
                log_file,
                maxBytes=max_size,
                backupCount=backup_count,
                encoding='utf-8'
            )
            file_handler.setLevel(level)
            
            # 设置格式 - 使用findCaller找到真正的调用代码位置
            # 在日志消息中添加前缀标识
            prefix_str = f"[{self.prefix}] " if self.prefix else ""
            formatter = logging.Formatter(
                f'%(asctime)s [%(levelname)s] {prefix_str}%(pathname)s:%(lineno)d - %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S'
            )
            file_handler.setFormatter(formatter)
            
            # 添加到日志器
            self.logger.addHandler(file_handler)
            self.handlers['file'] = file_handler
        else:
            self.handlers['file'].setLevel(level)
    
    def set_level(self, level):
        """
        设置日志级别
        
        参数:
            level: 可以是字符串('debug','info','warning','error','critical')
                  或者logging的级别常量
        """
        if isinstance(level, str):
            level = level.lower()
            if level in LOG_LEVELS:
                level = LOG_LEVELS[level]
            else:
                level = logging.INFO
        
        self.logger.setLevel(level)

    def debug(self, msg, *args, **kwargs):
        self.logger.debug(msg, *args, **kwargs)
        
    def info(self, msg, *args, **kwargs):
        self.logger.info(msg, *args, **kwargs)
        
    def warning(self, msg, *args, **kwargs):
        self.logger.warning(msg, *args, **kwargs)
        
    def error(self, msg, *args, **kwargs):
        self.logger.error(msg, *args, **kwargs)
        
    def critical(self, msg, *args, **kwargs):
        self.logger.critical(msg, *args, **kwargs)
        
    def exception(self, msg, *args, exc_info=True, **kwargs):
        kwargs['exc_info'] = exc_info
        self.logger.error(msg, *args, **kwargs)

# 创建一个包装器，确保返回的是Logger实例而不是logger对象
class LoggerWrapper:
    _instances = {}  # 修改为字典，支持不同前缀的实例
    
    @staticmethod
    def get_instance(prefix=''):
        if prefix not in LoggerWrapper._instances:
            LoggerWrapper._instances[prefix] = Logger.get_instance(prefix)
        return LoggerWrapper._instances[prefix]

# 初始化全局日志系统
def setup_logger(console_level='info', file_level='debug', 
               max_file_size=100*1024*1024, backup_count=5, prefix=''):
    """
    设置全局日志系统
    
    Args:
        console_level: 控制台日志级别
        file_level: 文件日志级别
        max_file_size: 日志文件最大大小
        backup_count: 备份文件数量
        prefix: 日志前缀，用于区分不同的机械臂
    """
    logger = Logger.get_instance(prefix)
    logger.setup_handlers(console_level, file_level, max_file_size, backup_count)
    return logger

# 创建一个方便的函数来获取日志对象
def get_logger(prefix=''):
    """
    获取日志对象
    
    Args:
        prefix: 日志前缀，用于区分不同的机械臂
    """
    # print(LoggerWrapper._instances)
    keys = list(LoggerWrapper._instances.keys())
    # print(f'key: {keys}')
    if keys:
        prefix = keys[-1]
    else:
        pass
    return LoggerWrapper.get_instance(prefix)