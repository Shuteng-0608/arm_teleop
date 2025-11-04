# 本文件提供日志使用示例，展示如何将logger.info语句替换为日志记录

from utils.logger import get_logger

# 获取日志对象
logger = get_logger()

def example_function():
    # 原始 logger.info 写法
    # logger.info("这是一个普通的信息输出")
    # logger.info(f"当前值是: {some_value}")
    # logger.info("警告：发现潜在问题")
    # logger.info(f"错误：操作失败: {error_message}")
    
    # 替换成日志记录的写法
    logger.info("这是一个普通的信息输出")
    
    some_value = 42
    logger.info(f"当前值是: {some_value}")
    
    logger.warning("警告：发现潜在问题")
    
    error_message = "连接超时"
    logger.error(f"错误：操作失败: {error_message}")
    
    # 记录详细的调试信息
    logger.debug("这是调试信息，默认只会记录到文件而不会显示在控制台")
    
    # 记录严重错误
    logger.critical("严重错误：系统可能无法继续运行")
    
    try:
        # 尝试执行可能引发异常的代码
        result = 1 / 0
    except Exception as e:
        # 记录异常，包括堆栈跟踪信息
        logger.exception(f"发生异常: {e}")

# 不同级别的日志使用指南
def log_level_guide():
    logger.debug("调试级别: 详细的开发信息，用于诊断问题")
    logger.info("信息级别: 确认程序正常工作的信息")
    logger.warning("警告级别: 表示可能出现的问题，程序仍在工作")
    logger.error("错误级别: 由于严重问题，程序无法执行某些功能")
    logger.critical("严重错误级别: 表示严重的错误，程序可能无法继续运行")

# 如何在类中使用日志
class ExampleClass:
    def __init__(self):
        self.logger = get_logger()
        self.logger.info("ExampleClass 已初始化")
    
    def some_method(self):
        self.logger.info("执行了 some_method")
