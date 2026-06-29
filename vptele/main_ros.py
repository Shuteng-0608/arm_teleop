#!/usr/bin/env python3

import rospy
import argparse
import yaml
import os
import sys
from utils.logger import get_logger, setup_logger
from arm_teleop.srv import StartDualTeleOP, StartDualTeleOPRequest
import time
import termios
import tty
import select

def run_teleop_system(config_path, vp_ip=None, robot_ip=None, end_effector=None, 
                      log_level=None, process_name=None, mode="full"):
    """
    初始化并运行机械臂遥操控系统
    
    Args:
        config_path (str): 配置文件路径
        vp_ip (str, optional): 覆盖配置文件中的VisionPro IP
        robot_ip (str, optional): 覆盖配置文件中的机械臂IP
        end_effector (str, optional): 覆盖配置文件中的末端执行器类型
        log_level (str, optional): 日志级别
        process_name (str, optional): 进程名称，用于多进程环境中区分日志
        mode (str, optional): 初始化模式，"full"或"playback"
        
    Returns:
        TeleopSystem: 遥操控系统实例
    """
    # 构建配置文件的绝对路径
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), config_path)
    
    # 加载配置文件
    try:
        with open(config_path, 'r', encoding="utf-8") as f:
            config = yaml.safe_load(f)
    except Exception as e:
        error_msg = f"无法加载配置文件 {config_path}: {e}"
        rospy.logerr(error_msg)
        raise RuntimeError(error_msg)
    
    # 设置日志系统
    log_config = config.get('logging', {})
    console_level = log_level or log_config.get('console_level', 'info')
    file_level = log_config.get('file_level', 'debug')
    max_file_size = log_config.get('max_file_size', 100 * 1024 * 1024)
    backup_count = log_config.get('backup_count', 5)
    
    # 如果提供了进程名称，添加到日志文件名中, 否则取ip地址的最后一位，例如169.254.128.19取19
    log_prefix = f"{process_name}" if process_name else f"{config['robot_ip'].split('.')[-1]}"
    
    setup_logger(console_level, file_level, max_file_size, backup_count, prefix=log_prefix)
    logger = get_logger(log_prefix)
    
    logger.info(f"正在启动VisionPro机械臂遥操控系统... 配置文件: {config_path}")
    
    # 命令行参数覆盖配置文件
    vp_ip = "192.168.8.145"
    if vp_ip:
        config['vp_ip'] = vp_ip
    # if robot_ip:
    #     config['robot_ip'] = robot_ip
    if end_effector:
        config['end_effector'] = end_effector
    
    # 验证必要的配置项
    if 'vp_ip' not in config:
        logger.error("错误: 未指定VisionPro IP地址")
        raise ValueError("未指定VisionPro IP地址")
    # if 'robot_ip' not in config:
    #     logger.error("错误: 未指定机械臂IP地址")
    #     raise ValueError("未指定机械臂IP地址")
    
    # 打印配置信息
    logger.info(f"VisionPro IP: {config['vp_ip']}")
    # logger.info(f"机械臂 IP: {config['robot_ip']}")
    logger.info(f"末端执行器: {config.get('end_effector', 'none')}")
    
    # 创建并初始化遥操控系统
    from core.teleop_system_ros import TeleopSystemROS

    system = TeleopSystemROS(config)
    system.initialize(mode=mode)
    logger.info(f"系统初始化完成 (模式: {mode})")
    
    return system

class TeleopROSNode:
    def __init__(self):
        rospy.init_node('teleop_system', anonymous=True)
        
        # 从ROS参数服务器获取参数
        self.config_path = rospy.get_param('~config_path', 'config/config_mh6_topic_bridge.yaml')
        self.vp_ip = rospy.get_param('~vp_ip', None)
        self.robot_ip = rospy.get_param('~robot_ip', None)
        self.end_effector = rospy.get_param('~end_effector', None)
        self.log_level = rospy.get_param('~log_level', None)
        self.process_name = rospy.get_param('~process_name', None)
        self.mode = rospy.get_param('~mode', 'full')

        # rospy.wait_for_service('/aris_node/start_teleop_srv')
        # self.start_teleop_service = rospy.ServiceProxy('/aris_node/start_teleop_srv', StartDualTeleOP)
        
        # 轨迹相关参数
        self.command = rospy.get_param('~command', None)
        self.record_name = rospy.get_param('~record_name', None)
        self.play_file = rospy.get_param('~play_file', None)
        self.play_speed = rospy.get_param('~play_speed', 1.0)
        
        self.system = None
        self.logger = None
        
    def run(self):
        """运行ROS节点"""
        try:
            # 创建并初始化系统
            self.system = run_teleop_system(
                config_path=self.config_path,
                vp_ip=self.vp_ip,
                robot_ip=self.robot_ip,
                end_effector=self.end_effector,
                log_level=self.log_level,
                process_name=self.process_name,
                mode="full"
            )
            
            self.logger = get_logger()
            
            # 正常启动遥操作系统
            self._handle_normal_operation()
                
        except Exception as e:
            rospy.logerr(f"遥操控系统启动失败: {e}")
            if self.logger:
                self.logger.error(f"遥操控系统启动失败: {e}")
            raise
    '''
    # self.start_teleop_service = rospy.ServiceProxy('/aris_node/start_teleop_srv', StartDualTeleOP)
    # tele_req = StartDualTeleOPRequest()
    # tele_req.running_flag = False
    # tele_response = self.start_teleop_service.call(tele_req)
    # if tele_response.success:
    #     rospy.loginfo("已通知底层控制节点关闭双臂遥操作模式")
    # else:
    #     rospy.logerr("通知底层控制节点关闭双臂遥操作模式失败")
    '''
    # def _handle_normal_operation(self):
    #     """处理正常运行模式"""
        
    #     # [1] 保存终端设置
    #     self.original_terminal_settings = termios.tcgetattr(sys.stdin)

    #     # [2] 初始化服务代理 (建议放在循环外，提前准备好)
    #     # 注意：这里假设你已经导入了 StartDualTeleOP 和 StartDualTeleOPRequest
    #     self.start_teleop_service = rospy.ServiceProxy('/aris_node/start_teleop_srv', StartDualTeleOP)

    #     def is_key_pressed(target_key='q'):
    #         """内部辅助函数：非阻塞检测按键"""
    #         try:
    #             tty.setraw(sys.stdin.fileno())
    #             rlist, _, _ = select.select([sys.stdin], [], [], 0)
    #             if rlist:
    #                 key = sys.stdin.read(1)
    #                 return key == target_key
    #             return False
    #         finally:
    #             termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)

    #     try:
    #         self.system.start()
    #         self.logger.info("系统已启动，按 'q' 键停止并通知底层...")
            
    #         rate = rospy.Rate(10)
    #         while not rospy.is_shutdown():
                
    #             # [3] 检测按键
    #             if is_key_pressed('q'):
    #                 self.logger.info("\n检测到停止指令 (q)...")
                    
    #                 # --- 插入你的服务调用逻辑 ---
    #                 try:
    #                     self.logger.info("正在调用服务通知底层关闭遥操作...")
    #                     tele_req = StartDualTeleOPRequest()
    #                     tele_req.running_flag = False
                        
    #                     # 调用服务
    #                     tele_response = self.start_teleop_service.call(tele_req)
                        
    #                     if tele_response.success:
    #                         self.logger.info("成功：已通知底层控制节点关闭双臂遥操作模式")
    #                     else:
    #                         self.logger.error("失败：底层控制节点返回关闭失败")
                            
    #                 except rospy.ServiceException as e:
    #                     self.logger.error(f"服务调用异常 (Service可能是断开的): {e}")
    #                 except Exception as e:
    #                     self.logger.error(f"发生未知错误: {e}")
                    
    #                 # --- 调用完成后跳出循环 ---
    #                 break 
                
    #             rate.sleep()
                
    #     except rospy.ROSInterruptException:
    #         self.logger.info("\n接收到ROS终止信号...")
    #     except Exception as e:
    #         self.logger.exception(f"系统运行过程中发生错误: {e}")
    #     finally:
    #         # [4] 恢复终端并关闭本地系统
    #         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)
            
    #         if self.system:
    #             self.system.stop()
    #             self.logger.info("系统已安全关闭")
    
    def _handle_normal_operation(self):
        """处理正常运行模式"""
        try:
            self.system.start()
            self.logger.info("系统已启动，等待ROS关闭信号")
            
            # ROS主循环
            rate = rospy.Rate(10)  # 10Hz
            while not rospy.is_shutdown():
                rate.sleep()
                
        except rospy.ROSInterruptException:
            self.logger.info("\n接收到ROS终止信号...")
        except Exception as e:
            self.logger.exception(f"系统运行过程中发生错误: {e}")
        finally:
            if self.system:
                self.system.stop()
                self.logger.info("系统已安全关闭")

def main():
    """ROS节点主函数"""
    try:
        node = TeleopROSNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"遥操控系统ROS节点运行失败: {e}")

if __name__ == "__main__":
    main()