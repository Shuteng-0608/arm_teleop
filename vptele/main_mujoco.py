#!/usr/bin/env python3

import argparse
import rospy
import os
from utils.logger import get_logger, setup_logger
from utils.mujoco_config import (
    MujocoConfigError,
    apply_runtime_overrides,
    load_mujoco_config,
    validate_mujoco_config,
)

def run_teleop_system(
    config_path,
    vp_ip=None,
    end_effector=None,
    log_level=None,
    process_name=None,
    mode="full",
    review_mode=None,
    target_episodes=None,
    max_attempts=None,
    reject_action=None,
):
    """
    初始化并运行机械臂遥操控系统
    
    Args:
        config_path (str): 配置文件路径
        vp_ip (str, optional): 覆盖配置文件中的VisionPro IP
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
    
    # 加载、覆盖并校验配置文件。严格加载器会拒绝重复 YAML 键。
    try:
        config = load_mujoco_config(config_path)
        config = apply_runtime_overrides(
            config,
            vp_ip=vp_ip,
            end_effector=end_effector,
            process_name=process_name,
            review_mode=review_mode,
            target_episodes=target_episodes,
            max_attempts=max_attempts,
            reject_action=reject_action,
        )
        validate_mujoco_config(config)
    except MujocoConfigError as e:
        error_msg = f"配置文件加载或校验失败 {config_path}: {e}"
        rospy.logerr(error_msg)
        raise RuntimeError(error_msg)
    
    # 设置日志系统
    log_config = config.get('logging', {})
    console_level = log_level or log_config.get('console_level', 'info')
    file_level = log_config.get('file_level', 'debug')
    max_file_size = log_config.get('max_file_size', 100 * 1024 * 1024)
    backup_count = log_config.get('backup_count', 5)
    
    # log_prefix = "mujoco"
    log_prefix = log_config.get('log_prefix', 'mujoco')

    setup_logger(console_level, file_level, max_file_size, backup_count, prefix=log_prefix)
    logger = get_logger(log_prefix)
    
    logger.info(f"正在启动VisionPro机械臂遥操控系统... 配置文件: {config_path}")
    
    # 创建并初始化遥操控系统
    from core.teleop_system_mujoco import TeleopSystemMujoco

    system = TeleopSystemMujoco(config)
    system.initialize(mode=mode)
    logger.info(f"系统初始化完成 (模式: {mode})")
    
    return system

class TeleopROSNode:
    def __init__(self, cli_options=None):
        rospy.init_node('teleop_system', anonymous=True)
        cli_options = cli_options or argparse.Namespace()
        
        # 从ROS参数服务器获取参数
        self.config_path = rospy.get_param('~config_path', 'config/config_arm_right_peg.yaml')
        self.vp_ip = rospy.get_param('~vp_ip', None)
        self.end_effector = rospy.get_param('~end_effector', None)
        self.log_level = rospy.get_param('~log_level', None)
        self.process_name = rospy.get_param('~process_name', None)
        self.mode = rospy.get_param('~mode', 'full')
        self.review_mode = getattr(cli_options, "review_mode", None)
        if self.review_mode is None:
            self.review_mode = rospy.get_param('~review_mode', None)
        self.target_episodes = getattr(cli_options, "target_episodes", None)
        if self.target_episodes is None:
            self.target_episodes = rospy.get_param('~target_episodes', None)
        self.max_attempts = getattr(cli_options, "max_attempts", None)
        if self.max_attempts is None:
            self.max_attempts = rospy.get_param('~max_attempts', None)
        self.reject_action = getattr(cli_options, "reject_action", None)
        if self.reject_action is None:
            self.reject_action = rospy.get_param('~reject_action', None)

        
        self.system = None
        self.logger = None
        
    def run(self):
        """运行ROS节点"""
        try:
            # 创建并初始化系统
            self.system = run_teleop_system(
                config_path=self.config_path,
                vp_ip=self.vp_ip,
                # robot_ip=self.robot_ip,
                end_effector=self.end_effector,
                log_level=self.log_level,
                process_name=self.process_name,
                mode=self.mode,
                review_mode=self.review_mode,
                target_episodes=self.target_episodes,
                max_attempts=self.max_attempts,
                reject_action=self.reject_action,
            )
            
            self.logger = get_logger()
            
            # 正常启动遥操作系统
            self._handle_normal_operation()
                
        except Exception as e:
            rospy.logerr(f"遥操控系统启动失败: {e}")
            if self.logger:
                self.logger.error(f"遥操控系统启动失败: {e}")
            raise
    
 
    
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

def build_cli_parser():
    """Build command-line arguments after ROS remapping arguments are removed."""
    parser = argparse.ArgumentParser(
        description="MuJoCo peg-in-hole teleoperation and data collection",
    )
    parser.add_argument(
        "--review-mode",
        choices=("manual", "auto"),
        default=None,
        help="manual keeps console review; auto collects a retained-data batch",
    )
    parser.add_argument(
        "--target-episodes",
        type=int,
        default=None,
        help="number of accepted episodes to retain in auto mode",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=None,
        help="attempt safety limit; 0 uses an automatic limit",
    )
    parser.add_argument(
        "--reject-action",
        choices=("quarantine", "delete"),
        default=None,
        help="what to do with automatically rejected episode directories",
    )
    return parser


def parse_cli_options(argv=None):
    return build_cli_parser().parse_args(rospy.myargv(argv=argv)[1:])


def main():
    """ROS节点主函数"""
    try:
        node = TeleopROSNode(cli_options=parse_cli_options())
        node.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        import traceback
        traceback.print_exc()
        rospy.logerr(f"遥操控系统ROS节点运行失败: {e}")
        raise
        # rospy.logerr(f"遥操控系统ROS节点运行失败: {e}")

if __name__ == "__main__":
    main()
