#!/usr/bin/env python3

import argparse
import yaml
import os
import sys
from utils.logger import get_logger, setup_logger
import time

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
        print(error_msg)
        raise RuntimeError(error_msg)
    
    # 设置日志系统
    log_config = config.get('logging', {})
    console_level = log_level or log_config.get('console_level', 'info')
    file_level = log_config.get('file_level', 'debug')
    max_file_size = log_config.get('max_file_size', 100*1024*1024)
    backup_count = log_config.get('backup_count', 5)
    
    # 如果提供了进程名称，添加到日志文件名中, 否则取ip地址的最后一位，例如169.254.128.19取19
    log_prefix = f"{process_name}" if process_name else f"{config['robot_ip'].split('.')[-1]}"
    
    setup_logger(console_level, file_level, max_file_size, backup_count, prefix=log_prefix)
    logger = get_logger(log_prefix)
    
    logger.info(f"正在启动VisionPro机械臂遥操控系统... 配置文件: {config_path}")
    
    # 命令行参数覆盖配置文件
    if vp_ip:
        config['vp_ip'] = vp_ip
    if robot_ip:
        config['robot_ip'] = robot_ip
    if end_effector:
        config['end_effector'] = end_effector
    
    # 验证必要的配置项
    if 'vp_ip' not in config:
        logger.error("错误: 未指定VisionPro IP地址")
        raise ValueError("未指定VisionPro IP地址")
    if 'robot_ip' not in config:
        logger.error("错误: 未指定机械臂IP地址")
        raise ValueError("未指定机械臂IP地址")
    
    # 打印配置信息
    logger.info(f"VisionPro IP: {config['vp_ip']}")
    logger.info(f"机械臂 IP: {config['robot_ip']}")
    logger.info(f"末端执行器: {config.get('end_effector', 'none')}")
    
    # 创建并初始化遥操控系统
    from core.teleop_system import TeleopSystem

    system = TeleopSystem(config)
    system.initialize(mode=mode)
    logger.info(f"系统初始化完成 (模式: {mode})")
    
    return system

def run_system_blocking(config_path, vp_ip=None, robot_ip=None, end_effector=None, 
                       log_level=None, process_name=None):
    """以阻塞方式运行遥操控系统，直到收到中断信号"""
    system = run_teleop_system(config_path, vp_ip, robot_ip, end_effector, log_level, process_name)
    logger = get_logger()
    
    try:
        system.start()
        logger.info("系统已启动，按Ctrl+C退出")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\n接收到终止信号...")
    except Exception as e:
        logger.exception(f"系统运行过程中发生错误: {e}")
    finally:
        system.stop()
        logger.info("系统已安全关闭")
        
    return system

def main():
    """命令行入口函数"""
    parser = argparse.ArgumentParser(description="VisionPro 机械臂遥操控系统")
    parser.add_argument('--config', type=str, default='config/config_arm_right.yaml', 
                        help="配置文件路径")
    parser.add_argument('--vp-ip', type=str, help="VisionPro的IP地址")
    parser.add_argument('--robot-ip', type=str, help="机械臂的IP地址")
    parser.add_argument('--end-effector', type=str, choices=['none', 'hand', 'gripper'],
                        help="末端执行器类型")
    parser.add_argument('--log-level', type=str, default=None,
                        choices=['debug', 'info', 'warning', 'error', 'critical'],
                        help="日志级别")
    
    # 添加轨迹子命令
    subparsers = parser.add_subparsers(dest='command', help='命令')
    
    # 轨迹记录命令
    record_parser = subparsers.add_parser('record', help='记录轨迹')
    record_parser.add_argument('--name', type=str, help="轨迹文件名")
    
    # 轨迹回放命令
    play_parser = subparsers.add_parser('play', help='回放轨迹')
    play_parser.add_argument('--file', type=str, required=True, help="轨迹文件路径")
    play_parser.add_argument('--speed', type=float, default=1.0, help="回放速度因子")
    
    # 轨迹列表命令
    subparsers.add_parser('list', help='列出可用轨迹')
    
    args = parser.parse_args()
    
    # 在处理命令行参数时添加初始化模式判断
    mode = "playback" if args.command in ["play", "list"] else "full"
    
    # 创建并初始化系统
    system = run_teleop_system(
        config_path=args.config,
        vp_ip=args.vp_ip,
        robot_ip=args.robot_ip,
        end_effector=args.end_effector,
        log_level=args.log_level,
        mode=mode  # 传递初始化模式
    )
    
    logger = get_logger()
    
    # 处理轨迹相关命令
    if args.command == 'record':
        try:
            logger.info("开始记录轨迹...")
            system.start()
            system.start_recording()
            logger.info("按Ctrl+C停止记录")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("\n接收到终止信号，停止记录...")
            file_path = system.stop_recording(save=True, filename=args.name)
            system.stop()
            logger.info(f"轨迹已保存到: {file_path}")
        finally:
            system.stop()
            
    elif args.command == 'play':
        try:
            logger.info(f"开始回放轨迹: {args.file}")
            if args.speed != 1.0:
                system.trajectory_player.set_speed_factor(args.speed)
                
            success = system.play_trajectory(args.file)
            if success:
                logger.info("轨迹回放中，按Ctrl+C停止")
                # 等待回放完成或用户中断
                while system.trajectory_player.playing:
                    time.sleep(0.5)
            else:
                logger.error("无法开始轨迹回放")
        except KeyboardInterrupt:
            logger.info("\n接收到终止信号，停止回放...")
            system.stop_playback()
        finally:
            system.stop()
            
    elif args.command == 'list':
        trajectories = system.list_trajectories()
        print("\n可用的轨迹文件:")
        for i, traj in enumerate(trajectories):
            duration = traj.get("duration", 0)
            points = traj.get("points_count", 0)
            created = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(traj.get("created", 0)))
            print(f"{i+1}. {traj['filename']} - 点数: {points}, 持续时间: {duration:.2f}秒, 创建于: {created}")
        system.stop()
    else:
        # 正常运行遥操控系统
        try:
            system.start()
            logger.info("系统已启动，按Ctrl+C退出")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("\n接收到终止信号...")
        except Exception as e:
            logger.exception(f"系统运行过程中发生错误: {e}")
        finally:
            system.stop()
            logger.info("系统已安全关闭")

if __name__ == "__main__":
    main()