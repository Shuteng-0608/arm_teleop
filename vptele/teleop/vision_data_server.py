import numpy as np
import math
import time
import argparse
import cv2
from multiprocessing import shared_memory
import threading
import os
import sys
import datetime
import json
from flask import Flask, jsonify
from waitress import serve

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.open_television.tv_wrapper import TeleVisionWrapper
from teleop.image_server.image_client import ImageClient

class VisionDataServer:
    def __init__(self, binocular=False, tv_img_shape=None, tv_img_shm_name=None):
        """初始化Vision数据服务器"""
        self.binocular = binocular
        self.tv_img_shape = tv_img_shape
        self.tv_img_shm_name = tv_img_shm_name
        
        # 创建TeleVision包装器
        self.tv_wrapper = TeleVisionWrapper(binocular, tv_img_shape, tv_img_shm_name)
        
        # 存储最新的跟踪数据
        self.latest_data = {
            "head_rmat": None,
            "left_wrist": None,
            "right_wrist": None,
            "left_hand": None,
            "right_hand": None,
            "timestamp": None
        }
        
        # 创建数据更新线程
        self.running = True
        self.update_thread = threading.Thread(target=self._update_data_loop)
        self.update_thread.daemon = True
        
    def start(self):
        """启动数据更新线程"""
        self.update_thread.start()
        print("Vision数据更新线程已启动")
        
    def stop(self):
        """停止数据更新线程"""
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        print("Vision数据更新线程已停止")
        
    def _update_data_loop(self):
        """持续更新最新的跟踪数据"""
        while self.running:
            try:
                # 获取最新数据
                head_rmat, left_wrist, right_wrist, left_hand, right_hand = self.tv_wrapper.get_data()
                
                # 转换numpy数组为列表以便JSON序列化
                self.latest_data = {
                    "head_rmat": head_rmat.tolist() if isinstance(head_rmat, np.ndarray) else head_rmat,
                    "left_wrist": left_wrist.tolist() if isinstance(left_wrist, np.ndarray) else left_wrist,
                    "right_wrist": right_wrist.tolist() if isinstance(right_wrist, np.ndarray) else right_wrist,
                    "left_hand": left_hand.tolist() if isinstance(left_hand, np.ndarray) else left_hand,
                    "right_hand": right_hand.tolist() if isinstance(right_hand, np.ndarray) else right_hand,
                    "timestamp": datetime.datetime.now().isoformat()
                }
                
                # 控制更新频率
                time.sleep(1/300)  # 约30FPS
                
            except Exception as e:
                print(f"数据更新线程出错: {e}")
                time.sleep(1.0)  # 出错后延迟重试
                
    def get_latest_data(self):
        """获取最新的跟踪数据"""
        return self.latest_data


def find_realsense_cameras():
    """查找连接的RealSense相机"""
    try:
        import pyrealsense2 as rs

        ctx = rs.context()
        devices = []
        for d in ctx.devices:
            devices.append(d.get_info(rs.camera_info.serial_number))
            print(f"找到RealSense相机: {d.get_info(rs.camera_info.name)} " 
                f"(序列号: {d.get_info(rs.camera_info.serial_number)})")
        return devices
    except Exception as e:
        print(f"检测RealSense相机时出错: {e}")
        return []


def create_app(vision_data_server):
    """创建Flask应用"""
    app = Flask(__name__)
    
    @app.route('/api/vision_data', methods=['GET'])
    def get_vision_data():
        """API端点 - 获取最新的Vision数据"""
        return jsonify(vision_data_server.get_latest_data())
    
    @app.route('/health', methods=['GET'])
    def health_check():
        """健康检查端点"""
        return jsonify({"status": "healthy", "timestamp": datetime.datetime.now().isoformat()})
    
    return app


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_type', type=str, choices=['opencv', 'realsense'], default='realsense', help='选择相机类型')
    parser.add_argument('--camera_id', type=int, default=0, help='OpenCV相机ID')
    parser.add_argument('--resolution', type=str, default='640x480', help='相机分辨率，格式为WxH')
    parser.add_argument('--port', type=int, default=5301, help='服务器监听端口')
    args = parser.parse_args()
    
    # 解析分辨率
    width, height = map(int, args.resolution.split('x'))
    print(f"相机分辨率设置为: {width}x{height}")
    
    # 相机配置
    if args.camera_type == 'realsense':
        realsense_devices = find_realsense_cameras()
        if not realsense_devices:
            print("未检测到RealSense相机，请检查相机连接")
            exit(1)
        
        img_config = {
            'fps': 30,
            'head_camera_type': 'realsense',
            'head_camera_image_shape': [height, width],
            'head_camera_id_numbers': [realsense_devices[0]],
        }
    else:  # OpenCV相机
        img_config = {
            'fps': 30,
            'head_camera_type': 'opencv',
            'head_camera_image_shape': [height, width],
            'head_camera_id_numbers': [args.camera_id],
        }
    
    ASPECT_RATIO_THRESHOLD = 2.0
    if len(img_config['head_camera_id_numbers']) > 1 or (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        BINOCULAR = True
    else:
        BINOCULAR = False
    
    # 设置图像共享内存
    if BINOCULAR and not (img_config['head_camera_image_shape'][1] / img_config['head_camera_image_shape'][0] > ASPECT_RATIO_THRESHOLD):
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1] * 2, 3)
    else:
        tv_img_shape = (img_config['head_camera_image_shape'][0], img_config['head_camera_image_shape'][1], 3)

    tv_img_shm = shared_memory.SharedMemory(create=True, size=np.prod(tv_img_shape) * np.uint8().itemsize)
    tv_img_array = np.ndarray(tv_img_shape, dtype=np.uint8, buffer=tv_img_shm.buf)
    
    # 创建图像客户端
    img_client = ImageClient(tv_img_shape=tv_img_shape, tv_img_shm_name=tv_img_shm.name, server_address='127.0.0.1', Unit_Test=True)  
    
    # 创建并启动图像接收线程
    image_receive_thread = threading.Thread(target=img_client.receive_process, daemon=True)
    image_receive_thread.daemon = True
    image_receive_thread.start()
    
    try:
        # 创建并启动Vision数据服务器
        vision_server = VisionDataServer(BINOCULAR, tv_img_shape, tv_img_shm.name)
        vision_server.start()
        
        # 创建Flask应用
        app = create_app(vision_server)
        
        print(f"Vision数据服务器启动，监听在0.0.0.0:{args.port}")
        # 使用waitress作为生产级WSGI服务器
        serve(app, host='0.0.0.0', port=args.port)
        
    except KeyboardInterrupt:
        print("用户中断，退出程序...")
    finally:
        try:
            vision_server.stop()
        except:
            pass
            
        tv_img_shm.unlink()
        tv_img_shm.close()
        print("程序已退出")