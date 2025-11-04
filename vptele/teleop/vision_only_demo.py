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
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

from teleop.open_television.tv_wrapper import TeleVisionWrapper
from teleop.image_server.image_client import ImageClient

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

def rotation_matrix_to_euler(R):
    """
    将旋转矩阵转换为欧拉角 (rx, ry, rz)
    
    参数:
        R: 3x3 旋转矩阵
        
    返回:
        list: 欧拉角 [rx, ry, rz]
    """
    # 确保是有效的旋转矩阵
    if abs(np.linalg.det(R) - 1.0) > 1e-6:
        # 如果不是正交矩阵，进行调整
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
    
    # 从旋转矩阵提取欧拉角 (rx, ry, rz)
    # 使用 ZYX 欧拉角顺序 (常见于机器人控制)
    if abs(R[2, 0]) != 1:
        theta1 = -math.asin(R[2, 0])
        theta2 = math.pi - theta1
        
        psi1 = math.atan2(R[2, 1] / math.cos(theta1), R[2, 2] / math.cos(theta1))
        psi2 = math.atan2(R[2, 1] / math.cos(theta2), R[2, 2] / math.cos(theta2))
        
        phi1 = math.atan2(R[1, 0] / math.cos(theta1), R[0, 0] / math.cos(theta1))
        phi2 = math.atan2(R[1, 0] / math.cos(theta2), R[0, 0] / math.cos(theta2))
        
        # 通常选择第一组解
        rx = psi1
        ry = theta1
        rz = phi1
    else:
        # 万向锁情况 (Gimbal lock)
        phi = 0  # 任意值
        if R[2, 0] == -1:
            theta = math.pi/2
            psi = phi + math.atan2(R[0, 1], R[0, 2])
        else:
            theta = -math.pi/2
            psi = -phi + math.atan2(-R[0, 1], -R[0, 2])
            
        rx = psi
        ry = theta
        rz = phi
    
    return [rx, ry, rz]

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera_type', type=str, choices=['opencv', 'realsense'], default='realsense', help='选择相机类型')
    parser.add_argument('--camera_id', type=int, default=0, help='OpenCV相机ID')
    parser.add_argument('--resolution', type=str, default='640x480', help='相机分辨率，格式为WxH')
    args = parser.parse_args()
    
    # 解析分辨率
    width, height = map(int, args.resolution.split('x'))
    
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
    
    # 创建TeleVision包装器，连接到VisionPro
    tv_wrapper = TeleVisionWrapper(BINOCULAR, tv_img_shape, tv_img_shm.name)
    
    try:
        print("VisionPro图像传输演示已启动")
        print("按'q'键退出程序")
        # Setup logging
        log_dir = os.path.join(current_dir, "logs")
        os.makedirs(log_dir, exist_ok=True)
        left_log_filename = os.path.join(log_dir, f"left_vision_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        right_log_filename = os.path.join(log_dir, f"right_vision_data_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log")
        
        running = True
        while running:
            # 获取数据，仅关注相机图像传输，忽略手部和手臂控制
            head_rmat, left_wrist, right_wrist, left_hand, right_hand = tv_wrapper.get_data()
            

            # Create log file if it doesn't exist yet
            # if not hasattr(tv_wrapper, 'log_created'):
            #     with open(log_filename, 'w') as f:
            #         f.write("timestamp,head_rmat_data,left_wrist_data,right_wrist_data,left_hand_data,right_hand_data\n")
            #     tv_wrapper.log_created = True

            # Log data
            timestamp = datetime.datetime.now().isoformat()
            with open(left_log_filename, 'a') as f:
                # f.write(f"{timestamp}\n{repr(head_rmat)}\n{repr(left_wrist)}\n{repr(right_wrist)}\n{repr(left_hand)}\n{repr(right_hand)}\n")
                # 转换left_wrist和right_wrist, 从4*4 到 x,y,z, rx ,ry,rz
                if left_wrist is not None:
                    # Extract position from left wrist transformation matrix
                    left_pos = [round(val, 3) for val in left_wrist[:3, 3].tolist()]  # Get last column for position
                    # Extract rotation matrix and convert to Euler angles
                    left_rot = [round(val, 3) for val in rotation_matrix_to_euler(left_wrist[:3, :3])]
                    # Combine position and rotation
                    left_wrist_converted = left_pos + left_rot
                else:
                    left_wrist_converted = None
                    
                f.write(f"{repr(left_wrist_converted)}\n")
                
            with open(right_log_filename, 'a') as f:
                if right_wrist is not None:
                    # Extract position from right wrist transformation matrix
                    right_pos = [round(val, 3) for val in right_wrist[:3, 3].tolist()]  # Get last column for position
                    # Extract rotation matrix and convert to Euler angles
                    right_rot = [round(val, 3) for val in rotation_matrix_to_euler(right_wrist[:3, :3])]
                    # Combine position and rotation
                    right_wrist_converted = right_pos + right_rot
                else:
                    right_wrist_converted = None
                f.write(f"{repr(right_wrist_converted)}\n")
            
            # 显示图像预览
            tv_resized_image = cv2.resize(tv_img_array, (tv_img_shape[1] // 2, tv_img_shape[0] // 2))
            cv2.imshow("VisionPro Camera Feed", tv_resized_image)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                running = False
                
            # 控制循环频率
            time.sleep(1/30)  # 约30FPS
            
    except KeyboardInterrupt:
        print("用户中断，退出程序...")
    finally:
        tv_img_shm.unlink()
        tv_img_shm.close()
        cv2.destroyAllWindows()
        print("程序已退出")