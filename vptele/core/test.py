#!/usr/bin/env python3
"""快速修复：从/dev/video8读取并转换格式，然后推送到Vision Pro"""

import cv2
import numpy as np
import time
import subprocess
import os
import sys
from avp_stream.streamer import VisionProStreamer

def main():
    device = "/dev/video8"
    avp_ip = "192.168.8.145"  # Vision Pro IP (shown in the app)
    
    print("快速修复摄像头格式问题")
    print("="*60)
    
    # 1. 创建虚拟摄像头
    print("创建虚拟摄像头...")
    os.system("sudo modprobe v4l2loopback devices=1 video_nr=20 card_label='VirtualCam' 2>/dev/null")
    time.sleep(2)
    
    virtual_device = "/dev/video20"
    if not os.path.exists(virtual_device):
        print(f"❌ 虚拟摄像头创建失败: {virtual_device}")
        return
    
    # 2. 启动ffmpeg进程
    print("启动ffmpeg格式转换...")
    ffmpeg_cmd = [
        'ffmpeg',
        '-f', 'rawvideo',
        '-pixel_format', 'bgr24',
        '-video_size', '640x400',
        '-framerate', '15',
        '-i', '-',
        '-f', 'v4l2',
        '-codec', 'mjpeg',
        '-q', '2',
        '-r', '15',
        virtual_device,
        '-y'
    ]
    
    try:
        ffmpeg_proc = subprocess.Popen(
            ffmpeg_cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(2)
        
        if ffmpeg_proc.poll() is not None:
            stderr = ffmpeg_proc.stderr.read().decode('utf-8')
            print(f"❌ ffmpeg启动失败: {stderr[:200]}")
            return
        
        print("✓ ffmpeg进程运行中")
    except Exception as e:
        print(f"❌ 启动ffmpeg失败: {e}")
        return
    
    # 3. 打开摄像头
    print(f"打开摄像头: {device}")
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"❌ 无法打开摄像头: {device}")
        ffmpeg_proc.terminate()
        return
    
    # 设置参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
    cap.set(cv2.CAP_PROP_FPS, 15)
    
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"摄像头实际参数: {actual_width}x{actual_height}")
    
    # 4. 启动Vision Pro流媒体
    print("启动Vision Pro流媒体...")
    try:
        s = VisionProStreamer(ip=avp_ip, record=False)
        s.configure_video(
            device=virtual_device,
            format="v4l2",
            size="640x400",
            fps=15,
        )
        s.start_webrtc(port=9999)
        print("✓ Vision Pro流媒体已启动")
        print(f"  Vision Pro应访问: http://192.168.8.52:8888/webrtc_info")
    except Exception as e:
        print(f"❌ Vision Pro流媒体启动失败: {e}")
        cap.release()
        ffmpeg_proc.terminate()
        return
    
    print("\n开始流媒体传输...")
    print("按 Ctrl+C 停止")
    print("="*60)
    
    # 5. 主循环
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # 读取摄像头帧
            ret, frame = cap.read()
            if not ret:
                print("读取帧失败")
                time.sleep(0.1)
                continue
            
            frame_count += 1
            
            # 如果是单通道（可能是拜耳格式），尝试转换
            if len(frame.shape) == 2:
                # 尝试不同的拜耳模式
                try:
                    # 首先尝试最常用的BG模式
                    frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_BG2BGR)
                except:
                    # 如果失败，尝试其他模式
                    try:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BAYER_RG2BGR)
                    except:
                        # 如果还是失败，转换为灰度伪彩色
                        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            
            # 调整大小到640x400
            if frame.shape[0] != 400 or frame.shape[1] != 640:
                frame = cv2.resize(frame, (640, 400))
            
            # 写入ffmpeg
            try:
                ffmpeg_proc.stdin.write(frame.tobytes())
            except BrokenPipeError:
                print("ffmpeg管道断开")
                break
            
            # 获取Vision Pro数据
            try:
                r = s.get_latest()
                if r and 'right_wrist' in r and r['right_wrist'] is not None:
                    pos = r['right_wrist'][0, :3, 3]
                    print(f"\r右手腕: [{pos[0]:7.3f}, {pos[1]:7.3f}, {pos[2]:7.3f}] 帧: {frame_count}", 
                          end="", flush=True)
            except Exception as e:
                print(f"\n获取数据错误: {e}")
            
            # 显示帧率
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start_time = time.time()
            
            time.sleep(0.01)  # 约100Hz
            
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        # 清理
        print("清理资源...")
        cap.release()
        if ffmpeg_proc and ffmpeg_proc.poll() is None:
            ffmpeg_proc.terminate()
            ffmpeg_proc.wait()
        
        print("程序结束")

if __name__ == "__main__":
    # 检查权限
    if os.geteuid() != 0:
        print("需要root权限来创建虚拟摄像头")
        print("请使用sudo运行: sudo python3 quick_fix.py")
        sys.exit(1)
    
    main()