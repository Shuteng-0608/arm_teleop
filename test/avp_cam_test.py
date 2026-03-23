from avp_stream import VisionProStreamer
import time

avp_ip = "192.168.8.145"  # Vision Pro IP (shown in the app)
s = VisionProStreamer(ip=avp_ip)
'''
# 1. 查找谁在占用 video6
sudo lsof /dev/video6

# 2. 杀掉所有相关的 python 进程（最简单粗暴的方法）
pkill -9 python
'''
# Configure video streaming from robot camera 
# "/dev/video6" is the color camera on the head
s.configure_video(device="/dev/video6", format="v4l2", size="640x400", fps=30) # /dev/video6 is the color camera on the head
# s.configure_video(device="/dev/video14", format="v4l2", size="1280x720", fps=60) # /dev/video14 is the color camera on the breast
                                                                                # /dev/video10 is the grey camera on the breast
time.sleep(5.0)  # 等待配置生效
s.start_webrtc()
try:
    while True:
        r = s.get_latest()
        # Use tracking data to control your robot
        head_pose = r['head']
        # right_wrist = r['right_wrist']
        # right_fingers = r['right_fingers']

        # print(head_pose)
        time.sleep(1.0)
finally:
    print("正在关闭流并释放资源...")
    # 检查 avp_stream 是否有 stop 或 close 方法
    # 根据 avp_stream 的源码，通常有类似 stop_webrtc()
    if hasattr(s, 'stop_webrtc'):
        s.cleanup()  # 假设 cleanup 方法会停止流并释放资源
        s.cleanup()  # 假设 cleanup 方法会停止流并释放资源
        s.cleanup()  # 假设 cleanup 方法会停止流并释放资源
        s.cleanup()  # 假设 cleanup 方法会停止流并释放资源
        s.cleanup()  # 假设 cleanup 方法会停止流并释放资源
        s.cleanup()  # 假设 cleanup 方法会停止流并释放资源
# import os
# import time
# from avp_stream import VisionProStreamer

# # --- 1. 强制重置相机状态 ---
# print("正在尝试重置视频设备...")
# # 关闭流并尝试切换一次格式来激活 ISP
# os.system("v4l2-ctl -d /dev/video14 --stream-off")
# time.sleep(0.5)

# # --- 2. 配置并启动 ---
# avp_ip = "192.168.8.145"
# s = VisionProStreamer(ip=avp_ip)

# # 建议：明确指定 MJPG 格式（如果 avp_stream 库允许通过 pixel_format 参数）
# # 如果库不支持指定格式，至少确保分辨率匹配
# try:
#     s.configure_video(device="/dev/video14", format="v4l2", size="640x400", fps=30)
#     s.start_webrtc()
    
#     print("== 视频流已启动，请查看 Apple Vision Pro ==")

#     while True:
#         r = s.get_latest()
#         # if r and 'head' in r:
#         #     print(f"Tracking active: {r['head'][0][0][:3]}") # 简略打印
#         time.sleep(0.1)

# except KeyboardInterrupt:
#     print("\n检测到退出，正在释放资源...")
# finally:
#     # 强制尝试释放视频节点
#     os.system("v4l2-ctl -d /dev/video14 --stream-off")
#     print("资源已回收")



    
