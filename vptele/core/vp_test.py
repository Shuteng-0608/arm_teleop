from avp_stream import VisionProStreamer
import time

avp_ip = "192.168.8.145"  # Vision Pro IP (shown in the app)
s = VisionProStreamer(ip=avp_ip)

# Configure video streaming from robot camera 
# "/dev/video6" is the color camera on the head
# s.configure_video(device="/dev/video6", format="v4l2", size="640x400", fps=30)
s.configure_video(device="/dev/video6", format="v4l2", size="1280x720", fps=60)
s.start_webrtc()

while True:
    r = s.get_latest()
    # Use tracking data to control your robot
    head_pose = r['head']
    # right_wrist = r['right_wrist']
    # right_fingers = r['right_fingers']

    print(head_pose)
    time.sleep(1.0)

