from avp_stream import VisionProStreamer
import time
avp_ip = "192.168.1.63"   # example IP 
s = VisionProStreamer(ip = avp_ip, record = True)

while True:
    time.sleep(2.0)
    r = s.latest
    print(r['right_wrist'])