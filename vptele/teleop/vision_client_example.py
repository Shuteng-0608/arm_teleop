import requests
import json
import time

def get_vision_data(server_url="http://localhost:5301"):
    """从Vision数据服务器获取最新的跟踪数据"""
    try:
        response = requests.get(f"{server_url}/api/vision_data")
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print(f"获取Vision数据失败: {e}")
        return None

if __name__ == "__main__":
    while True:
        data = get_vision_data()
        if data:
            print(f"时间戳: {data['timestamp']}")
            print(f"头部旋转矩阵: {data['head_rmat']}")
            print(f"左手腕位置: {data['left_wrist']}")
            print(f"右手腕位置: {data['right_wrist']}")
            # 根据需要显示更多数据
        
        time.sleep(0.01)  # 每100ms获取一次数据