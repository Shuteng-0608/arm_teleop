# 双臂7关节数据
std_msgs/Header header
uint32 sequence

# 左右机械臂数据
Joint7 right_arm
Joint7 left_arm

# 协调状态
bool coordination_ok
float64 sync_error