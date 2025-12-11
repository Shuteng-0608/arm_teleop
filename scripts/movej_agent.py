#!/usr/bin/env python

import rospy
from arm_teleop.srv import MovejService, MovejServiceRequest

def movej_client():
    # 初始化ROS节点
    rospy.init_node('movej_client', anonymous=True)
    
    # 等待服务可用
    rospy.loginfo("等待 /aris_node/movej_srv 服务...")
    rospy.wait_for_service('/aris_node/movej_srv')
    
    try:
        # 创建服务代理
        movej_proxy = rospy.ServiceProxy('/aris_node/movej_srv', MovejService)
        
        # 创建请求对象
        request = MovejServiceRequest()
        
        # 设置请求参数（根据实际需求修改这些值）0.018009209771832683
        # left
        request.arm_id = 0  # 机械臂ID
        request.target_joints = [-0.043330316363489225, 0.141567115520149, 0.08319549831730459, 1.5942354225874358, -1.3761425017046844, -0.11544114274704914, -0.005078013054971109]  # 目标关节角度
        # right
        # request.arm_id = 1  # 机械臂ID
        # request.target_joints = [-0.043330316363489225, -0.141567115520149, 0.08319549831730459, 1.5942354225874358, -1.3761425017046844, -0.11544114274704914, -0.005078013054971109]
        request.vel = 0.5    # 速度
        request.acc = 5.0    # 加速度
        request.jerk = 10.0  # 加加速度
        
        # 调用服务
        rospy.loginfo("发送移动请求...")
        response = movej_proxy(request)
        
        # 处理响应
        if response.success:
            rospy.loginfo("移动执行成功")
        else:
            rospy.logwarn("移动执行失败: %s", response.message)
            
        return response
        
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", str(e))
        return None

if __name__ == "__main__":
    movej_client()