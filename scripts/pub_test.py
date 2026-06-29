#!/usr/bin/env python
import rospy
from your_package.msg import DualArmMovej  # 请替换为实际的消息包名

def talker():
    rospy.init_node('dual_arm_command_publisher', anonymous=True)
    pub = rospy.Publisher('/arm_teleop/dual_arm_movej', DualArmMovej, queue_size=100)
    
    rate = rospy.Rate(100)  # 100Hz频率
    
    while not rospy.is_shutdown():
        try:
            # 创建双臂消息
            # rospy.loginfo(f"头部绕Z轴旋转角度: {self.get_head_z_rotation()}")
            dual_arm_msg = DualArmMovej()
            # head_z_rotation = self.get_head_z_rotation()
            # rospy.loginfo(f"头部绕Z轴旋转角度: {head_z_rotation}")
            # if abs(head_z_rotation) > 0.1:
            #     rospy.loginfo(f"更新头部绕Z轴旋转角度: {head_z_rotation}")
            #     self.lastest_head_z_rotation = head_z_rotation
            #     dual_arm_msg.head_z_rotation = self.lastest_head_z_rotation
            # else:
            #     rospy.loginfo(f"保持头部绕Z轴旋转角度不变: {self.lastest_head_z_rotation}")
            #     dual_arm_msg.head_z_rotation = self.lastest_head_z_rotation
            
            
            # 设置header
            dual_arm_msg.header = Header()
            dual_arm_msg.header.stamp = rospy.Time.now()
            dual_arm_msg.header.frame_id = "pangu_base"
            dual_arm_msg.sequence = self.sequence
            self.sequence += 1
            
            # 更新左右臂数据
            dual_arm_msg.right_arm.arm_id = 1
            dual_arm_msg.left_arm.arm_id = 0
            
            dual_arm_msg.right_arm.arm_joints = self.last_smooth_joints_right
            # dual_arm_msg.right_arm.arm_joints = [0,0,0,0,0,0,0]
            dual_arm_msg.left_arm.arm_joints = self.last_smooth_joints_left
            # [0.314957,   0.238734,   -0.658534,   1.496385,   -1.000000,   -0.080329,   -0.113492 ]

            # dual_arm_msg.left_arm.arm_joints = [0,0,0,0,0,0,0]

            # dual_arm_msg.head_z_rotation = self.lastest_head_z_rotation
            dual_arm_msg.head_z_rotation = 0.0
            
            
            # 发布数据
            self.dual_arm_publisher.publish(dual_arm_msg)
            
                
        except Exception as e:
            rospy.logerr("Publish error: %s", str(e))
        # # 创建消息实例
        # msg = DualArmMovej()
        
        # # 根据实际消息结构填充数据
        # # msg.data = ...  # 请根据实际消息字段填充
        
        # # 发布消息
        # pub.publish(msg)
        
        # 按照100Hz频率等待
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass