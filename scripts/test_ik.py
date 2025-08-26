#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion
from arm_teleop.srv import ArmIK, ArmIKRequest
import rospy
import numpy as np
import tf.transformations as tf_trans

def test_arm_ik_methods():
    rospy.init_node('arm_ik_method_tester')
    
    # 定义目标变换矩阵 (从C++代码提取)
    tee_matrix = np.array([
        [0.70295886,  0.01215941,  0.71112656,  0.17662990],
        [-0.71005079,  0.06956257,  0.70070602, -0.60003029],
        [-0.04094762, -0.99750348,  0.05753342,  0.30322956],
        [0.00000000,  0.00000000,  0.00000000,  1.00000000]
    ])
    
    # 提取位置信息
    position = Point(
        x=tee_matrix[0, 3],
        y=tee_matrix[1, 3],
        z=tee_matrix[2, 3]
    )
    
    # 提取旋转矩阵并转换为四元数
    rot_matrix = tee_matrix[:3, :3]
    quat = tf_trans.quaternion_from_matrix(tee_matrix)
    orientation = Quaternion(
        x=quat[0],
        y=quat[1],
        z=quat[2],
        w=quat[3]
    )
    
    # 创建目标位姿
    target_pose = Pose(
        position=position,
        orientation=orientation
    )
    
    # 初始关节角度 (全零)
    init_joints = [0.0] * 7
    
    # 等待服务准备就绪
    rospy.loginfo("等待逆运动学服务...")
    rospy.wait_for_service('arm_ik_service')
    ik_service = rospy.ServiceProxy('arm_ik_service', ArmIK)
    rospy.loginfo("服务已就绪，开始测试...")
    
    # 测试结果存储
    results = []
    methods = ["std", "ofst", "comb"]
    
    # 测试每种方法
    for method in methods:
        rospy.loginfo(f"测试方法: {method.upper()}")
        try:
            # 准备请求
            req = ArmIKRequest()
            req.method = method
            req.init_joints = init_joints
            req.target_pose = target_pose
            
            # 调用服务并计时
            start_time = rospy.Time.now().to_sec()
            resp = ik_service(req)
            solve_time = rospy.Time.now().to_sec() - start_time
            
            # 存储结果
            result = {
                "method": method,
                "success": resp.success,
                "message": resp.message,
                "solution": resp.solution,
                "time": solve_time
            }
            results.append(result)
            
            rospy.loginfo(f"  -> 结果: {'成功' if resp.success else '失败'}, "
                         f"耗时: {solve_time:.4f}秒")
            
            # 短暂延时避免服务过载
            rospy.sleep(0.2)
            
        except rospy.ServiceException as e:
            rospy.logerr(f"服务调用失败: {e}")
            results.append({
                "method": method,
                "error": f"ServiceException: {e}",
                "success": False
            })
    
    # 打印详细报告
    print("\n" + "="*80)
    print("           七自由度机械臂逆运动学求解方法对比测试")
    print("="*80)
    
    # 打印目标位姿信息
    print(f"\n目标位置: [X: {position.x:.6f}, Y: {position.y:.6f}, Z: {position.z:.6f}]")
    print(f"目标姿态: [X: {orientation.x:.6f}, Y: {orientation.y:.6f}, "
          f"Z: {orientation.z:.6f}, W: {orientation.w:.6f}]")
    
    # 打印每种方法的测试结果
    print("\n" + "-"*80)
    print("{:<8} | {:<8} | {:<10} | {}".format("方法", "状态", "耗时(秒)", "详细信息"))
    print("-"*80)
    
    for res in results:
        # 处理异常情况
        if "error" in res:
            status = "错误"
            message = res["error"]
        elif not res["success"]:
            status = "失败"
            message = res["message"]
        else:
            status = "成功"
            message = res["message"]
        
        print("{:<8} | {:<8} | {:<10.5f} | {}".format(
            res["method"].upper(), 
            status, 
            res.get("time", 0),
            message
        ))
    
    # 打印成功方法的关节角解决方案
    success_results = [r for r in results if r.get("success", False)]
    if success_results:
        print("\n" + "-"*80)
        print("{:<8} | 关节角1-7 (弧度)".format("方法"))
        print("-"*80)
        
        for res in success_results:
            joints = ", ".join([f"{j:.6f}" for j in res["solution"]])
            print("{:<8} | {}".format(res["method"].upper(), joints))
    
    print("\n" + "="*80)
    rospy.loginfo("测试完成!")

if __name__ == "__main__":
    test_arm_ik_methods()















# from geometry_msgs.msg import Pose, Point, Quaternion
# from arm.srv import ArmIK, ArmIKRequest
# import rospy
# # 创建请求
# req = ArmIKRequest()
# req.method = "std"  # 选择求解方法
# req.init_joints = [0.0]*7
# req.target_pose = Pose(
#     position=Point(0.176, -0.600, 0.303),
#     orientation=Quaternion(0.012, 0.070, -0.998, 0.703)
# )

# # 调用服务
# rospy.wait_for_service('arm_ik_service')
# rospy.loginfo("使用的求解方法: %s", req.method)
# try:
#     resp = rospy.ServiceProxy('arm_ik_service', ArmIK)(req)
#     print(f"Success: {resp.success}")
#     print(f"Joints: {resp.solution}")
#     print(f"Message: {resp.message}")
# except rospy.ServiceException as e:
#     print(f"Service call failed: {e}")