import mujoco
import mujoco.viewer
import time

MODEL_PATH = "/home/stw/pangu/src/arm_teleop/model/Arm_right_15.xml"  


def main():
    # 加载模型
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    
    # 打印模型信息
    print(f"模型名称: {model.names}")
    print(f"自由度数量 (nv): {model.nv}")
    print(f"执行器数量 (nu): {model.nu}")
    print(f"关节数量 (njnt): {model.njnt}")
    
    # 列出所有关节
    print("\n关节列表:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name:
            print(f"  - {name}")
    
    # 列出所有执行器
    print("\n执行器列表:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        if name:
            print(f"  - {name}")
    
    # 打开查看器
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("\n可视化已启动，按ESC退出")
        print("模型已加载，不进行任何控制")
        
        # 主循环
        while viewer.is_running():
            # 推进仿真（无控制）
            mujoco.mj_step(model, data)
            
            # 更新可视化
            viewer.sync()
            
            # 短暂睡眠以控制仿真速度
            time.sleep(0.01)
    
    print("仿真结束")

if __name__ == "__main__":
    main()