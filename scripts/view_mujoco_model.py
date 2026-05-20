#!/usr/bin/env python3
import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer


def print_model_info(model):
    print("=" * 60)
    print("MuJoCo model loaded successfully.")
    print(f"nq:   {model.nq}")
    print(f"nv:   {model.nv}")
    print(f"nu:   {model.nu}")
    print(f"njnt: {model.njnt}")
    print("=" * 60)

    print("\nJoints:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"  {i:02d}: {name}")

    print("\nActuators:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  {i:02d}: {name}")

    print("=" * 60)


def set_initial_arm_pose(model, data):
    """
    设置一个稍微弯曲的初始姿态，避免机械臂完全摊开。
    如果你不想设置初始姿态，可以直接注释掉 main() 里的这行调用。
    """
    init_q = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]
    multiplier = [-1, 1, 1, -1, 1, 1, 1]
    init_q = [target_joints * multiplier for target_joints, multiplier in zip(init_q, multiplier)]

    for i, q in enumerate(init_q):
        joint_name = f"joint_{i + 1}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)

        if joint_id == -1:
            print(f"Warning: cannot find {joint_name}")
            continue

        qpos_addr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_addr] = q

    # mujoco.mj_forward(model, data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=str,
        default="model/right_arm_peg_tool.xml",
        help="Path to MuJoCo XML model."
    )
    parser.add_argument(
        "--no-init-pose",
        action="store_true",
        help="Do not set initial arm pose."
    )
    args = parser.parse_args()

    model_path = Path(args.model)

    if not model_path.exists():
        raise FileNotFoundError(f"Cannot find model file: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    print_model_info(model)

    if not args.no_init_pose:
        set_initial_arm_pose(model, data)

    print("\nOpening MuJoCo viewer...")
    print("Press ESC or close the viewer window to exit.")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()