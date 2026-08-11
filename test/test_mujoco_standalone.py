#!/usr/bin/env python3
"""
独立的 MuJoCo 机械臂仿真测试程序。

目的：验证 MuJoCo 模型加载、物理仿真和 viewer 可视化是否正常工作，
      绕过所有 ROS / VisionPro / IK 服务依赖。

用法：
    python test_mujoco_standalone.py [--model PATH] [--mode MODE]

模式 (--mode):
    sine     : 对关节 1/2/4/6 施加正弦波运动，证明仿真能动（默认）
    idle     : 只加载模型并保持初始姿态，用于检查模型和 viewer
    keyframe : 在几个预设姿态之间来回切换

示例：
    python test_mujoco_standalone.py
    python test_mujoco_standalone.py --mode idle
    python test_mujoco_standalone.py --model /path/to/other_model.xml
"""

import argparse
import math
import time
import sys
import os

import numpy as np
import mujoco
import mujoco.viewer


# ---------------------------------------------------------------------------
# Default initial joint angles (radians) — same as the config file
# ---------------------------------------------------------------------------
DEFAULT_INITIAL_JOINTS = [-0.046, -0.2, 0.0, 1.6, -1.32, 0.005, 0.005]

# Actuator names in the model: motor_joint_1 .. motor_joint_7
ARM_JOINT_NAMES = [f"joint_{i}" for i in range(1, 8)]
ARM_ACTUATOR_NAMES = [f"motor_joint_{i}" for i in range(1, 8)]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _resolve_model_path(override: str = None) -> str:
    """Find the MuJoCo XML model file."""
    if override and os.path.exists(override):
        return override

    # Try default relative paths from this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, "..", "model", "pangu_all_right.xml"),
        os.path.join(script_dir, "..", "model", "right_arm_peg_tool.xml"),
        "/home/stw/pangu/src/arm_teleop/model/pangu_all_right.xml",
    ]
    for p in candidates:
        p = os.path.normpath(p)
        if os.path.exists(p):
            return p

    raise FileNotFoundError(
        "Cannot find MuJoCo XML model.  Pass --model PATH explicitly.\n"
        f"Tried: {candidates}"
    )


def set_arm_actuators(data, model, joint_angles):
    """
    Write joint angle commands directly to the arm actuators.

    Parameters
    ----------
    data : mujoco.MjData
    model : mujoco.MjModel
    joint_angles : list[float]   length-7 external joint angles (radians)
    """
    for joint_name, angle in zip(ARM_JOINT_NAMES, joint_angles):
        actuator_name = f"motor_{joint_name}"
        try:
            act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            if act_id >= 0:
                data.ctrl[act_id] = angle
        except Exception:
            pass


def get_arm_qpos(data, model):
    """Read current arm joint positions (external convention)."""
    angles = []
    for joint_name in ARM_JOINT_NAMES:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if joint_id == -1:
            angles.append(0.0)
        else:
            addr = model.jnt_qposadr[joint_id]
            angles.append(float(data.qpos[addr]) if addr < len(data.qpos) else 0.0)
    return angles


# ---------------------------------------------------------------------------
# Motion generators
# ---------------------------------------------------------------------------

def sine_wave(t: float, amplitude: float = 0.5, period: float = 3.0) -> float:
    """Single-joint sine wave."""
    return amplitude * math.sin(2.0 * math.pi * t / period)


def sine_motion(t: float, initial_joints):
    """
    Superimpose sine waves on joints 1, 2, 4, 6 to make the arm visibly move.

    The motion is kept conservative to avoid self-collision or extreme poses.
    Joints 1, 2, 4, 6 are chosen because they produce the most visible
    end-effector displacement.
    """
    # Periods are slightly different so the combined motion doesn't loop trivially
    return [
        initial_joints[0] + sine_wave(t, amplitude=0.3, period=4.0),   # joint_1
        initial_joints[1] + sine_wave(t, amplitude=0.2, period=5.5),   # joint_2
        initial_joints[2] + 0.0,                                        # joint_3 : hold
        initial_joints[3] + sine_wave(t, amplitude=0.15, period=6.0),  # joint_4
        initial_joints[4] + sine_wave(t, amplitude=0.1, period=4.5),   # joint_5
        initial_joints[5] + sine_wave(t, amplitude=0.08, period=7.0),  # joint_6
        initial_joints[6] + 0.0,                                        # joint_7 : hold
    ]


# ---------------------------------------------------------------------------
# Keyframe presets
# ---------------------------------------------------------------------------

KEYFRAMES = {
    "home":    [-0.046, -0.2,  0.0,  1.6,  -1.32, 0.005, 0.005],
    "raised":  [-0.046, -0.5,  0.3,  1.8,  -1.0,  0.3,   0.005],
    "forward": [ 0.2,   -0.4, -0.2,  1.4,  -1.5,  0.0,   0.1],
    "side":    [-0.3,   -0.1,  0.2,  2.0,  -0.8,  0.5,  -0.1],
}


def keyframe_motion(t: float, initial_joints):
    """
    Cycle through preset keyframes, holding each for ~3 seconds.
    Blends linearly over 1.5 s between poses.
    """
    names = list(KEYFRAMES.keys())
    total_frames = len(names)
    hold = 3.0     # seconds per keyframe
    blend = 1.5    # blend duration
    cycle = total_frames * (hold + blend)
    phase = t % cycle

    # Which segment are we in?
    seg_idx = int(phase // (hold + blend))
    seg_rem = phase % (hold + blend)

    a_idx = seg_idx % total_frames
    b_idx = (seg_idx + 1) % total_frames

    if seg_rem < hold:
        # Holding
        return list(KEYFRAMES[names[a_idx]])
    else:
        # Blending a -> b
        alpha = (seg_rem - hold) / blend
        # smoothstep
        alpha = alpha * alpha * (3.0 - 2.0 * alpha)
        q_a = np.array(KEYFRAMES[names[a_idx]])
        q_b = np.array(KEYFRAMES[names[b_idx]])
        return (q_a + alpha * (q_b - q_a)).tolist()


# ---------------------------------------------------------------------------
# Main test
# ---------------------------------------------------------------------------

def run_test(model_path: str, mode: str = "sine"):
    print("=" * 60)
    print("  MuJoCo standalone arm test")
    print(f"  Model : {model_path}")
    print(f"  Mode  : {mode}")
    print("=" * 60)

    # ---- Load model ----
    print("\n[1] Loading MuJoCo model ...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print(f"     Loaded.  Joints: {model.njnt},  Actuators: {model.nu}")
    print(f"     Timestep: {model.opt.timestep:.5f} s")

    # List arm actuators
    print("\n[2] Arm actuators found:")
    for name in ARM_ACTUATOR_NAMES:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act_id >= 0:
            print(f"       {name}  →  ctrl[{act_id}]")
        else:
            print(f"       {name}  →  NOT FOUND (will skip)")

    # ---- Set initial pose ----
    print(f"\n[3] Setting initial pose: {DEFAULT_INITIAL_JOINTS}")
    set_arm_actuators(data, model, DEFAULT_INITIAL_JOINTS)
    mujoco.mj_forward(model, data)

    # ---- Run simulation with viewer ----
    print("\n[4] Launching viewer + physics loop ...")
    print("     Controls:")
    print("       - Drag left mouse  : rotate view")
    print("       - Drag right mouse : translate view")
    print("       - Scroll wheel     : zoom")
    print("       - ESC / close window : quit")
    print()

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Switch to CCTV camera for a good overview angle
            cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_cam")
            if cam_id >= 0:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = cam_id
                print("     Viewer camera set to 'cctv_cam'.")
            else:
                print("     'cctv_cam' not found; using free camera.")

            sim_time = 0.0
            viewer_rate = 60.0
            viewer_period = 1.0 / viewer_rate
            last_sync = time.perf_counter()
            step_count = 0

            while viewer.is_running():
                step_start = time.perf_counter()

                # --- compute target joints for this frame ---
                if mode == "sine":
                    targets = sine_motion(sim_time, DEFAULT_INITIAL_JOINTS)
                elif mode == "keyframe":
                    targets = keyframe_motion(sim_time, DEFAULT_INITIAL_JOINTS)
                else:  # idle
                    targets = list(DEFAULT_INITIAL_JOINTS)

                # --- apply to actuators ---
                set_arm_actuators(data, model, targets)

                # --- step physics ---
                mujoco.mj_step(model, data)
                sim_time += model.opt.timestep

                # --- sync viewer ---
                now = time.perf_counter()
                if now - last_sync >= viewer_period:
                    viewer.sync()
                    last_sync = now

                # --- status print every 2 s ---
                step_count += 1
                if step_count % 2000 == 0:
                    q = get_arm_qpos(data, model)
                    print(f"  t={sim_time:.1f}s  joints: {[f'{x:+.3f}' for x in q]}")

                # --- real-time pacing ---
                elapsed = time.perf_counter() - step_start
                sleep_time = model.opt.timestep - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    print("\nDone.  MuJoCo test finished successfully.")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Standalone MuJoCo arm simulation test"
    )
    parser.add_argument(
        "--model",
        default=None,
        help="Path to MuJoCo XML model file (auto-detected if omitted).",
    )
    parser.add_argument(
        "--mode",
        default="sine",
        choices=["sine", "idle", "keyframe"],
        help="Test mode: sine (moving joints), idle (static), keyframe (pose cycling).",
    )
    args = parser.parse_args()

    model_path = _resolve_model_path(args.model)
    run_test(model_path, args.mode)


if __name__ == "__main__":
    main()
