import numpy as np

def vec_to_str(v):
    return f"{v[0]:.6f} {v[1]:.6f} {v[2]:.6f}"

def lookat_xyaxes(camera_pos, target_pos, up=np.array([0.0, 0.0, 1.0])):
    camera_pos = np.array(camera_pos, dtype=float)
    target_pos = np.array(target_pos, dtype=float)

    # MuJoCo camera looks along local -Z
    z_cam = camera_pos - target_pos
    z_cam = z_cam / np.linalg.norm(z_cam)

    x_cam = np.cross(up, z_cam)
    x_cam = x_cam / np.linalg.norm(x_cam)

    y_cam = np.cross(z_cam, x_cam)
    y_cam = y_cam / np.linalg.norm(y_cam)

    return vec_to_str(x_cam) + " " + vec_to_str(y_cam)

# 改这里：相机位置
camera_pos = [-0.10000, -0.30000, 1.10000]

# 改这里：看向目标，一般设为 wall_task 的 pos
target_pos = [-0.250, -0.500, 1.000]

print(lookat_xyaxes(camera_pos, target_pos))