# import h5py
# import numpy as np

# path = "/home/stw/pangu/src/arm_teleop/data/peg_hole_fixed_insertion_tare/20260617_153036_teleop_002/episode.hdf5"

# with h5py.File(path, "r") as f:
#     raw = f["observations/ft_wrench_raw"][:]
#     grav = f["observations/ft_wrench_gravity"][:]
#     comp = f["observations/ft_wrench"][:]

#     print("mode:", f["episode_metadata"].attrs["ft_compensation_mode"])
#     print("convention:", f["episode_metadata"].attrs["ft_wrench_convention"])
#     print("tool mass:", f["episode_metadata"].attrs["ft_gravity_tool_mass_initial"])
#     print("sign:", f["episode_metadata"].attrs["ft_gravity_sensor_sign"])

#     print("raw shape:", raw.shape)
#     print("grav shape:", grav.shape)
#     print("comp shape:", comp.shape)

#     print("max abs(raw - grav - comp):", np.max(np.abs(raw - grav - comp)))

#     print("raw mean first 100:", np.mean(raw[:100], axis=0))
#     print("grav mean first 100:", np.mean(grav[:100], axis=0))
#     print("comp mean first 100:", np.mean(comp[:100], axis=0))
import h5py
import numpy as np

path = "/home/stw/pangu/src/arm_teleop/data/peg_hole_fixed_insertion_tare/20260617_192158_teleop_002/episode.hdf5"

with h5py.File(path, "r") as f:
    qpos = f["observations/joint_pos"][:]
    action = f["action"][:]
    cmd = f["actions/joint_pos_command"][:]

    print("qpos shape:", qpos.shape)
    print("action shape:", action.shape)
    print("cmd shape:", cmd.shape)

    print("max abs(action - cmd):", np.max(np.abs(action - cmd)))

    print("first qpos:", qpos[0])
    print("first action:", action[0])
    print("first tracking error action-qpos:", action[0] - qpos[0])

    print("mean abs tracking error:",
          np.mean(np.abs(action - qpos), axis=0))