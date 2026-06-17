import h5py
import numpy as np

path = "/home/stw/pangu/src/arm_teleop/data/peg_hole_fixed_insertion_tare/20260617_153036_teleop_002/episode.hdf5"

with h5py.File(path, "r") as f:
    raw = f["observations/ft_wrench_raw"][:]
    grav = f["observations/ft_wrench_gravity"][:]
    comp = f["observations/ft_wrench"][:]

    print("mode:", f["episode_metadata"].attrs["ft_compensation_mode"])
    print("convention:", f["episode_metadata"].attrs["ft_wrench_convention"])
    print("tool mass:", f["episode_metadata"].attrs["ft_gravity_tool_mass_initial"])
    print("sign:", f["episode_metadata"].attrs["ft_gravity_sensor_sign"])

    print("raw shape:", raw.shape)
    print("grav shape:", grav.shape)
    print("comp shape:", comp.shape)

    print("max abs(raw - grav - comp):", np.max(np.abs(raw - grav - comp)))

    print("raw mean first 100:", np.mean(raw[:100], axis=0))
    print("grav mean first 100:", np.mean(grav[:100], axis=0))
    print("comp mean first 100:", np.mean(comp[:100], axis=0))