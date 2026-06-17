import h5py
import numpy as np

path = "/home/stw/pangu/src/arm_teleop/data/peg_hole_fixed_insertion_tare/20260617_143940_teleop_002/episode.hdf5"

with h5py.File(path, "r") as f:
    raw = f["observations/ft_wrench_raw"][:]
    comp = f["observations/ft_wrench"][:]
    bias = f["episode_metadata/ft_wrench_bias_raw"][:]

    print("raw shape:", raw.shape)
    print("comp shape:", comp.shape)
    print("bias:", bias)

    print("first raw:", raw[0])
    print("first comp:", comp[0])
    print("raw[0] - bias:", raw[0] - bias)

    print("mean first 20 comp:", np.mean(comp[:20], axis=0))