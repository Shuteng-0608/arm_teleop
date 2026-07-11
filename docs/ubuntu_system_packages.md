# Ubuntu System Packages

Audit-generated package inventory for deploying the MuJoCo peg-in-hole teleoperation and HDF5 recording stack.

Status labels:
- required: directly needed by repository build/runtime.
- likely required: needed by common wheels/backends or by the observed runtime architecture.
- optional: needed only for alternate modes.
- development only: useful for building/debugging, not runtime.
- unknown until target inspection: depends on GPU, display server, or local private libraries.

## Ubuntu 20.04 + ROS Noetic

Recommended native target because ROS Noetic is the ROS1 distribution normally paired with Ubuntu 20.04 and Python 3.8.

| Purpose | Package | Status | Notes |
|---|---|---:|---|
| build tools | `build-essential` | required | C++ IK nodes build from source. |
| build tools | `cmake` | required | `CMakeLists.txt` requires CMake >= 3.10. |
| build tools | `git` | required | Repository checkout and subdependency retrieval. |
| Python development | `python3`, `python3-dev`, `python3-pip`, `python3-setuptools` | required | ROS Noetic uses Python 3.8 on Ubuntu 20.04. |
| ROS | `ros-noetic-ros-base` or `ros-noetic-desktop-full` | required | `rospy`, `roscpp`, message/service generation, `roscore`. Desktop-full is heavier but convenient. |
| ROS messages/services | `ros-noetic-std-msgs`, `ros-noetic-std-srvs`, `ros-noetic-geometry-msgs`, `ros-noetic-sensor-msgs`, `ros-noetic-trajectory-msgs`, `ros-noetic-roslib` | required | Directly imported or listed in `CMakeLists.txt`. |
| ROS build | `python3-catkin-tools` or `catkin`, `ros-noetic-message-generation`, `ros-noetic-message-runtime` | required | Build package messages/services and nodes. |
| C++ IK | `libeigen3-dev`, `libceres-dev`, `libyaml-cpp-dev`, `libsuitesparse-dev` | required | `CMakeLists.txt` uses Eigen3, Ceres SuiteSparse, yaml-cpp. |
| Local IK library | `/usr/local/lib/libarm_kinematics.so`, ArmKinematics CMake config | required | Not provided by apt or this repo; current build hardcodes `/usr/local/lib/libarm_kinematics.so`. |
| OpenGL/Mesa | `libgl1`, `libegl1`, `libgles2`, `libosmesa6`, `mesa-utils` | likely required | MuJoCo/OpenCV rendering backends; exact backend depends on `MUJOCO_GL`. |
| GLFW/X11 | `libglfw3`, `libx11-6`, `libxrandr2`, `libxinerama1`, `libxcursor1`, `libxi6`, `libxxf86vm1` | likely required | MuJoCo viewer and GLFW-based offscreen contexts. |
| OpenCV GUI | `libglib2.0-0`, `libsm6`, `libxext6`, `libxrender1`, `libgtk-3-0`, `libxcb-xinerama0` | likely required | Required by GUI-enabled OpenCV wheels on many Linux desktops. |
| HDF5 | `libhdf5-103` or conda `hdf5` | likely required | Pip/conda `h5py` wheels often bundle/link HDF5, but native library may still matter. |
| FFmpeg/image | `ffmpeg`, `libavcodec-extra` | optional | Useful for analysis/visualization; OpenCV wheels include FFmpeg. |
| network tools | `iproute2`, `net-tools`, `iputils-ping`, `curl` | optional | Debug Vision Pro/ROS network connectivity. |
| performance | `linux-tools-generic`, `cpufrequtils` | optional | Inspect/set CPU governor. |
| GPU | NVIDIA driver packages | unknown until target inspection | Needed only if using NVIDIA EGL/desktop GL. |
| USB/device | `udev` rules, camera packages | optional | Current MuJoCo peg path does not require physical camera devices. |

## Ubuntu 22.04

Ubuntu 22.04 is not equivalent for this repository because it is ROS1 `rospy`/catkin code, while Ubuntu 22.04 is the native platform for ROS2 Humble, not ROS Noetic.

| Purpose | Package/route | Status | Notes |
|---|---|---:|---|
| ROS route | Docker/VM with Ubuntu 20.04 + ROS Noetic | likely required | Lowest-risk way to keep ROS1 Noetic behavior on a 22.04 host. |
| ROS route | RoboStack ROS Noetic in Conda | optional | Can work, but must validate `rospy`, generated messages, OpenCV GUI, MuJoCo GL, and catkin build. |
| ROS route | Source build ROS Noetic on 22.04 | development only | High maintenance; Python and system dependency friction expected. |
| ROS2 route | Port to ROS2 Humble | development only | Requires code changes; outside this audit. |
| Non-ROS route | Remove ROS service/client dependency | development only | Requires code changes to preserve keyboard/service workflow. |

The non-ROS native packages for OpenGL, GLFW, X11, HDF5, Ceres, Eigen, and yaml-cpp have similar names on 22.04, but ROS package names/distribution compatibility are the hard part.
