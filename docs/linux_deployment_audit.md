# Linux Deployment Audit

Audit date: 2026-07-07

Repository: `/Users/wangshuteng/Desktop/arm_teleop`

Branch: `codex/force-visual-feedback-hud`

Working tree at start: clean (`git status --short` returned no output)

Remote: `origin git@github.com:Shuteng-0608/arm_teleop.git`

## 1. Executive Summary

The current MuJoCo peg-in-hole data-collection path is deployable in principle, but not from the committed dependency files alone. The code path requires ROS1/catkin, official `mujoco` Python bindings, GUI-enabled OpenCV, `h5py`, NumPy, SciPy, PyYAML, and the `avp_stream` Vision Pro package. The committed Conda YAML files are incomplete or risky for this path:

- `vptele/environment.yaml` comments indicate Python 3.9.21, but it does not list `mujoco`, `h5py`, or active `scipy`.
- `config/environment.yaml` pins Python 3.13.5, which is not suitable for native ROS1 Noetic.
- Both YAML files list `avp-stream==1.0`, but the package source/reproducibility could not be verified locally.
- The local shell is macOS Apple Python 3.9.6 with no project packages installed, so installed runtime versions could not be verified by import.

Primary recommendation: deploy on Ubuntu 20.04 with ROS Noetic, Python 3.8 for ROS/catkin interop, and a carefully pinned Conda or venv environment for the MuJoCo/OpenCV/HDF5 packages. Use GUI-enabled `opencv-python` or `opencv-contrib-python`, not any `*-headless` OpenCV wheel.

Top blockers:

1. Hardcoded Linux paths in config/defaults, especially `/home/stw/pangu/src/arm_teleop/...`.
2. Missing dependency pins for `mujoco`, `h5py`, and active `scipy`.
3. ROS1 Noetic/Ubuntu 20.04/Python 3.8 constraints conflict with the Python 3.13 Conda file.
4. `CMakeLists.txt` hardcodes `/usr/local/lib/libarm_kinematics.so` and `/usr/lib/libceres.so`.
5. OpenCV and MuJoCo rendering require a working Linux display/OpenGL stack.

## 2. Current Environment Findings

| Check | Result |
|---|---|
| `pwd` | `/Users/wangshuteng/Desktop/arm_teleop` |
| `git branch --show-current` | `codex/force-visual-feedback-hud` |
| `git status --short` | clean |
| `git remote -v` | `origin git@github.com:Shuteng-0608/arm_teleop.git` |
| `python --version` | command not found |
| `python3 --version` | Python 3.9.6 |
| `which python` | not found |
| `which python3` | `/usr/bin/python3` |
| `conda info --envs` | command not found |
| `python -m pip --version` | command not found |
| `python3 -m pip --version` | pip 21.2.4 from Apple CLT Python 3.9 |
| `python3 -m pip freeze` | only Apple internal packages: `altgraph`, `future`, `macholib`, `six` |
| Import check | `numpy`, `h5py`, `yaml`, `cv2`, `mujoco`, `scipy`, `matplotlib`, `avp_stream`, `requests` all missing in current shell |
| OpenCV build inspection | failed because `cv2` is not installed |
| MuJoCo version inspection | failed because `mujoco` is not installed |

The current machine is not a verified working runtime environment for this repo. Version recommendations below are based on repository code/config and compatibility constraints, not on successful local imports.

## 3. Runtime Call Chain

Entry point:

1. `vptele/main_mujoco.py`
   - Initializes ROS node `teleop_system`.
   - Loads YAML config, defaulting to `config/config_arm_right_peg.yaml` relative to `vptele/main_mujoco.py`.
   - Creates `TeleopSystemMujoco`.
2. `vptele/core/teleop_system_mujoco.py`
   - Imports `core.vp_streamer_avp.VPStreamer`.
   - Requires `vp_ip`.
   - Creates AVP streamer.
   - Creates `RobotControllerMuJoCoPegTool`.
   - Creates `ArmTeleopMujoco`.
3. `vptele/core/vp_streamer_avp.py`
   - Imports `avp_stream.VisionProStreamer`.
   - Appends the repository root to `sys.path`.
4. `vptele/arm_control/arm_teleop_mujoco.py`
   - Waits for `/arm_teleop/right_arm_ik_srv`.
   - Creates episode services under `/arm_teleop_mujoco`: `/stop`, `/recalibrate`, `/start`.
5. `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py`
   - Loads MuJoCo XML with `mujoco.MjModel.from_xml_path`.
   - Starts simulation/viewer thread.
   - Creates monitor camera renderer via `mujoco.Renderer`.
   - Displays OpenCV windows.
   - Creates ROS recording service `/mujoco_hdf5_recording/set_recording`.
   - Calls `MujocoHDF5Recorder.record_if_needed()` after `mj_step`.
6. `vptele/utils/mujoco_hdf5_recorder.py`
   - Writes HDF5 episode data, force samples, state samples, image tensors, action aliases, metadata, and event datasets.
7. `scripts/recording_keyboard_client.py`
   - Connects to `/mujoco_hdf5_recording/set_recording` and toggles recording using Enter.

This path requires `roscore`, generated package services, the right-arm IK service, and the MuJoCo teleop node at the same time.

## 4. OS / ROS / Python Matrix

| Route | Ubuntu | ROS | Python | Recommendation | Notes |
|---|---:|---|---:|---|---|
| Primary | 20.04 | ROS Noetic | 3.8 | recommended | Native ROS1 Noetic target. Best match for `rospy`, catkin, and generated services. |
| Conservative Conda | 20.04 | ROS Noetic from apt | 3.8 Conda/venv | recommended with validation | Need ensure `rospy` and generated messages are importable inside env, or run with system Python and install packages into compatible env. |
| Python 3.9 | 20.04 or custom | ROS1 custom/RoboStack | 3.9 | possible but riskier | `vptele/environment.yaml` suggests 3.9.21, but native Noetic apt packages target Python 3.8. |
| Python 3.10 | 22.04 | not native Noetic | 3.10 | not primary | ROS1 Noetic apt packages are not native to 22.04. |
| Python 3.11+ | 22.04+ | not native Noetic | 3.11+ | not recommended | Package wheels may exist, but `rospy`/catkin Noetic and generated messages are the constraint. |
| Python 3.13 | 22.04+ | not native Noetic | 3.13 | blocker | Present in `config/environment.yaml`; do not use for ROS1 Noetic deployment. |

Compatibility categories:

- Python language compatibility: code is mostly compatible with Python 3.8+ syntax. It uses f-strings, `from __future__ import annotations`, standard type hints, and no obvious Python 3.10-only syntax.
- Python package wheel availability: OpenCV, NumPy, h5py, SciPy, and MuJoCo wheels vary by Python version. Python 3.8/3.9 are safest for ROS1. Python 3.13 narrows compatibility for older ROS tooling and some pinned packages.
- ROS Python compatibility: ROS Noetic is the practical ROS1 Python 3 distribution and is normally paired with Python 3.8 on Ubuntu 20.04. Melodic/Kinetic are Python 2-era and unsuitable for this Python 3 code.
- System-library compatibility: OpenCV GUI and MuJoCo viewer/offscreen rendering require X11/GLFW/OpenGL/EGL/OSMesa libraries depending on display mode.

## 5. ROS Compatibility

The repository is ROS1, not ROS2:

- Uses `rospy`, `roscpp`, `std_srvs`, `geometry_msgs`, ROS `.msg` and `.srv` files.
- Uses catkin package metadata: `package.xml`, `CMakeLists.txt`, `catkin_python_setup()`, `add_message_files()`, `add_service_files()`, `generate_messages()`.
- README starts `roscore`, `rosrun arm_teleop ik_service_right_node`, and `rosrun arm_teleop main_mujoco.py`.

Expected ROS distribution:

- Recommended: ROS Noetic on Ubuntu 20.04.
- Native Python: Python 3.8.
- `roscore` is required.
- Catkin workspace setup is required:

```bash
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash
```

Required ROS packages:

- `catkin`
- `roscpp`
- `rospy`
- `std_msgs`
- `std_srvs`
- `geometry_msgs`
- `sensor_msgs`
- `trajectory_msgs`
- `roslib`
- `message_generation`
- `message_runtime`

Required local services/nodes:

- `ik_service_right_node`, serving `/arm_teleop/right_arm_ik_srv`.
- Optional left service from launch: `ik_service_left_node`.
- `main_mujoco.py`, serving `/mujoco_hdf5_recording/set_recording`.
- `ArmTeleopMujoco` episode services: `/arm_teleop_mujoco/stop`, `/arm_teleop_mujoco/recalibrate`, `/arm_teleop_mujoco/start`.
- `scripts/recording_keyboard_client.py` as the keyboard client.

Ubuntu 22.04 caution:

- Do not treat Ubuntu 22.04 + ROS Noetic as equivalent to native Ubuntu 20.04 + Noetic.
- For 22.04, use an Ubuntu 20.04 container/VM, RoboStack, source builds, or a ROS2/non-ROS refactor. All require extra validation.

## 6. MuJoCo / OpenGL Requirements

Code uses official MuJoCo Python bindings exclusively:

- `import mujoco`
- `import mujoco.viewer`
- `mujoco.MjModel.from_xml_path`
- `mujoco.MjData`
- `mujoco.mj_step`
- `mujoco.mj_forward`
- `mujoco.mj_name2id`, `mj_id2name`
- `mujoco.Renderer`
- `mujoco.viewer.launch_passive`
- `mujoco.mjv_initGeom`, `mjv_connector`
- `mujoco.mjtObj`, `mujoco.mjtCamera`, `mujoco.mjtGeom`

No `mujoco-py` usage was found.

Current installed MuJoCo version: not available in current shell.

Recommended pinned MuJoCo version: `mujoco==3.2.6` as a conservative modern official binding. Pin and validate with smoke tests because the repo has no existing MuJoCo pin.

Version-sensitive APIs:

- `mujoco.Renderer(model, height=..., width=...)`
- `renderer.update_scene(data, camera=cam_id)`
- `renderer.render()`
- `mujoco.viewer.launch_passive(model, data)`
- direct array fields like `model.sensor_adr`, `model.sensor_dim`, `model.sensor_objid`, `data.sensordata`
- `mujoco.mjv_initGeom` and `mujoco.mjv_connector`

Rendering architecture:

- Optional default MuJoCo viewer: `mujoco.viewer.launch_passive`.
- Live OpenCV camera windows: separate `mujoco.Renderer` in controller.
- HDF5 image recording: separate `mujoco.Renderer` in recorder.
- Disabling the default viewer should not disable camera rendering, because the controller and recorder create their own renderers. It still requires a valid OpenGL/offscreen backend.
- The OpenCV HUD is display-only. HDF5 images are rendered through the recorder renderer before the HUD path, so HUD contamination should not occur.

MuJoCo XML:

- `model/pangu_all_right.xml`
- timestep `0.001`, implying 1000 Hz physics.
- camera names: `ee_cam`, `base_top_cam`, `cctv_cam`, plus `wall_task_cam`, `wall_side_cam`.
- FT sensors: `peg_ft_force`, `peg_ft_torque` on `ft_sensor_site`.
- mesh directory: `../meshes_mujoco_stable/` relative to XML.

Linux graphics backend notes:

| Scenario | Suggested `MUJOCO_GL` | Notes |
|---|---|---|
| Normal desktop display | unset or `glfw` | Requires X11/desktop OpenGL and GLFW. |
| SSH with X forwarding | `glfw` may work | Often fragile/slow; OpenCV GUI also needs X forwarding. |
| Headless CPU/software | `osmesa` | No OpenCV windows unless using virtual display; install OSMesa. |
| Headless NVIDIA | `egl` | Validate NVIDIA driver/EGL stack. OpenCV windows still need display or must be disabled/refactored. |
| Mesa software display | `glfw` or `osmesa` | Depends on whether X/Wayland display exists. |

No rendering backend was changed during this audit.

Official MuJoCo docs confirm the package is installed with `pip install mujoco`, includes its MuJoCo library, provides `mujoco.viewer`, and requires an OpenGL context for rendering: <https://mujoco.readthedocs.io/en/stable/python.html>.

## 7. OpenCV GUI Requirements

The runtime uses OpenCV windows directly:

- `cv2.namedWindow`
- `cv2.imshow`
- `cv2.waitKey`
- `cv2.resizeWindow`
- `cv2.setWindowProperty`
- `cv2.WINDOW_NORMAL`
- `cv2.WINDOW_FULLSCREEN`

Deployment must use GUI-enabled OpenCV. Do not install:

- `opencv-python-headless`
- `opencv-contrib-python-headless`

The PyPI OpenCV package page explicitly says headless packages do not contain GUI functionality and should only be used when not using `cv2.imshow` and similar APIs: <https://pypi.org/project/opencv-python/>.

Current installed OpenCV backend could not be checked because `cv2` is absent in the current shell.

Likely Linux GUI dependencies:

- X11 libraries: `libx11-6`, `libxext6`, `libxrender1`, `libsm6`, `libxcb-xinerama0`.
- GL/GLib: `libgl1`, `libglib2.0-0`.
- Qt/GTK depending on wheel/build. Non-headless pip wheels ship Qt; distro builds may use GTK.

## 8. HDF5 / Dataset Requirements

Recorder implementation: `vptele/utils/mujoco_hdf5_recorder.py`.

Runtime dependencies:

- `h5py`
- `numpy`
- `mujoco.Renderer` for image tensors
- local write permissions under `hdf5_record_dir`

Schema highlights:

- HDF5 root attrs include `schema_version=compact_mujoco_hdf5_v1`, rates, model path, and schema JSON.
- State stream at 30 Hz: `observations/ee_pose`, `joint_pos`, `joint_vel`, `joint_torque`, optional `action`.
- Force stream at 500 Hz: `observations/ft_wrench`, optional `ft_wrench_raw`, `ft_wrench_gravity`.
- Images at 30 Hz: `observations/images/<camera>` as `uint8` RGB tensors, chunked by frame, default LZF compression.
- Events: string dataset `events/names` plus simulation/episode/wall timestamps.
- String datasets use `h5py.string_dtype(encoding="utf-8")`.

File lifecycle:

- `start_episode()` creates `<output_dir>/<timestamp>_<label>/episode.hdf5`.
- `stop_episode()` adds event, sets inactive, writes final metadata/events, flushes, closes, and writes `metadata.json`.
- `close()` stops active recording with `controller_shutdown` and closes renderer.
- HDF5 writes are protected by `threading.RLock`.

Robustness notes:

- Clean shutdown through `disconnect()` should close HDF5.
- Hard process kill or power loss can corrupt the active file because data is appended directly and flush occurs at stop.
- Network-mounted storage may block the simulation thread, because HDF5 appends occur inside the post-step path.

Disk estimate:

- Two 640x480 RGB cameras at 30 Hz raw: about 55 MB/s before compression.
- Force/state numeric data is small by comparison.
- With LZF, real size depends heavily on scene entropy; plan conservatively for 2-4 GB per minute if compression is weak, and validate on target.
- SSD/NVMe storage is recommended.

## 9. Vision Pro / Network Stream Requirements

Current MuJoCo path uses `vptele/core/vp_streamer_avp.py`, which imports:

```python
from avp_stream import VisionProStreamer as AVPStreamer
```

Config:

- `vp_ip: "192.168.1.144"`
- `vp_record: false`
- `vuer: false`

The AVP wrapper passes `ip` and `record` to `VisionProStreamer`. Ports, TLS/certificates, and transport details are hidden inside `avp_stream` and could not be verified from repository code. The README notes `avp_stream` may require environment-specific/private installation.

Alternative `vp_streamer_vuer.py` exists:

- hardcodes `http://localhost:5301`, ignoring the `ip` argument.
- uses `requests`.
- expects `/health` and `/api/vision_data`.
- README references a Vuer URL with HTTPS/WSS on port 8012 and data server port 5301.

Firewall/network requirements:

- Vision Pro and Linux machine must be on the same reachable network.
- Allow AVP stream package ports after inspecting `avp_stream`.
- If using Vuer mode, allow TCP 5301 and 8012.
- ROS network variables may be needed for multi-machine ROS: `ROS_MASTER_URI`, `ROS_IP`, or `ROS_HOSTNAME`. The repo does not set them.

## 10. Dependency Inventory

| Dependency | Type | Imported or used by | Current version | Recommended version | Required | Python compatibility | Ubuntu compatibility | Risk notes |
|---|---|---|---|---|---:|---|---|---|
| Python | system/conda | all Python scripts | shell: 3.9.6; no `python` | 3.8 on Noetic | required | 3.8 safest for ROS1 Noetic | Ubuntu 20.04 native | Python 3.13 env file is risky/blocking. |
| ROS1 / rospy | ROS apt | `main_mujoco`, controller, keyboard client | not installed locally | ROS Noetic | required | Python 3.8 native | Ubuntu 20.04 | ROS2 not compatible without refactor. |
| roscpp/catkin | ROS apt | C++ IK services | not installed locally | Noetic catkin | required | N/A | Ubuntu 20.04 | Build depends on local ArmKinematics. |
| std_msgs/std_srvs | ROS apt | messages/services | not installed locally | Noetic | required | Python 3.8 | Ubuntu 20.04 | `std_srvs/Trigger` used. |
| geometry_msgs | ROS apt | `ArmIK.srv`, tests | not installed locally | Noetic | required | Python 3.8 | Ubuntu 20.04 | Listed in CMake, not package.xml build deps explicitly. |
| sensor_msgs | ROS apt | package metadata | not installed locally | Noetic | likely required | Python 3.8 | Ubuntu 20.04 | Listed in package/CMake. |
| trajectory_msgs | ROS apt | package metadata | not installed locally | Noetic | likely required | Python 3.8 | Ubuntu 20.04 | Listed in package/CMake. |
| MuJoCo `mujoco` | pip/conda | controller, recorder, model tools | not installed locally | `mujoco==3.2.6` pending validation | required | wheels support modern Python; use 3.8/3.9 for ROS | Linux | Missing from env files. |
| `mujoco.viewer` | pip | default viewer | not installed locally | same as `mujoco` | optional but enabled by default | same | needs GUI/OpenGL | `launch_viewer` is hardcoded true in teleop config construction. |
| NumPy | pip/conda | all numeric code | env YAML: 2.0.2 | 1.24.* with Py3.8/Noetic | required | 1.24 supports Py3.8 | Linux wheels | NumPy 2 can break older compiled extensions. |
| SciPy | pip/conda | `Rotation` in teleop | commented/absent | 1.10.* for Py3.8 | required | Py3.8-compatible line needed | Linux wheels | Missing active pin. |
| OpenCV | pip/conda | live windows and HUD | env YAML: opencv-python 4.9.0.80 + contrib 4.10.0.82 | one GUI package, e.g. 4.9.0.80 | required | supports Py3.x wheels | needs GUI libs | Do not install headless; avoid both regular and contrib simultaneously. |
| h5py | pip/conda | HDF5 recorder/scripts | missing | 3.10.* | required | Py3.8-compatible | Linux wheels/conda | Missing from env files. |
| HDF5 native | apt/conda | h5py backend | unknown | conda hdf5 or apt libhdf5 | likely required | N/A | Ubuntu packages | Depends on install route. |
| PyYAML | pip/conda | config loading | env YAML: 6.0.2 | 6.0.2 | required | Py3.8+ | Linux wheels | OK. |
| matplotlib | pip/conda | analysis scripts | env YAML: 3.9.4 | optional | optional | Py3.8 version constraints matter | Linux wheels | Not needed for runtime collection. |
| imageio | pip | old recorder only | absent | optional | optional | Py3.8+ | Linux wheels | Current recorder does not use it. |
| requests | pip | Vuer streamer | env YAML: 2.32.3 | 2.32.* | optional | broad | Linux wheels | Only Vuer path. |
| avp-stream | pip/private | AVP streamer | env YAML: 1.0 | exact package source unresolved | required | unknown | unknown | Reproducibility blocker. |
| robotic-arm | pip/private | real-arm paths | env YAML: 1.0.6 | optional | optional | unknown | unknown | Not used by current peg MuJoCo path. |
| Ceres | apt/source | IK C++ nodes | `/usr/lib/libceres.so` expected | Ubuntu package or matching source | required | N/A | 20.04 package available | Hardcoded path. |
| Eigen3 | apt | IK C++ nodes | unknown | `libeigen3-dev` | required | N/A | Ubuntu | Required by CMake. |
| yaml-cpp | apt | IK C++ nodes | unknown | `libyaml-cpp-dev` | required | N/A | Ubuntu | Required by CMake. |
| ArmKinematics | local source/install | IK C++ nodes | expected `/usr/local/lib/libarm_kinematics.so` | local install with CMake config | required | N/A | target-local | Not reconstructable from standard pip/apt. |
| FFmpeg | wheel/apt | OpenCV/analysis | wheel bundled | optional apt `ffmpeg` | optional | N/A | Ubuntu | Useful for inspection. |

## 11. Native apt Dependencies

See [ubuntu_system_packages.md](ubuntu_system_packages.md) for grouped package lists with status labels.

Critical apt/native items for Ubuntu 20.04:

- ROS Noetic/catkin/message packages.
- `build-essential`, `cmake`, `git`.
- `libeigen3-dev`, `libceres-dev`, `libsuitesparse-dev`, `libyaml-cpp-dev`.
- GUI/OpenGL stack: X11, GLFW, Mesa/OpenGL, optional EGL/OSMesa.
- OpenCV GUI support libraries.
- Local ArmKinematics library installed under the path expected by `CMakeLists.txt`.

## 12. Python Dependencies

Required for the MuJoCo recording path:

```text
numpy
scipy
pyyaml
mujoco
opencv-python or opencv-contrib-python
h5py
avp-stream
requests only for Vuer mode
matplotlib only for analysis scripts
imageio only for old recorder
```

Do not install both `opencv-python` and `opencv-contrib-python` unless there is a specific reason. The current env files list both, with different versions; that can cause import ambiguity.

## 13. Non-Portable Paths

| File | Line | Current value | Why non-portable | Recommended portable replacement |
|---|---:|---|---|---|
| `vptele/config/config_arm_right_peg.yaml` | 3 | `/home/stw/pangu/src/arm_teleop/data/peg_in_hole` | Host/user-specific data path | Use ROS param/env var, e.g. `${ARM_TELEOP_DATA_DIR}/peg_in_hole`. |
| `vptele/config/config_arm_right_peg.yaml` | 11 | `/home/stw/pangu/src/arm_teleop/data/peg_hole_try` | Host/user-specific HDF5 path | Use `~/.local/share/arm_teleop/peg_hole_try` or configured dataset root. |
| `vptele/config/config_arm_right_peg.yaml` | 63 | `/home/stw/pangu/src/arm_teleop/model/pangu_all_right.xml` | Host/user-specific model path | Resolve from ROS package path or repo root. |
| `vptele/config/config_arm_right_peg.yaml` | 160 | `192.168.1.144` | Site-specific Vision Pro IP | ROS param/env var or per-machine config overlay. |
| `vptele/core/teleop_system_mujoco.py` | 209 | `/home/stw/pangu/src/arm_teleop/data/peg_in_hole` | Hardcoded fallback | Compute from package/repo path or require config. |
| `vptele/core/teleop_system_mujoco.py` | 509 | `/home/stw/pangu/src/arm_teleop/data/peg_in_hole_hdf5` | Hardcoded fallback | Configurable dataset root. |
| `vptele/core/teleop_system_mujoco.py` | 634 | `/home/stw/pangu/src/arm_teleop/model/pangu_all_right.xml` | Hardcoded fallback | Resolve `model/pangu_all_right.xml` relative to package. |
| `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` | 351 | `/home/stw/pangu/src/arm_teleop/data/peg_in_hole` | Hardcoded fallback | Configurable dataset root. |
| `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` | 372 | `/home/stw/pangu/src/arm_teleop/data/peg_in_hole_hdf5` | Hardcoded fallback | Configurable dataset root. |
| `vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py` | 3098 | `/home/stw/pangu/src/arm_teleop/model/pangu_all_right.xml` | Standalone test path | Resolve relative to file/repo. |
| `CMakeLists.txt` | 41 | `/usr/local/lib/cmake/ArmKinematics/**` | Local install path | Use `CMAKE_PREFIX_PATH` or package config variable. |
| `CMakeLists.txt` | 243, 256 | `/usr/local/lib/libarm_kinematics.so` | Local install path | Link imported target from `find_package(ArmKinematics)`. |
| `CMakeLists.txt` | 244, 257 | `/usr/lib/libceres.so` | Distro/path-specific | Link `Ceres::ceres` from CMake package. |
| `vptele/core/vp_streamer_vuer.py` | 23 | `http://localhost:5301` | Ignores passed IP/port | Use `f"http://{ip}:{port}"`. |
| `vptele/environment.yaml` | 72 | `/home/pangu/miniconda3/envs/vrtele` | User-specific prefix | Remove `prefix` from shared env file. |
| `config/environment.yaml` | 61 | `/home/stw/miniconda3/envs/vrtele` | User-specific prefix | Remove `prefix` from shared env file. |
| `test/display_model.py` | 5 | `/home/stw/pangu/src/arm_teleop/model/Arm_right_15.xml` | Missing/host-specific model | Use repo model path. |
| `test/dataset_test.py` | 28 | `/home/stw/pangu/src/arm_teleop/data/.../episode.hdf5` | Local dataset path | CLI argument. |
| `test/play_multi.py` | 26-33 | `/home/pangu/pangu/src/arm_teleop/demo/*.bag` | Local bag paths | CLI arguments or package-relative demo dir. |
| `test/play_multi.py` | 42 | `/home/pangu/pangu/src/aiui/audio/task9.mp3` | Local audio path | Configurable path. |
| `vptele/arm_control/arm_teleop_ros.py` | 229 | `/home/pangu/pangu/src/arm_teleop/data_log` | Local CSV path | Configurable log/data root. |
| `vptele/arm_control/robot_controller_mujoco.py` | 240 | `/home/pangu/pangu/src/arm_teleop/model/right_arm_stable.xml` | Standalone test path | Resolve relative to package. |
| `vptele/quick_start.md` | 8 | `/home/rm/thomasyht/vptele` | Documentation path | Replace with `<catkin_ws>/src/arm_teleop/vptele`. |

## 14. Performance Requirements and Risks

Observed architecture:

- MuJoCo physics timestep: 0.001 s, so simulation targets 1000 Hz.
- HDF5 force samples: 500 Hz.
- HDF5 state samples: 30 Hz.
- HDF5 image samples: 30 Hz.
- Live monitor camera stream defaults to 15 Hz in code, even though image recording is 30 Hz.
- Live CCTV/HUD display and HDF5 images use separate renderers.
- `cv2.imshow`/`cv2.waitKey(1)` run in the simulation/viewer thread after physics, recording, and alarm checks.

Resource recommendations:

- CPU: modern 6+ core desktop CPU recommended; strong single-thread performance matters for 1000 Hz stepping plus Python work.
- RAM: 16 GB minimum, 32 GB recommended for long sessions and analysis.
- GPU/OpenGL: integrated graphics may work for 640x480 multi-camera rendering, but discrete GPU improves margin. NVIDIA is not strictly required unless using EGL/headless GPU rendering.
- Disk: local SSD/NVMe strongly recommended. Avoid network-mounted storage for active recording.
- CPU governor: use performance mode for data collection if timing jitter appears.
- Thread variables: set `OMP_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, `OPENBLAS_NUM_THREADS=1` if NumPy/SciPy oversubscription causes jitter.
- Remote desktop/SSH: X forwarding and remote desktop can break or slow MuJoCo/OpenCV windows. Prefer local display, VirtualGL, or validated EGL/OSMesa plus disabled GUI paths if headless.

Risk notes:

- Multiple renderers increase GL context and GPU load.
- HDF5 image writes occur inside the recording path and can block the simulation loop.
- Fullscreen OpenCV windows can introduce display-server stalls.
- Python scheduling and GIL can affect service callbacks, GUI calls, and HDF5 writes, though `mj_step` releases the GIL inside MuJoCo.

## 15. Deployment Recommendations

Recommended route:

- Ubuntu: 20.04.
- ROS: Noetic.
- Python: 3.8 for ROS interop.
- Environment manager: Conda or venv, but verify `rospy` and generated `arm_teleop` messages/services import from the chosen interpreter.
- MuJoCo: official `mujoco` Python package pinned and smoke-tested, proposed `3.2.6`.
- OpenCV: GUI-enabled `opencv-python` or `opencv-contrib-python`, not headless.
- Graphics: local desktop X11/GLFW first; only use EGL/OSMesa after smoke tests.
- Storage: local SSD/NVMe.

Alternative Ubuntu 22.04 route:

- Use Docker/VM with Ubuntu 20.04 + ROS Noetic.
- Or use RoboStack/source builds with careful validation.
- Do not deploy native Ubuntu 22.04 + ROS Noetic as though it were the same as Ubuntu 20.04.

Containerized route:

- Good for dependency reproducibility.
- Harder for GUI/OpenGL/OpenCV windows and Vision Pro/ROS networking.
- Needs X11/Wayland socket or EGL/NVIDIA runtime configuration.

Headless route:

- Possible for HDF5 rendering with `MUJOCO_GL=osmesa` or `egl`.
- Current behavior includes OpenCV operator windows and optional viewer; fully headless collection would require disabling/refactoring GUI behavior, so it is not behavior-preserving.

No-default-MuJoCo-viewer route:

- The code can run camera renderers without the passive viewer if `launch_viewer` is false, but current `TeleopSystemMujoco` hardcodes `launch_viewer: True`.
- Changing that is a behavior/runtime change and was not done in this audit.

## 16. Deployment Checklist

1. Install Ubuntu 20.04 with working desktop/OpenGL.
2. Install ROS Noetic and catkin tools.
3. Create catkin workspace and clone this repo as `src/arm_teleop`.
4. Install native C++ dependencies and the local ArmKinematics library.
5. Build catkin workspace and source `devel/setup.bash`.
6. Create Python environment with Python 3.8 and required pip/conda packages.
7. Ensure `python -c "import rospy; import arm_teleop.srv"` works in the same shell used for launch.
8. Set dataset output directory to a writable local SSD path.
9. Set `vp_ip` to the actual Vision Pro IP.
10. Validate MuJoCo XML asset paths and camera/sensor names.
11. Run staged smoke tests below.
12. Record a short episode and run quality check.

## 17. Smoke-Test Checklist

Stage A: Python imports

```bash
python -c "import numpy, h5py, yaml, cv2, mujoco"
python -c "import rospy; from arm_teleop.srv import SetRecording, ArmIK"
python -c "from avp_stream import VisionProStreamer"
```

Stage B: MuJoCo XML load

```bash
python - <<'PY'
from pathlib import Path
import mujoco
model_path = Path("model/pangu_all_right.xml").resolve()
m = mujoco.MjModel.from_xml_path(str(model_path))
d = mujoco.MjData(m)
print("loaded", model_path)
print("nq", m.nq, "nv", m.nv, "nu", m.nu, "nsensor", m.nsensor)
PY
```

Stage C: OpenCV GUI

```bash
python - <<'PY'
import numpy as np
import cv2
img = np.zeros((240, 320, 3), dtype=np.uint8)
cv2.putText(img, "OpenCV GUI OK", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
cv2.imshow("opencv_smoke_test", img)
print("Press any key in the OpenCV window")
cv2.waitKey(0)
cv2.destroyAllWindows()
PY
```

Stage D: MuJoCo camera render

```bash
python - <<'PY'
from pathlib import Path
import mujoco
model = mujoco.MjModel.from_xml_path(str(Path("model/pangu_all_right.xml").resolve()))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
renderer = mujoco.Renderer(model, height=480, width=640)
for name in ["cctv_cam", "ee_cam", "base_top_cam"]:
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, name)
    assert cam_id != -1, name
    renderer.update_scene(data, camera=cam_id)
    frame = renderer.render()
    print(name, frame.shape, frame.dtype, int(frame.mean()))
renderer.close()
PY
```

Stage E: FT sensor check

```bash
python - <<'PY'
from pathlib import Path
import mujoco
model = mujoco.MjModel.from_xml_path(str(Path("model/pangu_all_right.xml").resolve()))
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
for name in ["peg_ft_force", "peg_ft_torque"]:
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
    assert sid != -1, name
    adr = int(model.sensor_adr[sid])
    dim = int(model.sensor_dim[sid])
    site_id = int(model.sensor_objid[sid])
    site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, site_id)
    print(name, "site", site_name, "value", data.sensordata[adr:adr+dim])
site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ft_sensor_site")
assert site != -1
print("ft_sensor_site", data.site_xpos[site])
PY
```

Stage F: ROS check

```bash
roscore
```

In another shell:

```bash
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash
rosrun arm_teleop ik_service_right_node
```

In another shell:

```bash
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash
python -c "import rospy; from arm_teleop.srv import SetRecording, ArmIK; from std_srvs.srv import Trigger"
rosservice list | grep -E "right_arm_ik_srv|arm_teleop_mujoco|mujoco_hdf5"
```

Stage G: teleoperation without recording

```bash
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash
rosrun arm_teleop main_mujoco.py _config_path:=config/config_arm_right_peg.yaml
```

Verify:

- CCTV window appears.
- Force HUD appears on `cctv_cam`.
- Auxiliary camera stream window appears as configured.
- Default MuJoCo viewer appears unless disabled by a later approved config/code change.

Stage H: short recording

```bash
source /opt/ros/noetic/setup.bash
source <catkin_ws>/devel/setup.bash
rosrun arm_teleop recording_keyboard_client.py
```

Record 5-10 seconds. Then verify:

```bash
python - <<'PY'
from pathlib import Path
import h5py
import numpy as np
episode = Path("PATH_TO_EPISODE/episode.hdf5")
with h5py.File(episode, "r") as h5:
    for p in [
        "timestamps/force",
        "timestamps/state",
        "timestamps/image",
        "observations/ft_wrench",
        "observations/ee_pose",
        "observations/joint_pos",
        "observations/images/ee_cam",
        "observations/images/base_top_cam",
        "events/names",
    ]:
        print(p, h5[p].shape)
    for p in ["timestamps/force", "timestamps/state", "timestamps/image"]:
        t = h5[p][:]
        print(p, "monotonic", bool(np.all(np.diff(t) >= -1e-12)), "rate approx", (len(t)-1)/(t[-1]-t[0]) if len(t) > 1 else 0)
PY
```

Stage I: quality check

```bash
python scripts/check_hdf5_episode_quality.py --episode PATH_TO_EPISODE/episode.hdf5
```

## 18. Known Blockers

- No verified project environment exists on the current machine.
- `mujoco`, `h5py`, and active `scipy` are missing from committed env files.
- `config/environment.yaml` pins Python 3.13.5, incompatible with the recommended ROS1 Noetic route.
- `avp_stream` package details are opaque; cannot confirm standard pip reconstruction.
- Local ArmKinematics library is required but not provided by this repo.
- Hardcoded absolute paths must be overridden or fixed before deployment.
- OpenCV build backend could not be verified locally.
- MuJoCo version could not be verified locally.

## 19. Items That Could Not Be Verified

- Current installed MuJoCo version.
- Current installed OpenCV GUI backend.
- Current installed h5py/NumPy/SciPy versions.
- AVP network ports, TLS/certificate requirements, package source, and Python compatibility.
- Whether the target Linux GPU/display stack supports the chosen MuJoCo backend.
- Runtime write throughput on the target dataset filesystem.
- Whether `ArmKinematics` is available on the target.

## 20. Commands Run

```bash
pwd
git branch --show-current
git status --short
git remote -v
ls
find .. -name CODEX_RULES.md -print
sed -n '1,240p' ../ForceAwareACT/CODEX_RULES.md
find . -maxdepth 4 -type f
find . -iname requirements*.txt -o -iname environment*.yml -o -iname environment*.yaml -o -iname setup.py -o -iname pyproject.toml -o -iname package.xml -o -iname CMakeLists.txt
rg -n "^(import|from) " vptele scripts src test
rg -n "rospy|rosservice|std_srvs|sensor_msgs|geometry_msgs|cv_bridge|ros_numpy" .
rg -n "mujoco|mujoco\.viewer|Renderer|GLFW|OpenGL|EGL|OSMesa" .
python --version
python3 --version
which python
which python3
conda info --envs
python -m pip --version
python3 -m pip --version
python3 -m pip freeze
rg --files -g 'requirements*.txt' -g 'environment*.yml' -g 'environment*.yaml' -g 'setup.py' -g 'pyproject.toml' -g 'package.xml' -g 'CMakeLists.txt'
rg -n "cv2\.imshow|cv2\.namedWindow|cv2\.waitKey|WINDOW_NORMAL|WINDOW_FULLSCREEN" .
rg -n "h5py|HDF5|imageio|numpy|scipy|yaml|torch|avp|socket|websocket|zmq" .
rg -n "/Users/|/home/|Desktop|Documents|Downloads" .
nl -ba vptele/environment.yaml
nl -ba config/environment.yaml
nl -ba package.xml
nl -ba CMakeLists.txt
nl -ba setup.py
nl -ba launch/teleop_service.launch
python3 - <<'PY'
mods=['numpy','h5py','yaml','cv2','mujoco','scipy','matplotlib','avp_stream','requests']
import importlib
for m in mods:
    try:
        mod=importlib.import_module(m)
        print(f'{m}:', getattr(mod,'__version__','imported'))
    except Exception as exc:
        print(f'{m}: FAILED {exc!r}')
PY
python3 - <<'PY'
try:
    import cv2
    print('cv2 version:', cv2.__version__)
    print(cv2.getBuildInformation())
except Exception as exc:
    print('OpenCV inspection failed:', repr(exc))
PY
python3 - <<'PY'
try:
    import mujoco
    print('mujoco version attr:', getattr(mujoco, '__version__', None))
    try:
        print('mj_version:', mujoco.mj_version())
        print('mj_versionString:', mujoco.mj_versionString())
    except Exception as e:
        print('mj_version check failed:', repr(e))
except Exception as exc:
    print('MuJoCo inspection failed:', repr(exc))
PY
rg -n "PYTHONPATH|LD_LIBRARY_PATH|MUJOCO_GL|DISPLAY|ROS_MASTER_URI|ROS_IP|ROS_HOSTNAME" .
rg -n "VisionProStreamer|vp_ip|avp_ip|recording_service_name|camera|cctv|hdf5_record_dir|mujoco_model_path|roslaunch|rosrun|catkin|source /opt/ros|source .*devel" .
nl -ba vptele/main_mujoco.py
nl -ba vptele/core/teleop_system_mujoco.py
nl -ba scripts/recording_keyboard_client.py
nl -ba vptele/core/vp_streamer_avp.py
nl -ba vptele/core/vp_streamer_vuer.py
nl -ba vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py | sed -n '1,460p'
nl -ba vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py | sed -n '900,1105p'
nl -ba vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py | sed -n '1460,1668p'
nl -ba vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py | sed -n '2130,2188p'
nl -ba vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py | sed -n '2290,2688p'
nl -ba vptele/arm_control/robot_controller_mujoco_peg_tool_contact.py | sed -n '2930,3055p'
nl -ba vptele/utils/mujoco_hdf5_recorder.py | sed -n '1,230p'
nl -ba vptele/utils/mujoco_hdf5_recorder.py | sed -n '258,355p'
nl -ba vptele/utils/mujoco_hdf5_recorder.py | sed -n '355,425p'
nl -ba vptele/utils/mujoco_hdf5_recorder.py | sed -n '426,760p'
nl -ba vptele/utils/mujoco_hdf5_recorder.py | sed -n '760,940p'
nl -ba vptele/utils/ft_wrench_utils.py
nl -ba vptele/utils/force_feedback_overlay.py | sed -n '1,220p'
nl -ba vptele/config/config_arm_right_peg.yaml
nl -ba model/pangu_all_right.xml
find model meshes_mujoco_stable -maxdepth 2 -type f
find msg srv -maxdepth 2 -type f -print
for f in msg/*.msg srv/*.srv; do printf '\n### %s\n' "$f"; nl -ba "$f"; done
nl -ba vptele/arm_control/arm_teleop_mujoco.py | sed -n '1,140p'
nl -ba vptele/arm_control/arm_teleop_mujoco.py | sed -n '140,260p'
nl -ba README.md
nl -ba vptele/README.md | sed -n '60,155p'
nl -ba config/kinematics_params.yaml
mkdir -p docs
git diff --check
git status --short
python3 - <<'PY'
from pathlib import Path
try:
    import yaml
except Exception as exc:
    print('YAML validation failed: cannot import yaml:', repr(exc))
    raise SystemExit(1)
path = Path('docs/environment_recommendation.yml')
if path.exists():
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    print('environment_recommendation.yml parses:', isinstance(data, dict))
PY
ruby -e 'require "yaml"; data = YAML.load_file("docs/environment_recommendation.yml"); puts "environment_recommendation.yml parses with ruby yaml: #{data.is_a?(Hash)}"'
git diff -- docs/environment_recommendation.yml docs/ubuntu_system_packages.md docs/linux_deployment_audit.md
```

External documentation consulted:

- MuJoCo Python documentation: <https://mujoco.readthedocs.io/en/stable/python.html>
- OpenCV Python PyPI package notes: <https://pypi.org/project/opencv-python/>
- ROS wiki/docs pages were attempted but blocked by the site protection page during this audit.
