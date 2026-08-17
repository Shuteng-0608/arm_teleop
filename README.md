# Pangu Arm Teleopration System via Apple Vision Pro  

## ROS-free MuJoCo environment

Create the project-specific Python 3.10 environment from the repository root:

```bash
conda env create -f environment.yml
conda activate arm_teleop
```

If the environment already exists, synchronize it with the checked-in file:

```bash
conda env update -n arm_teleop -f environment.yml --prune
```

This environment contains the MuJoCo simulator, Vision Pro stream client,
HDF5/image tooling, analysis libraries, and the ROS-free test dependencies. It
does not install ROS, `rospy`, catkin, or generated ROS message packages.

Load the moving-hole model in the viewer:

```bash
python scripts/view_mujoco_model.py \
  --model model/pangu_moving_hole_fixed_peg.xml
```

## ROS-free Vision Pro teleoperation

Start human teleoperation and manual HDF5 collection without ROS:

```bash
conda activate arm_teleop
python -m vptele.main_vr \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --vp-ip 172.20.10.2
```

Use `r` (or Enter) to reset/calibrate/start an episode, `k` to keep it, `d`
to discard it, `c` to recalibrate, and `q` to shut down safely. For hardware-
free commissioning, add `--synthetic`; add `--headless` to disable all local
windows. See [docs/vr_teleop_python.md](docs/vr_teleop_python.md).

## Legacy ROS teleoperation

1. Ensure ROS Master is running
```bash
roscore
```
2. Launch the arm inverse kinematics service:
```bash
rosrun arm_teleop ik_service_right_node
```
3. Launch the arm mujoco sim teleopration node:
```bash
rosrun arm_teleop main_mujoco.py
```

## 在新电脑部署并进行自动数据采集

下面的流程从新电脑克隆代码开始，适用于 Ubuntu x86_64。自动脚本采集
完全使用 Python 和 MuJoCo，不需要 ROS、`roscore`、Vision Pro 或 C++ IK
节点。所有命令除特别说明外，都在终端中依次执行。

### 1. 安装系统依赖

```bash
sudo apt update
sudo apt install -y \
  git \
  curl \
  bzip2 \
  libgl1 \
  libegl1 \
  libglfw3 \
  libglib2.0-0 \
  libx11-6 \
  libxrandr2 \
  libxinerama1 \
  libxcursor1 \
  libxi6
```

### 2. 克隆包含采集功能的分支

```bash
mkdir -p ~/ForceAwareACT_workspace
cd ~/ForceAwareACT_workspace

git clone \
  --branch feature/moving-hole-fixed-peg \
  --single-branch \
  https://github.com/Shuteng-0608/arm_teleop.git

cd arm_teleop
git branch --show-current
git log -3 --oneline
```

`git branch --show-current` 应输出：

```text
feature/moving-hole-fixed-peg
```

确认模型、配置和环境文件齐全：

```bash
test -f model/pangu_moving_hole_fixed_peg.xml
test -f vptele/config/config_arm_right_moving_hole.yaml
test -f environment.yml
echo "repository files OK"
```

### 3. 安装 Miniconda（已经安装可跳过）

```bash
cd /tmp
curl -O \
  https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh

bash Miniconda3-latest-Linux-x86_64.sh \
  -b \
  -p "$HOME/miniconda3"

source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda init bash
```

重新打开一个终端，或者在当前终端继续执行：

```bash
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda --version
```

### 4. 创建专用 Python 3.10 环境

```bash
cd ~/ForceAwareACT_workspace/arm_teleop

conda env create -f environment.yml
conda activate arm_teleop
```

如果 `arm_teleop` 环境已经存在，不要重复创建，改为同步环境：

```bash
conda env update \
  --name arm_teleop \
  --file environment.yml \
  --prune

conda activate arm_teleop
```

不需要运行 `pip install -e .`。当前自动采集入口应直接从仓库根目录以
`python -m vptele...` 的形式启动。

确认关键依赖：

```bash
python -c "
import sys, mujoco, numpy, scipy, h5py, cv2, yaml
print('python ', sys.version.split()[0])
print('mujoco', mujoco.__version__)
print('numpy  ', numpy.__version__)
print('scipy  ', scipy.__version__)
print('h5py   ', h5py.__version__)
print('opencv ', cv2.__version__)
print('yaml   ', yaml.__version__)
"
```

### 5. 运行无 ROS 回归测试

```bash
cd ~/ForceAwareACT_workspace/arm_teleop
conda activate arm_teleop

PYTHONPATH=. python -m pytest -q test \
  --ignore=test/test_hole_center.py \
  --ignore=test/test_ik.py \
  --ignore=test/test_visionpro_video.py
```

这三个被排除的文件是遗留 ROS/旧硬件测试，不属于纯 Python 自动采集
链路。提交 `1e832ba` 对应的基线结果为 `75 passed`；后续提交增加测试时，
通过数量可能继续增加，但不应出现失败。

### 6. 配置 MuJoCo 图形后端

如果新电脑是带显示器的桌面环境，通常不需要设置环境变量。可先打开
模型验证：

```bash
python scripts/view_mujoco_model.py \
  --model model/pangu_moving_hole_fixed_peg.xml
```

如果通过 SSH 在 NVIDIA 机器上无头运行，使用 EGL：

```bash
export MUJOCO_GL=egl
```

如果是没有 GPU 的纯 CPU 服务器，安装并使用 OSMesa：

```bash
sudo apt install -y libosmesa6
export MUJOCO_GL=osmesa
```

即使没有传入 `--show-ui`，采集器仍需渲染 `ee_cam` 和
`base_top_cam` 图像，因此无头机器也需要可用的 EGL 或 OSMesa 后端。

### 7. 先各采集一条进行验收

先验证纯净、无主动碰撞模式：

```bash
cd ~/ForceAwareACT_workspace/arm_teleop
conda activate arm_teleop

python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 1 \
  --max-attempts 3 \
  --scenario clean \
  --reject-action quarantine \
  --show-ui
```

确认以下现象：

- 末端圆环孔与机械臂连接正常；
- 孔以零 X/Z 偏移接近固定 peg；
- 洞口没有主动碰撞和碰撞后回退；
- 插入过程中没有孔内扰动；
- 成功后 terminal hold 没有明显大力；
- 最终日志出现 `scripted_replay_success`；
- 自动质量检查结果为 accepted。

再验证碰撞与恢复模式：

```bash
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 1 \
  --max-attempts 3 \
  --scenario collision \
  --reject-action quarantine \
  --show-ui
```

确认洞口偏移接触、回撤对齐、孔内横向扰动、力反馈恢复和最终成功均
正常。无显示器时删除两个命令末尾的 `--show-ui`。

### 8. 检查单条数据

默认输出目录为：

```text
data/moving_hole_fixed_peg/
```

查看最新目录和批次结果：

```bash
find data/moving_hole_fixed_peg \
  -maxdepth 1 \
  -type d \
  -name '*_scripted' \
  | sort \
  | tail

find data/moving_hole_fixed_peg/batch_manifests \
  -type f \
  -name 'batch_*.jsonl' \
  | sort \
  | tail -n 1 \
  | xargs -r tail -n 2
```

每条成功数据通常包含：

```text
episode.hdf5          正式训练数据
stage1_trace.hdf5     Stage 1 诊断轨迹
stage1_summary.json   接触和恢复摘要
metadata.json         正式 episode 摘要
```

`episode.hdf5` 中的状态、动作和图像为 30 Hz，原始及重力补偿六维力为
500 Hz。CCTV/HUD 仅用于操作员观察，不写入训练图像。

### 9. 正式采集 100 条碰撞数据

```bash
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 100 \
  --max-attempts 500 \
  --scenario collision \
  --reject-action quarantine
```

碰撞覆盖空间由 4 个半径和 24 个角度组成，共 96 格。为了让调度器在
同一轮运行中依次前进，应使用上面的一个命令连续采集100条，不要拆成
100次单条命令。第97至100条会进入下一轮覆盖。

如果进程中断，当前覆盖游标不会跨进程自动持久化。重新启动前应根据
最后一条数据的 episode metadata 手动调整配置中的：

```yaml
error_coverage_start_cycle
error_coverage_start_index
in_hole_disturbance_start_cycle
in_hole_disturbance_start_index
```

### 10. 正式采集 100 条纯净数据

碰撞批次结束后执行：

```bash
python -m vptele.main_scripted \
  --config vptele/config/config_arm_right_moving_hole.yaml \
  --target-episodes 100 \
  --max-attempts 500 \
  --scenario clean \
  --reject-action quarantine
```

两种场景使用相同的数据格式、重力补偿、两阶段回放、成功判定和质量
门。clean数据的 episode metadata 中
`scripted_collection_scenario=clean`，collision数据则为 `collision`。

### 11. 磁盘空间、拒绝数据和进度

当前单条数据约为 250--350 MB。200条数据预计需要约 50--70 GB，考虑
拒绝数据和安全余量，正式采集前建议至少准备100 GB可用空间：

```bash
df -h .
du -sh data/moving_hole_fixed_peg 2>/dev/null || true
```

自动质量门拒绝的数据默认移动到正式数据目录的相邻隔离目录，不会混入
训练数据。默认隔离目录为：

```text
data/moving_hole_fixed_peg_rejected/
```

不要在尚未验证质量门时使用 `--reject-action delete`。

每次尝试和最终批次状态都会写入：

```text
data/moving_hole_fixed_peg/batch_manifests/*.jsonl
```

如果连续10条数据被拒绝，批次会提前停止，应先检查错误，不要盲目继续。

### 12. 采集结束后备份数据

`data/` 被 `.gitignore` 排除，执行 `git push` 不会上传任何 HDF5 数据。
使用 `rsync` 将整个目录复制到存储服务器或原电脑：

```bash
rsync -avh --info=progress2 \
  data/moving_hole_fixed_peg/ \
  用户名@目标电脑IP:/目标路径/moving_hole_fixed_peg/

rsync -avh --info=progress2 \
  data/moving_hole_fixed_peg_rejected/ \
  用户名@目标电脑IP:/目标路径/moving_hole_fixed_peg_rejected/
```

如果隔离目录尚不存在，第二条命令可以跳过。确认正式episode、
`batch_manifests`和隔离数据均已备份后，再清理新电脑上的原始数据。
