# mycobot_ros2

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu)

mycobot 280 机械臂 ROS 2 仿真学习项目，按阶段递进：

| 阶段 | 命令 | 内容 |
| --- | --- | --- |
| 一 | `robotg` | 仅启动 Gazebo 仿真 |
| 二 | `robotm` | 一键启动 Gazebo + MoveIt + RViz |
| 三 | `robotg` + `image_sub` | 视觉入门：订阅相机图像、实时显示与按键存图 |

> `build`、`robotg`、`robotm` 为 `~/.bashrc` 中定义的别名，配置见下文环境准备。

## 环境准备（前提条件）

以下步骤是所有阶段的执行前提，首次使用时按序完成一次；每个新终端只需重新执行第 2 步的 `source /opt/ros/jazzy/setup.bash` 和 `source ~/dev/ros2_ws/install/setup.bash`。

**配置 `~/.bashrc` 别名（一次性，后续步骤及各阶段都会用到）：**

```bash
cat >> ~/.bashrc << 'EOF'
alias build='cd ~/dev/ros2_ws && colcon build && source install/setup.bash'
alias robotg='ros2 launch mycobot_gazebo mycobot.gazebo.launch.py'
alias robotm='bash ~/dev/ros2_ws/src/mycobot_ros2/mycobot_bringup/scripts/mycobot_280_gazebo_and_moveit.sh'
EOF
source ~/.bashrc
```

1. 创建并切换到 conda 环境（Python 3.12 与 Jazzy 匹配，提供 OpenCV 等视觉依赖）：

```bash
conda create -n mycobot_ros2 python=3.12
conda activate mycobot_ros2
pip install -r ~/dev/ros2_ws/src/mycobot_ros2/mycobot_learn/requirements.txt
```

2. 加载 ROS 2 Jazzy，用 rosdep 安装依赖：

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init                                # 已初始化过会报错，跳过即可
rosdep update
rosdep install --from-paths src --ignore-src -y # 在 ~/dev/ros2_ws 下执行
```

3. 构建并加载工作区：

```bash
build   # 即 cd ~/dev/ros2_ws && colcon build && source install/setup.bash
```

## 阶段一：仅启动 Gazebo

新终端执行：

```bash
robotg
```

即 `ros2 launch mycobot_gazebo mycobot.gazebo.launch.py`，只加载 Gazebo 仿真环境和机械臂模型，用于确认仿真正常、查看关节状态与话题。

## 阶段二：一键启动 Gazebo + MoveIt

新终端执行：

```bash
robotm
```

即一键脚本 `mycobot_280_gazebo_and_moveit.sh`（Gazebo + MoveIt + RViz，自动等待 15 秒后启动 MoveIt 并调整视角），Ctrl+C 一次性退出所有进程。

<details>
<summary>手动分步启动（可选）</summary>

终端 1：

```bash
ros2 launch mycobot_gazebo mycobot.gazebo.launch.py
```

终端 2：

```bash
ros2 launch mycobot_moveit_config move_group.launch.py
```

</details>

## 阶段三：视觉入门（image_sub）

先起仿真（终端 1），再运行图像订阅节点（终端 2）：

```bash
robotg
ros2 run mycobot_learn image_sub
```

功能（`mycobot_learn/mycobot_learn/image_sub.py`）：

- 订阅 `/camera_head/color/image_raw`（话题可用参数 `image_topic` 覆盖），BEST_EFFORT QoS
- `cv_bridge` 转 OpenCV 后 `imshow` 实时显示（仿真相机 5Hz）
- 按 `s` 保存当前帧到 `~/dev/ros2_ws/img/`（文件名带序号+时间戳）
- 按 `q` 关闭窗口并退出节点

指定其他话题：

```bash
ros2 run mycobot_learn image_sub --ros-args -p image_topic:=/camera_head/depth/image_rect_raw
```

后续视觉练习路线（HSV 分割 → solvePnP → 点云/ICP → 联动 MoveIt）见 [docs/视觉练习步骤.md](docs/视觉练习步骤.md)。

## 仿真相机（D435）

- 分辨率 424×240，5Hz，深度上限 1.5m
- 安装高 0.50m，**俯角 45°**（`mycobot_description/urdf/sensors/intel_rgbd_cam_d435.urdf.xacro` 中 `camera_tilt_angle_deg`），画面下边缘覆盖机械臂基座附近地面

## 包结构

```
mycobot_ros2/
├── mycobot_bringup/        # 一键启动脚本（Gazebo / Gazebo+MoveIt）
├── mycobot_description/    # URDF/Xacro、mesh、RViz 配置（含 D435 相机模型）
├── mycobot_gazebo/         # 仿真 launch、世界文件、桥接配置
├── mycobot_learn/          # 学习节点：listener/talk（话题入门）、image_sub（视觉入门）
├── mycobot_moveit_config/  # MoveIt2 配置与 launch
└── docs/                   # 视觉练习步骤等文档
```
