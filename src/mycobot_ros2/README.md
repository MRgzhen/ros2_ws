# mycobot_ros2

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu)

mycobot 280 机械臂 ROS 2 仿真学习项目，按阶段递进：

| 阶段 | 命令 | 内容 |
| --- | --- | --- |
| 一 | `robotg` | 仅启动 Gazebo 仿真 |
| 二 | `robotm` | 一键启动 Gazebo + MoveIt + RViz |
| 三 | `robotg` + `image_sub` | 视觉入门：订阅相机图像、实时显示与按键存图 |
| 四 | `robotm` + `image_sub3` + `arm_move_to_object` | 视觉引导抓取：检测→TF→MoveIt 规划执行→夹取 |

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

## 阶段四：视觉引导抓取（ArmMover 公共封装）

视觉链路（HSV 检测 → solvePnP → TF 广播，见 `image_sub1` → `image_sub3`）打通后，
用公共封装 `ArmMover` 把 MoveIt（moveit_py）+ TF + 夹爪串成"检测 → 运动 → 夹取"：

```
[任意视觉节点]                 [库层 arm_mover.py]                [Gazebo]
 image_sub3 / 未来的     ──►   ArmMover                          arm_controller
 YOLO、SAM ...                 ├─ TF 查询 object_frame→base_link  gripper_action_controller
   唯一契约：广播              ├─ moveit_py 规划+执行
   object_frame TF             └─ 夹爪/附着搬运封装
                                       ▲ import
                               [节点层 arm_move_to_object.py]
```

**对接契约（换视觉实现只动视觉侧）**：往 TF 广播 `object_frame`（范本
`image_sub3.publish_tf`），或发 `PointStamped` 到 `/vision/detected_point`
（`target_source:=topic`）。

文件（`mycobot_learn/mycobot_learn/`）：

- `arm_mover.py`：公共封装库——TF 查询、规划执行（`move_to_pose` / `move_to_named` /
  `move_to_frame` / `hover_and_descend`）、夹爪开合、附着搬运，任何脚本 import 即用
- `arm_hello_moveit.py`：演示①，第一个 MoveIt 程序（ready → 固定点位 → home）
- `arm_move_to_object.py`：演示②，完整抓取流程（工作空间护栏、悬停、竖直下降、夹取、
  可选附着提起）

运行（三个终端）：

```bash
robotm                                              # 终端1：Gazebo + MoveIt + RViz
ros2 run mycobot_learn image_sub3                   # 终端2：视觉检测并广播 object_frame
ros2 run tf2_ros tf2_echo base_link object_frame    # 读物体坐标，标定 grasp_z_offset

# 终端3：按序验证
ros2 run mycobot_learn arm_hello_moveit --ros-args -p use_sim_time:=true
ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true -p dry_run:=true
ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true -p grasp_z_offset:=0.10
```

先跑 `arm_hello_moveit` 验证 moveit_py 链路，再 `dry_run` 在 RViz 检查轨迹终点
是否在木棒正上方、夹爪是否竖直朝下，最后实跑。

常用参数（完整见 `arm_move_to_object.py` 头部注释）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `grasp_z_offset` | 0.0 | 抓取点相对物体中心的 z 偏移（tf2_echo 实测标定） |
| `hover_height` | 0.06 | 悬停点高于抓取点的高度 |
| `tcp_offset` | 0.10 | flange 原点到指尖距离 |
| `dry_run` | false | 只规划不执行，RViz 查轨迹 |
| `do_grasp` | true | 是否闭合夹爪 |
| `lift_after_grasp` | false | 抓后附着物体并提起（MoveIt 层搬运） |
| `target_source` | tf | tf / topic（后者订阅 `/vision/detected_point`） |

抓取策略：0.03m 直径圆柱顶面太细，夹爪对准轴心竖直下降，手指跨两侧**夹上部侧壁**。
已知坑：严格竖直姿态可能触发 KDL "no valid states for goal tree"——本流程从 ready
（已竖直朝下）出发规避；仍失败时调 `grasp_yaw` 或加大 `planning_time`。

## 仿真相机（D435）

- 分辨率 424×240，5Hz，深度上限 1.5m
- 安装高 0.50m，**俯角 45°**（`mycobot_description/urdf/sensors/intel_rgbd_cam_d435.urdf.xacro` 中 `camera_tilt_angle_deg`），画面下边缘覆盖机械臂基座附近地面

## 包结构

```
mycobot_ros2/
├── mycobot_bringup/        # 一键启动脚本（Gazebo / Gazebo+MoveIt）
├── mycobot_description/    # URDF/Xacro、mesh、RViz 配置（含 D435 相机模型）
├── mycobot_gazebo/         # 仿真 launch、世界文件、桥接配置
├── mycobot_learn/          # 学习节点：listener/talk、image_sub 系列（视觉）、arm_mover/arm_move_to_object（视觉引导抓取）
├── mycobot_moveit_config/  # MoveIt2 配置与 launch
└── docs/                   # 视觉练习步骤等文档
```
