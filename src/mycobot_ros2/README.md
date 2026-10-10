# mycobot_ros2

![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04-E95420?logo=ubuntu)

mycobot 280 机械臂 ROS 2 仿真学习项目，按阶段递进：

| 阶段 | 命令 | 内容 |
| --- | --- | --- |
| 一 | `robotg` | 仅启动 Gazebo 仿真 |
| 二 | `robotm` | 一键启动 Gazebo + MoveIt + RViz |
| 三 | `robotg` + `image_sub` | 视觉入门：订阅相机图像、实时显示与按键存图 |
| 四 | `robotm` + `image_sub3` + `arm_move_to_object` | 视觉引导抓取：检测→base_link 坐标→侧抓→夹取 |

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

视觉链路（HSV 检测 → solvePnP → base_link 换算，见 `image_sub1` → `image_sub3`）打通后，
分三层把"检测 → 运动 → 夹取"串起来：

```
[image_sub3 视觉节点]──/vision/detected_point──►[arm_move_to_object 执行节点]
  HSV 分割→solvePnP→换算 base_link                等检测点→建场景→张爪→
  + calib_x/calib_y 校准                          pre-grasp→直线接近→闭爪→退回
  （换 YOLO/SAM：同名同格式                         │ 目标位姿一行调库构造
   发这个话题即可对接）                             ▼
                                        [grasp_geometry 纯函数库：侧抓/顶抓位姿]
                                               [arm_mover 库层封装：moveit_py
                                                规划执行/夹爪/附着]→Gazebo 控制器
```

**对接契约（换视觉实现只动视觉侧）**：往 `/vision/detected_point` 发
`PointStamped`（base_link 系，坐标换算与校准在视觉侧完成）。TF 广播的
`object_frame` 仅作 RViz / tf2_echo 调试用，运动侧不读它。

文件（`mycobot_learn/mycobot_learn/`）：

- `arm_mover.py`：公共封装库——规划执行（`move_to_pose` / `move_to_named` /
  `move_cartesian` / `plan_first_feasible`）、夹爪开合、规划场景/附着搬运，
  任何脚本 import 即用
- `grasp_geometry.py`：抓取几何纯函数库——侧抓/顶抓法兰目标位姿构造
  （`side_grasp_pair` / `top_down_pair`），含夹爪安装旋转修正（URDF rpy 1.579）
- `arm_hello_moveit.py`：演示①，第一个 MoveIt 程序（ready → 固定点位 → home）
- `arm_move_to_object.py`：演示②，纯执行节点——等检测点→调库建目标→护栏→
  pre-grasp→直线接近→闭爪→退回（不做坐标/几何计算）

运行（三个终端）：

```bash
robotm                    # 终端1：Gazebo + MoveIt + RViz
# 终端2：视觉节点（检测 + 换算 base_link + 校准，发 /vision/detected_point）
ros2 run mycobot_learn image_sub3 --ros-args -p use_sim_time:=true -p calib_y:=-0.02
# 校准思路：tf2_echo base_link object_frame 读原始检测值，与 detected_point
# 对比差多少补多少 calib_x/calib_y（solvePnP 的 y 实测偏高 ~2cm）
ros2 topic echo /vision/detected_point

# 终端3：按序验证
ros2 run mycobot_learn arm_hello_moveit --ros-args -p use_sim_time:=true
ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true -p dry_run:=true
ros2 run mycobot_learn arm_move_to_object --ros-args -p use_sim_time:=true
```

先跑 `arm_hello_moveit` 验证 moveit_py 链路，再 `dry_run` 在 RViz 检查轨迹终点：
side 模式手指应水平指向木棍、指尖贴抓取点。常用调整：`approach_yaw_deg` 换接近
方向，`grasp_z_offset` 调抓取高度。

常用参数（完整见各文件头部注释；`calib_x`/`calib_y` 传给 image_sub3，其余传给 arm_move_to_object）：

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `calib_x` / `calib_y` | 0 | 视觉偏差校准（image_sub3；y 实测偏高 ~2cm） |
| `grasp_mode` | side | side 水平侧抓（默认）/ top_down 竖直顶抓（矮物体备选） |
| `approach_yaw_deg` | 135.0 | 接近方向方位角；不可达时 `yaw_offsets_deg` 候选依次尝试 |
| `grasp_z_offset` | 0.0 | 抓取点相对物体中心的 z 偏移 |
| `tcp_offset` | 0.10 | 法兰原点到指尖距离 |
| `hover_height` | 0.06 | 悬停高度（仅 top_down） |
| `dry_run` | false | 只规划不执行，RViz 查轨迹 |
| `do_grasp` | true | 是否闭合夹爪 |
| `lift_after_grasp` | false | 抓后附着物体并退回提起（MoveIt 层搬运） |

抓取策略（side，默认）：0.35m 木棍竖放，顶抓时棍顶必穿手腕（物理无解），
唯一可行是水平侧抓——夹爪横着接近、手指夹棍侧壁。几何修正（夹爪相对法兰
转 1.579 rad 的安装旋转、法兰→指尖偏移、高度补偿）统一在 `grasp_geometry.py`。
top_down 保留作矮物体备选；其严格竖直姿态可能触发 KDL
"no valid states for goal tree"，从 ready 出发规避。

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
