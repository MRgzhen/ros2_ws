# mycobot_ros2 #
![OS](https：//img.shields.io/ubunt/)

mycobot 280 机械臂 ROS 2 仿真学习项目，按阶段递进：

| 阶段 | 命令 | 内容 |
| --- | --- | --- |
| 一 | `robotg` | 仅启动 Gazebo 仿真 |
| 二 | `robotm` | 一键启动 Gazebo + MoveIt + RViz |
| 三 | `robotm` + `hello_moveit` | 第一个 MoveIt 程序，自动运动到目标位姿 |
| 四 | `robotm` + `plan_around_objects` | 避障规划 Demo |
| 五 | `robotm` + `attach_object_demo` | 附着物体搬运 Demo |
| 六 | 见下文 | 手动抓取测试、视觉引导抓取 |

> `robotg`、`robotm` 为 `~/.bashrc` 中定义的别名。

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
