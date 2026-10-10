"""抓取位姿几何库（纯函数，只依赖 math/scipy，不依赖 rclpy/moveit）。

为什么需要它：视觉给出的是"物体在 base_link 下的位置"（image_sub3 发
/vision/detected_point），而 MoveIt 要的是"link6_flange 的目标位姿"——
两者之间差着抓取策略（接近方向、悬停后退）和机器人自身几何（指尖到
法兰的距离、夹爪安装旋转）。这些量与视觉无关、只跟机械臂/夹爪有关，
统一收口在本库；arm_move_to_object 拿到检测点后一行调用即得目标位姿。

安装几何来源：mycobot_280.urdf.xacro 里 adaptive_gripper 固定关节
link6_flange → gripper_base origin xyz="0 0 0.034" rpy="1.579 0 0"：
手指沿 gripper_base z 轴伸出，而运动目标下给的是 link6_flange
（arm_mover.pose_link）。目标姿态里不扣掉这 ~90° 安装旋转，"手指水平
指向物体"会变成手指竖直朝下——实测指尖戳到检测点下方 ~10cm、把木棍
碰倒（2026-10 TF 时间线复盘定位）。修正姿态（腕下翻、法兰 z 朝下）下
gripper_base 挂在法兰 z 正向=世界下方 MOUNT_DZ 处、指尖平面与它同高，
所以法兰目标高度要加 MOUNT_DZ。均以 dry_run 在 RViz 实看为准。
"""

import math

from scipy.spatial.transform import Rotation

# 夹爪竖直朝下的法兰姿态四元数 xyzw（pose_link=link6_flange、表达在 base
# 系，由 ready 位姿 FK 实算，绕 x 轴 +90°）。注意与旧 C++ 的 (1,0,0,0)
# 不同：那边 pose_link 不是 link6_flange。
DOWNWARD_QUAT = (0.70711, 0.0, 0.0, 0.70711)

# link6_flange → gripper_base 固定安装（URDF adaptive_gripper origin）
MOUNT_RX = 1.579   # 安装旋转绕 x（rad）
MOUNT_DZ = 0.034   # gripper_base 沿法兰 z 的安装偏移（m）


def side_grasp_pair(ox, oy, grasp_z, yaw, tcp_offset, backoff):
    """水平侧抓：返回一对法兰目标 (pregrasp, grasp)。

    每个目标为 ((x, y, z), (qx, qy, qz, qw))，可 mover.goal_pose(*xyz, q) 直接用。
    ox/oy/grasp_z  物体抓取点（base_link 系，grasp_z 已含 grasp_z_offset）
    yaw            接近方向方位角（rad）：夹爪沿该方向水平伸向物体
    tcp_offset     法兰原点→指尖距离（沿接近方向）
    backoff        pre-grasp 相对抓取位的后退量（=直线接近段长度）

    姿态构造分三步（腕下翻 + 绕手指轴滚转）：
      ① Rz(yaw−90°)·DOWNWARD_QUAT·Rx(MOUNT_RX)：腕下翻法兰目标（法兰 z
        竖直朝下，DOWNWARD 同族）。腕上翻变体（Rx(−MOUNT_RX)，法兰 z 朝上）
        数学上同样"手指水平"，但 mycobot 上 IK 无解（2026-10 实测 "Unable
        to sample any valid states for goal tree"），故取下翻。
      ② 再叠安装旋转 Rx(MOUNT_RX) 得 gripper_base 姿态：手指（gripper z）
        水平沿接近方向 ✓，但闭合方向是 gripper y（URDF adaptive_gripper
        左右手指链沿 y=±对称布置），此时 ≈竖直——手指板上下开合、夹不住
        竖直木棍（2026-10 RViz 实测）。故绕手指轴（gripper z）右乘
        Rz(90°) 滚转：手指方向不变（Rz 不动 z 轴）、闭合面转到水平⊥接近
        方向，跨棍两侧。
      ③ 左乘/右乘 Rx(−MOUNT_RX) 反解回法兰目标姿态（MoveIt 目标是
        link6_flange）。
    滚转后法兰 z 变为水平，安装偏移 (0,0,MOUNT_DZ) 在世界系的投影也
    变为水平（z 分量≈0），故位置补偿用完整 3D 投影而非只补高度。
    """
    R_g = (Rotation.from_euler("z", yaw - math.pi / 2)
           * Rotation.from_quat(DOWNWARD_QUAT)
           * Rotation.from_euler("x", 2 * MOUNT_RX))   # ①+② 前半：法兰·安装
    R_g = R_g * Rotation.from_euler("z", math.pi / 2)   # ② 绕手指轴滚 90°
    R_f = R_g * Rotation.from_euler("x", -MOUNT_RX)     # ③ 反解法兰目标姿态
    q = tuple(R_f.as_quat())
    approach = (math.cos(yaw), math.sin(yaw))
    # TCP 对准物体中心：法兰 = 抓取点 - 安装偏移投影 - tcp_offset·approach；
    # 安装偏移沿法兰 z（滚转后水平），逐分量投影，不再只补 z 高度
    off = R_f.as_matrix() @ (0.0, 0.0, MOUNT_DZ)
    gx = ox - off[0] - tcp_offset * approach[0]
    gy = oy - off[1] - tcp_offset * approach[1]
    gz = grasp_z - off[2]
    grasp = ((gx, gy, gz), q)
    pregrasp = ((gx - backoff * approach[0], gy - backoff * approach[1], gz), q)
    return pregrasp, grasp


def top_down_pair(ox, oy, grasp_z, tcp_offset, hover_height, yaw=0.0):
    """竖直顶抓（V1，矮物体备选）：返回一对法兰目标 (hover, descend)。"""
    q = tuple((Rotation.from_euler("z", yaw)
               * Rotation.from_quat(DOWNWARD_QUAT)).as_quat())
    descend_z = grasp_z + tcp_offset  # 目标是"指尖到抓取点"，法兰再抬高 tcp_offset
    hover = ((ox, oy, descend_z + hover_height), q)
    descend = ((ox, oy, descend_z), q)
    return hover, descend
