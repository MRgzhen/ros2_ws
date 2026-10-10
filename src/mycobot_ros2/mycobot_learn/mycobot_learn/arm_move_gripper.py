import arm_move as amove
import rclpy
from rclpy.node import Node


class ArmMoveGripper(Node):
    def __init__(self):
        super().__init__("arm_move_gripper")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
