import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self, name):
        super().__init__(name)
        
        self.pub = self.create_publisher(String, '/demo/read', 10)
        self.time = self.create_timer(1, self.tick)
        self.count = 0
        
    def tick(self):
        msg = String()
        msg.data = f'hello {self.count}'
        self.pub.publish(msg)
        self.count += 1
        
def main():
    rclpy.init()
    node = Talker('talker')
    try:
        rclpy.spin(node)  # 阻塞在这里, 反复执行定时器回调
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
        
        