import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    
    
    def __init__(self, name):
        super().__init__(name)
        
        # 创建监听者
        self.create_subscription(String, '/demo/read', self.callback, 10)
        self.get_logger().info('已经开始')
        
    def callback(self, msg):
        print(msg)
        
def main():
    rclpy.init()
    node = Listener('listener')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
        
if __name__ == '__main__':
    main()

