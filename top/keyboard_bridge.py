#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardBridge(Node):
    """橋接 /keyboard_control -> /keyboard_input，讓上機構接受 vision 發出的指令。"""

    def __init__(self):
        super().__init__('keyboard_bridge')
        self.sub = self.create_subscription(
            String, '/keyboard_control', self.callback, 10)
        self.pub = self.create_publisher(String, '/keyboard_input', 10)
        self.get_logger().info('Keyboard bridge started: /keyboard_control -> /keyboard_input')

    def callback(self, msg: String):
        # 直接轉發字串
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

