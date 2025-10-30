#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info('Keyboard input node has started. Type any letter and press Enter.')

        # 創建一個定時器來不斷檢查輸入
        self.timer_ = self.create_timer(0.1, self.check_input)

    def check_input(self):
        user_input = input("Enter a letter: ")  # 從標準輸入捕捉鍵盤輸入
        if user_input.isalpha():  # 檢查是否為字母
            msg = String()
            msg.data = user_input
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()

    try:
        rclpy.spin(keyboard_publisher)
    except KeyboardInterrupt:
        keyboard_publisher.get_logger().info('Node stopped manually by user.')
    finally:
        keyboard_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

