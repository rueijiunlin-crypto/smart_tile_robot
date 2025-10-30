#!/usr/bin/env python3

import sys
import os
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from top_base_notify.msg import Move  # Replace with the actual path to your custom Move message
import time

class TileCenterPoseSubscriber(Node):

    def __init__(self):
        super().__init__('tile_center_pose_subscriber')

        # Subscriptions
        self.tile_center_subscriber = self.create_subscription(
            String,
            '/World_Coordinates',
            self.tile_center_callback,
            10
        )
        self.camera_pose_subscriber = self.create_subscription(
            PoseStamped,
            '/camera/pose',
            self.camera_pose_callback,
            10
        )
        self.hit_initialized_subscriber = self.create_subscription(
            String,
            '/move_hit_initialized',
            self.hit_initialized_callback,
            10
        )

        # Publishers
        self.move_publisher = self.create_publisher(Move, '/move', 10)
        self.move2_publisher = self.create_publisher(String, '/move2', 10)

        # State variables
        self.camera_pose_x = 0
        self.command_sequence = []
        self.flow_state = "continuous_d"
        self.last_world_coordinates_time = None

        # Batch command state
        self.batch_active = False
        self.batch_command = None
        self.batch_count = 0

        # Timer to check the command sequence every 5 seconds
        self.timer = self.create_timer(5.0, self.timer_callback)

    def camera_pose_callback(self, msg):
        self.camera_pose_x = round(msg.pose.position.x, 3)
        self.get_logger().info(f'Received Camera Pose - X: {self.camera_pose_x}')

        if self.camera_pose_x > 0.74 and self.flow_state not in ["switch_to_a", "continuous_a"]:
            self.command_sequence = ['s'] * 12
            self.flow_state = "switch_to_a"
            self.get_logger().info('Switching to `s` followed by `a`')
        elif self.camera_pose_x < -0.06 and self.flow_state not in ["switch_to_d", "continuous_d"]:
            self.command_sequence = ['s'] * 12
            self.flow_state = "switch_to_d"
            self.get_logger().info('Switching to `s` followed by `d`')

    def tile_center_callback(self, msg: String):
        self.last_world_coordinates_time = time.time()
        self.get_logger().info('Received World Coordinates - Pausing Command Sequence')

    def hit_initialized_callback(self, msg: String):
        current_time = time.time()
        if self.last_world_coordinates_time and current_time - self.last_world_coordinates_time <= 1:
            self.batch_command = 'a' if self.flow_state.startswith("continuous_a") else 'd'
            self.batch_active = True
            self.batch_count = 0
            self.get_logger().info(f'Batch process started with command `{self.batch_command}`')

    def timer_callback(self):
        if self.batch_active:
            if self.batch_count < 7:
                self.publish_command(self.batch_command)
                self.batch_count += 1
                self.get_logger().info(f'Batch command `{self.batch_command}` sent ({self.batch_count}/7)')
            else:
                self.batch_active = False
                self.batch_command = None
                self.get_logger().info('Batch process completed')
            return

        if self.last_world_coordinates_time and time.time() - self.last_world_coordinates_time <= 3:
            return

        if self.command_sequence:
            command = self.command_sequence.pop(0)
            self.publish_command(command)

        if not self.command_sequence:
            if self.flow_state == "switch_to_a":
                self.command_sequence = ['a'] * 3
                self.flow_state = "continuous_a"
                self.get_logger().info('Transitioning to continuous `a`')
            elif self.flow_state == "switch_to_d":
                self.command_sequence = ['d'] * 3
                self.flow_state = "continuous_d"
                self.get_logger().info('Transitioning to continuous `d`')
            elif self.flow_state == "continuous_a":
                self.command_sequence = ['a']
            elif self.flow_state == "continuous_d":
                self.command_sequence = ['d']

    def publish_command(self, command):
        move_msg = Move()
        move_msg.data = command
        move_msg.uid = str(uuid.uuid4())
        self.move_publisher.publish(move_msg)
        self.get_logger().info(f'Published to /move: data={move_msg.data}, uid={move_msg.uid}')

        move2_msg = String()
        move2_msg.data = command
        self.move2_publisher.publish(move2_msg)
        self.get_logger().info(f'Published to /move2: data={move2_msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TileCenterPoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
