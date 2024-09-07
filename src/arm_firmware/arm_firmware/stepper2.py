#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CartesianRobotController(Node):
    def __init__(self):
        super().__init__('cartesian_robot_controller')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        self.timer = self.create_timer(1.0, self.send_command)
        self.counter = 0

    def send_command(self):
        msg = String()
        # Command format: "X100 Y200 Z300"
        msg.data = f"X{self.counter * 10} Y{self.counter * 10} Z{self.counter * 10}"
        self.publisher.publish(msg)
        self.counter += 1

def main(args=None):
    rclpy.init(args=args)
    node = CartesianRobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
