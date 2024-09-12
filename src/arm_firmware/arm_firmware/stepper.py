#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class CartesianRobotController(Node):
    def __init__(self):
        super().__init__('cartesian_robot_controller')
        self.publisher = self.create_publisher(String, 'robot_commands', 10)
        
        # Try connecting to the first port, and if it fails, try another one
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200)
            self.get_logger().info('Connected to /dev/ttyACM0')
        except serial.SerialException:
            try:
                self.serial_port = serial.Serial('/dev/ttyUSB0', 115200)
                self.get_logger().info('Connected to /dev/ttyUSB0')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to connect to serial ports: {e}')
                self.serial_port = None

    def send_command(self, command: str):
        if self.serial_port:
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
            self.serial_port.write(command.encode() + b'\n')  # Send command to ESP32
        else:
            self.get_logger().error('No serial connection established')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianRobotController()

    # Example usage:
    while rclpy.ok():
        command = input("Enter command (e.g., 'X100 Y200 Z300'): ")
        if command.lower() == 'exit':
            break
        node.send_command(command)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class CartesianRobotController(Node):
#     def __init__(self):
#         super().__init__('cartesian_robot_controller')
#         self.publisher = self.create_publisher(String, 'robot_commands', 10)

#     def send_command(self, command: str):
#         msg = String()
#         msg.data = command
#         self.publisher.publish(msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = CartesianRobotController()

#     # Example usage:
#     # You can modify this part to send commands from another script or an interface
#     while rclpy.ok():
#         command = input("Enter command (e.g., 'X100 Y200 Z300'): ")
#         if command.lower() == 'exit':
#             break
#         node.send_command(command)

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()





# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import String

# class CartesianRobotController(Node):
#     def __init__(self):
#         super().__init__('cartesian_robot_controller')
#         self.publisher = self.create_publisher(String, 'robot_commands', 10)
#         self.timer = self.create_timer(1.0, self.send_command)
#         self.counter = 0

#     def send_command(self):
#         # Example command format: "X100 Y200 Z300"
#         msg = String()
#         msg.data = f"X{self.counter * 10} Y{self.counter * 10} Z{self.counter * 10}"
#         self.publisher.publish(msg)
#         self.counter += 1

# def main(args=None):
#     rclpy.init(args=args)
#     node = CartesianRobotController()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
