import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from arduinobot_msgs.action import ArmTask


class ArmTaskClient(Node):

    def __init__(self):
        super().__init__('arm_task_client')
        self._action_client = ActionClient(self, ArmTask, '/arm_task_server')

    def send_goal(self, x, y, z):
        goal_msg = ArmTask.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Success={result.success}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')


class ArmControlGUI(QWidget):

    def __init__(self):
        super().__init__()

        # Initialize the ROS2 client
        rclpy.init(args=None)
        self.arm_task_client = ArmTaskClient()

        # Set up the GUI layout
        self.init_ui()

    def init_ui(self):
        # Create the layout
        layout = QVBoxLayout()

        # Create input boxes for X, Y, Z
        self.x_input = QLineEdit(self)
        self.x_input.setPlaceholderText("Enter X value")

        self.y_input = QLineEdit(self)
        self.y_input.setPlaceholderText("Enter Y value")

        self.z_input = QLineEdit(self)
        self.z_input.setPlaceholderText("Enter Z value")

        # Create labels
        self.result_label = QLabel(self)

        # Create the "Send Goal" button
        send_button = QPushButton('Send Goal', self)
        send_button.clicked.connect(self.send_goal)

        # Add widgets to the layout
        layout.addWidget(QLabel("X:"))
        layout.addWidget(self.x_input)
        layout.addWidget(QLabel("Y:"))
        layout.addWidget(self.y_input)
        layout.addWidget(QLabel("Z:"))
        layout.addWidget(self.z_input)
        layout.addWidget(send_button)
        layout.addWidget(self.result_label)

        # Set the layout
        self.setLayout(layout)

        # Set the window title
        self.setWindowTitle('Arm Control')

        # Set the window size
        self.setGeometry(300, 300, 300, 200)

    def send_goal(self):
        # Get the X, Y, Z values from the input boxes
        try:
            x_value = float(self.x_input.text())
            y_value = float(self.y_input.text())
            z_value = float(self.z_input.text())
        except ValueError:
            self.result_label.setText("Invalid input! Please enter numeric values.")
            return

        # Call the action client to send the goal
        self.arm_task_client.send_goal(x_value, y_value, z_value)
        self.result_label.setText(f"Goal sent: X={x_value}, Y={y_value}, Z={z_value}")


def main():
    # Create a Qt application
    app = QApplication(sys.argv)

    # Create the GUI
    gui = ArmControlGUI()
    gui.show()

    # Run the Qt application
    sys.exit(app.exec_())

    # Shut down ROS2 after the Qt application exits
    rclpy.shutdown()


if __name__ == '__main__':
    main()
