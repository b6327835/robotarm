import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
import actionlib
from pynput.keyboard import Key, Listener
import sys

class CartesianTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        # Parameters
        self.declare_parameter("xyz_max", 0.1)  # Max movement step in meters
        self.XYZ_MAX = self.get_parameter("xyz_max").value

        # Initialize MoveIt action client
        self.move_group_client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self.move_group_client.wait_for_server()

        # Set up key listener
        self.key_listener = Listener(on_press=self.update_position)
        self.key_listener.start()

        # Key bindings for Cartesian control
        self.keys_bindings = {
            "w": (self.XYZ_MAX, 0.0, 0.0),
            "s": (-self.XYZ_MAX, 0.0, 0.0),
            "a": (0.0, self.XYZ_MAX, 0.0),
            "d": (0.0, -self.XYZ_MAX, 0.0),
            "r": (0.0, 0.0, self.XYZ_MAX),
            "f": (0.0, 0.0, -self.XYZ_MAX),
        }

        self.get_logger().info("""
        Keyboard Teleop for Cartesian Control
        Controls:
        W: Move +X
        S: Move -X
        A: Move +Y
        D: Move -Y
        R: Move +Z
        F: Move -Z
        Any other key to stop
        CTRL-C to quit
        """)

    def send_cartesian_goal(self, x, y, z):
        goal = MoveGroupGoal()
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.w = 1.0
        
        goal.request.goal_constraints.append(pose_goal)
        self.move_group_client.send_goal(goal)
        self.move_group_client.wait_for_result()
        self.get_logger().info(f"Moved to position: x={x}, y={y}, z={z}")

    def update_position(self, key):
        if key.char in self.keys_bindings:
            x, y, z = self.keys_bindings[key.char]
            self.send_cartesian_goal(x, y, z)
        else:
            self.send_cartesian_goal(0.0, 0.0, 0.0)  # Stop
#ads
def main(args=None):
    rclpy.init(args=args)
    node = CartesianTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
