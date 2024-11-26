
from PyQt5.QtCore import QThread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import cartesian as panda
from threading import Thread
import time

class MoveRobotThread(QThread):
    def __init__(self, x, y, z, mode, parent=None):
        super().__init__(parent)
        self.x = round(x, 2)
        self.y = round(y, 2)
        self.z = round(z, 2)
        self.mode = mode

    def run(self):
        rclpy.init()
        node = Node("python_move_goal")

        # Create callback group for MoveIt2
        callback_group = ReentrantCallbackGroup()
        moveit2 = MoveIt2(
            node=node,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        # Spin the node and allow for execution of the motion
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        node.create_rate(1.0).sleep()
        moveit2.max_velocity = 0.1
        moveit2.max_acceleration = 0.1
        
        def move():
            joint_positions = [self.x, self.y, 0.0]
            # node.get_logger().info(f"Moving to joint positions: {list(positions)}")
            moveit2.move_to_configuration(joint_positions)
            moveit2.wait_until_executed()
        def pnp():
            # 1. Move to x, y to pos
            moveit2.move_to_configuration([self.x, self.y, 0.0])
            moveit2.wait_until_executed()
            time.sleep(3)

            # 2. Move z (pick)
            moveit2.move_to_configuration([self.x, self.y, 0.120])
            moveit2.wait_until_executed()
            time.sleep(3)

            node.get_logger().info("Waiting for 2 seconds...")
            # time.sleep(2)

            # 4. Move z back to 0 (place)
            moveit2.move_to_configuration([self.x, self.y, 0.0])
            moveit2.wait_until_executed()
            time.sleep(3)

            # 5. Move xy back to 0 
            moveit2.move_to_configuration([0.0, 0.0, 0.0])
            moveit2.wait_until_executed()
            time.sleep(3)

            moveit2.move_to_configuration([0.0, 0.0, 0.120])
            moveit2.wait_until_executed()
            time.sleep(4)
            
            moveit2.move_to_configuration([0.0, 0.0, 0.0])
            moveit2.wait_until_executed()
            
        def pick():
            node.get_logger().info(f"picking at joint positions: {[self.x, self.y, 0.120]}")
            moveit2.move_to_configuration([self.x, self.y, 0.120])
            moveit2.wait_until_executed()
            moveit2.move_to_configuration([self.x, self.y, 0.0])
            moveit2.wait_until_executed()
            
        def home():
            node.get_logger().info(f"Moving to joint positions: home")
            moveit2.move_to_configuration([0.0,0.0,0.0])
            moveit2.wait_until_executed() 
        print(f"mode: {self.mode}")
        if self.mode == "move":
            move()
        if self.mode == "pnp":
            pnp()
        if self.mode == "pick":
            pick()
        if self.mode == "home":
            home()

        rclpy.shutdown()
        executor_thread.join()