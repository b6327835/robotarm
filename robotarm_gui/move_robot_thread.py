from PyQt5.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import cartesian as panda # type: ignore
from threading import Thread
import time

class MoveRobotThread(QThread):
    movement_status = pyqtSignal(str)  # Signal to update UI with movement status

    def __init__(self, x, y, z, mode, parent=None, dest_x=0.0, dest_y=0.0):
        super().__init__(parent)
        # Assign positions directly without validation
        self.x = round(x, 3)
        self.y = round(y, 3)
        self.z = round(z, 3)
        self.dest_x = round(dest_x, 3)
        self.dest_y = round(dest_y, 3)
        self.mode = mode

    def _execute_movement(self, moveit2, positions, description="Moving", max_attempts=10):
        """Execute movement with error handling, configurable retries."""
        for attempt in range(max_attempts):
            try:
                self.movement_status.emit(f"{description}: {positions} (Attempt {attempt + 1})")
                moveit2.move_to_configuration(positions)
                success = moveit2.wait_until_executed()
                if not success:
                    self.movement_status.emit(f"Movement failed on attempt {attempt + 1}")
                    time.sleep(0.1)  # Wait before retrying
                    continue
                return True
            except Exception as e:
                self.movement_status.emit(f"Error during movement on attempt {attempt + 1}: {str(e)}")
                time.sleep(1)  # Wait before retrying
        self.movement_status.emit(f"Failed to execute movement after {max_attempts} attempts.")
        return False

    def run(self):
        try:
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

            # Configure movement parameters
            moveit2.max_velocity = 0.15
            moveit2.max_acceleration = 0.15
            moveit2.planning_time = 2.0
            moveit2.num_planning_attempts = 3

            # Spin the node and allow for execution of the motion
            executor = rclpy.executors.MultiThreadedExecutor(2)
            executor.add_node(node)
            executor_thread = Thread(target=executor.spin, daemon=True, args=())
            executor_thread.start()
            node.create_rate(1.0).sleep()
            
            def move():
                return self._execute_movement(moveit2, [self.x, self.y, 0.0], "Moving to position")

            def pnp():
                pnp_x = self.x
                pnp_y = self.y
                # Approach position from above
                if not self._execute_movement(moveit2, [pnp_x, pnp_y, 0.0], "Approaching position"):
                    return
                time.sleep(1)
                # Pick
                if not self._execute_movement(moveit2, [pnp_x, pnp_y, 0.112], "Picking"):
                    return
                time.sleep(1)
                # Lift
                if not self._execute_movement(moveit2, [pnp_x, pnp_y, 0.0], "Lifting"):
                    return
                time.sleep(1)
                # Move to placement position
                if not self._execute_movement(moveit2, [self.dest_x, self.dest_y, 0.0], "Moving to placement"):
                    return
                time.sleep(1)
                # Place
                if not self._execute_movement(moveit2, [self.dest_x, self.dest_y, 0.112], "Placing object"):
                    return
                time.sleep(1)
                # Lift after placing
                if not self._execute_movement(moveit2, [self.dest_x, self.dest_y, 0.0], "Lifting after placing"):
                    return
                time.sleep(1)
                # Return to home position
                if not self._execute_movement(moveit2, [0.0, 0.0, 0.0], "Returning home"):
                    return
                print("Completed pnp movement.")

            def pick():
                if not self._execute_movement(moveit2, [self.x, self.y, 0.0], "Approaching pick position"):
                    return
                time.sleep(1)
                self._execute_movement(moveit2, [self.x, self.y, 0.0], "Picking")

            def home():
                self._execute_movement(moveit2, [0.0, 0.0, 0.0], "Returning home")

            def auto_pnp():
                # Use self.x and self.y for the object's position
                # Use self.x and self.y for the object's position
                pick_x = self.x
                pick_y = self.y
                # Pick
                if not self._execute_movement(moveit2, [self.x, self.y, 0.0], "Approaching object"):
                    return
                time.sleep(1)
                if not self._execute_movement(moveit2, [self.x, self.y, 0.112], "Picking object"):
                    return
                time.sleep(1)
                # Lift
                if not self._execute_movement(moveit2, [self.x, self.y, 0.0], "Lifting object"):
                    return
                time.sleep(1)
                # Move to placement position
                if not self._execute_movement(moveit2, [self.dest_x, self.dest_y, 0.0], "Moving to placement"):
                    return
                time.sleep(1)
                if not self._execute_movement(moveit2, [self.dest_x, self.dest_y, 0.112], "Placing object"):
                    return
                time.sleep(1)
                # Lift after placing
                if not self._execute_movement(moveit2, [self.dest_x, self.dest_y, 0.0], "Lifting after placing"):
                    return
                time.sleep(1)
                # Return to home position
                if not self._execute_movement(moveit2, [0.0, 0.0, 0.0], "Returning home"):
                    return
                print("Completed auto_pnp movement.")

            # Execute requested movement mode
            movement_functions = {
                "move": move,
                "pnp": pnp,
                "pick": pick,
                "home": home,
                "auto_pnp": auto_pnp
            }
            
            print(f"Starting movement in mode: {self.mode}")
            if self.mode in movement_functions:
                movement_functions[self.mode]()
            else:
                self.movement_status.emit(f"Invalid movement mode: {self.mode}")
            print("Movement function completed.")
        except Exception as e:
            self.movement_status.emit(f"Error: {str(e)}")
            print(f"Exception in MoveRobotThread: {e}")
        finally:
            rclpy.shutdown()
            if 'executor_thread' in locals():
                executor_thread.join()
            print("MoveRobotThread completed.")

