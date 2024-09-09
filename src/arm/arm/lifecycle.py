import rclpy
import rclpy.executors
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from std_msgs.msg import String
import time
class SimpleLifecycleNode(Node):
    def __init__(self, node_name, **kwargs):
            super().__init__(node_name, **kwargs)
    
    def on_configure(self,state: State) -> TransitionCallbackReturn:
          self.sub_ = self.create_subscription(String, "Chatter", self.msgCallback, 10)
          self.get_logger().info("Lifecycle Node on_onfig called.")
          return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self,state: State) -> TransitionCallbackReturn:
          self.sub_ = self.destroy_subscription(self.sub_)
          self.get_logger().info("Lifecycle Node shutdown() called.")
          return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self,state: State) -> TransitionCallbackReturn:
          self.sub_ = self.destroy_subscription(self.sub_)
          self.get_logger().info("Lifecycle Node cleanup() called.")
          return TransitionCallbackReturn.SUCCESS
    
    def on_activate(self,state: State) -> TransitionCallbackReturn:
          self.get_logger().info("Lifecycle Node activate() called.")
          time.sleep(2)
          return super().on_activate(state)
    
    def on_deactivate(self,state: State) -> TransitionCallbackReturn:
          self.get_logger().info("Lifecycle Node deactivate() called.")
          return super().on_deactivate(state)
    
    def msgCallback(self, msg):
          cstate = self._state_machine.current_state
          if cstate[1] == "active":
                self.get_logger().info(f"lifecycle callback got {msg.data}")

def main():
    rclpy.init()
    executor = rclpy.executors.SingleThreadedExecutor()
    lifecycle_node = SimpleLifecycleNode("simple_lifecycle_node")
    executor.add_node(lifecycle_node)
    try:
          executor.spin()
    except(KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
          lifecycle_node.destroy_node()


if __name__ == "__main__":
    main()