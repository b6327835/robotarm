#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <memory>

void move_robot(const std::shared_ptr<rclcpp::Node> node, 
                double x, double y, double z, double jog_step)
{
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(node, "arm");

    // Create target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z;
    target_pose.orientation.w = 1.0; // Assuming no rotation

    arm_move_group.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
    bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(arm_plan_success)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Planner SUCCEEDED, moving the arm");
        arm_move_group.move();
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Planner failed!");
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 5) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Usage: simple_moveit_interface x y z jog_step");
      return 1;
  }

  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);
  double z = std::stod(argv[3]);
  double jog_step = std::stod(argv[4]);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("simple_moveit_interface");
  move_robot(node, x, y, z, jog_step);

  rclcpp::spin(node);
  rclcpp::shutdown();
}
