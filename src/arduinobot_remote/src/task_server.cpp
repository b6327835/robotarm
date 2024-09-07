#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "arduinobot_msgs/action/arm_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>

#include <memory>
#include <thread>

using namespace std::placeholders;

namespace arduinobot_remote
{
class TaskServer : public rclcpp::Node
{
public:
  explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("task_server", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArmTask>(
        this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
        std::bind(&TaskServer::cancelCallback, this, _1),
        std::bind(&TaskServer::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<arduinobot_msgs::action::ArmTask>::SharedPtr action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const arduinobot_msgs::action::ArmTask::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with coordinates: [%f, %f, %f], jog_step: %f",
                goal->x, goal->y, goal->z, goal->jog_step);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
    arm_move_group.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
  {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
  }

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    auto result = std::make_shared<arduinobot_msgs::action::ArmTask::Result>();

    // MoveIt 2 Interface
    auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");

    // Define the target pose (end-effector position)
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = goal_handle->get_goal()->x;
    target_pose.position.y = goal_handle->get_goal()->y;
    target_pose.position.z = goal_handle->get_goal()->z;

    // Set the target pose
    arm_move_group.setPoseTarget(target_pose);

    // Plan and execute the movement
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (arm_move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(get_logger(), "Planning successful, executing the movement");
      arm_move_group.move();
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Planning failed!");

      // Print the bounds of the planning group
      const moveit::core::JointModelGroup* joint_model_group = arm_move_group.getCurrentState()->getJointModelGroup("arm");

      // Iterate through joints and retrieve joint limits
      const std::vector<const moveit::core::JointModel*>& joint_models = joint_model_group->getActiveJointModels();
      for (const moveit::core::JointModel* joint_model : joint_models)
      {
        const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds(joint_model->getName());
        RCLCPP_INFO(get_logger(), "Joint %s: min position: %f, max position: %f",
                    joint_model->getName().c_str(), bounds.min_position_, bounds.max_position_);
      }

      result->success = false;
      goal_handle->abort(result);
      return;
    }

    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }
};

}  // namespace arduinobot_remote

RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)





// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include "arduinobot_msgs/action/arduinobot_task.hpp"
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <memory>


// using namespace std::placeholders;

// namespace arduinobot_remote
// {
// class TaskServer : public rclcpp::Node
// {
// public:
//   explicit TaskServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
//     : Node("task_server", options)
//   {
//     RCLCPP_INFO(get_logger(), "Starting the Server");
//     action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArduinobotTask>(
//         this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
//         std::bind(&TaskServer::cancelCallback, this, _1),
//         std::bind(&TaskServer::acceptedCallback, this, _1));
//   }

// private:
//   rclcpp_action::Server<arduinobot_msgs::action::ArduinobotTask>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse goalCallback(
//       const rclcpp_action::GoalUUID& uuid,
//       std::shared_ptr<const arduinobot_msgs::action::ArduinobotTask::Goal> goal)
//   {
//     RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
//     (void)uuid;
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse cancelCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
//   {
//     (void)goal_handle;
//     RCLCPP_INFO(get_logger(), "Received request to cancel goal");
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
//     //auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");
//     arm_move_group.stop();
//     //gripper_move_group.stop();
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void acceptedCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
//   {
//     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//     std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
//   }

//   void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArduinobotTask>> goal_handle)
//   {
//     RCLCPP_INFO(get_logger(), "Executing goal");
//     auto result = std::make_shared<arduinobot_msgs::action::ArduinobotTask::Result>();

//     // MoveIt 2 Interface
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
//     //auto gripper_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "gripper");

//     std::vector<double> arm_joint_goal;
//     //std::vector<double> gripper_joint_goal;

//     if (goal_handle->get_goal()->task_number == 0)
//     {
//       arm_joint_goal = {0.0, 0.0, 0.0};
//       //gripper_joint_goal = {-0.7, 0.7};
//     }
//     else if (goal_handle->get_goal()->task_number == 1)
//     {
//       arm_joint_goal = {-0.05, -0.05, -0.05};
//       //gripper_joint_goal = {0.0, 0.0};
//     }
//     else if (goal_handle->get_goal()->task_number == 2)
//     {
//       arm_joint_goal = {0.05,0.05,0.05};
//       //gripper_joint_goal = {0.0, 0.0};
//     }
//     else
//     {
//       RCLCPP_ERROR(get_logger(), "Invalid Task Number");
//       return;
//     }

//     bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
//     //bool gripper_within_bounds = gripper_move_group.setJointValueTarget(gripper_joint_goal);
//     //if (!arm_within_bounds | !gripper_within_bounds)
//     if (!arm_within_bounds)
//     {
//       RCLCPP_WARN(get_logger(),
//                   "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
//       return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
//     //moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
//     bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
//     //bool gripper_plan_success = (gripper_move_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
//     //if(arm_plan_success && gripper_plan_success)
//     if(arm_plan_success)
//     {
//       RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arm");
//       arm_move_group.move();
//       //gripper_move_group.move();
//     }
//     else
//     {
//       RCLCPP_ERROR(get_logger(), "One or more planners failed!");
//       return;
//     }
  
//     result->success = true;
//     goal_handle->succeed(result);
//     RCLCPP_INFO(get_logger(), "Goal succeeded");
//   }
// };
// }  // namespace arduinobot_remote

// RCLCPP_COMPONENTS_REGISTER_NODE(arduinobot_remote::TaskServer)