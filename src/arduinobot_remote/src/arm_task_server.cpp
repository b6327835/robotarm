// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include "arduinobot_msgs/action/arm_task.hpp"
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
//     action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArmTask>(
//         this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
//         std::bind(&TaskServer::cancelCallback, this, _1),
//         std::bind(&TaskServer::acceptedCallback, this, _1));
//   }

// private:
//   rclcpp_action::Server<arduinobot_msgs::acticd on::ArmTask>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse goalCallback(
//       const rclcpp_action::GoalUUID& uuid,
//       std::shared_ptr<const arduinobot_msgs::action::ArmTask::Goal> goal)
//   {
//     RCLCPP_INFO(get_logger(), "Received goal request with XYZ coordinates: [%f, %f, %f] and jog step: %f",
//                 goal->x, goal->y, goal->z, goal->jog_step);
//     (void)uuid;
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse cancelCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
//   {
//     (void)goal_handle;
//     RCLCPP_INFO(get_logger(), "Received request to cancel goal");
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
//     arm_move_group.stop();
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void acceptedCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
//   {
//     std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
//   }

//   void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
//   {
//     RCLCPP_INFO(get_logger(), "Executing goal");
//     auto result = std::make_shared<arduinobot_msgs::action::ArmTask::Result>();

//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");

//     // Convert XYZ and jog_step to joint positions if necessary
//     std::vector<double> arm_joint_goal = {
//         goal_handle->get_goal()->x,
//         goal_handle->get_goal()->y,
//         goal_handle->get_goal()->z
//     };

//     // Apply jog_step if needed
//     for (auto& joint : arm_joint_goal) {
//         joint += goal_handle->get_goal()->jog_step;
//     }

//     bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
//     if (!arm_within_bounds)
//     {
//       RCLCPP_WARN(get_logger(),
//                   "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
//       return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
//     bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
//     if(arm_plan_success)
//     {
//       RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arm");
//       arm_move_group.move();
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










// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp_action/rclcpp_action.hpp>
// #include <rclcpp_components/register_node_macro.hpp>
// #include "arduinobot_msgs/action/arm_task.hpp"
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
//     action_server_ = rclcpp_action::create_server<arduinobot_msgs::action::ArmTask>(
//         this, "task_server", std::bind(&TaskServer::goalCallback, this, _1, _2),
//         std::bind(&TaskServer::cancelCallback, this, _1),
//         std::bind(&TaskServer::acceptedCallback, this, _1));
//   }

// private:
//   rclcpp_action::Server<arduinobot_msgs::action::ArmTask>::SharedPtr action_server_;

//   rclcpp_action::GoalResponse goalCallback(
//       const rclcpp_action::GoalUUID& uuid,
//       std::shared_ptr<const arduinobot_msgs::action::ArmTask::Goal> goal)
//   {
//     RCLCPP_INFO(get_logger(), "Received goal request with coordinates: [%f, %f, %f], jog_step: %f",
//                 goal->x, goal->y, goal->z, goal->jog_step);
//     (void)uuid;
//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
//   }

//   rclcpp_action::CancelResponse cancelCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
//   {
//     (void)goal_handle;
//     RCLCPP_INFO(get_logger(), "Received request to cancel goal");
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");
//     arm_move_group.stop();
//     return rclcpp_action::CancelResponse::ACCEPT;
//   }

//   void acceptedCallback(
//       const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
//   {
//     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
//     std::thread{ std::bind(&TaskServer::execute, this, _1), goal_handle }.detach();
//   }

//   void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<arduinobot_msgs::action::ArmTask>> goal_handle)
//   {
//     RCLCPP_INFO(get_logger(), "Executing goal");
//     auto result = std::make_shared<arduinobot_msgs::action::ArmTask::Result>();

//     // MoveIt 2 Interface
//     auto arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "arm");

//     // Convert XYZ and jog_step to joint positions if needed
//     std::vector<double> arm_joint_goal = {
//       goal_handle->get_goal()->x,
//       goal_handle->get_goal()->y,
//       goal_handle->get_goal()->z
//     };

//     bool arm_within_bounds = arm_move_group.setJointValueTarget(arm_joint_goal);
//     if (!arm_within_bounds)
//     {
//       RCLCPP_WARN(get_logger(),
//                   "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
//       return;
//     }

//     moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
//     bool arm_plan_success = (arm_move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

//     if(arm_plan_success)
//     {
//       RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the arm");
//       arm_move_group.move();
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

