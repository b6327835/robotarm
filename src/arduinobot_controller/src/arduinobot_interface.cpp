#include "arduinobot_controller/arduinobot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <iomanip>
#include <sstream>

namespace arduinobot_controller
{
ArduinobotInterface::ArduinobotInterface()
{
}


std::string format_double(double value, int precision = 3)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

ArduinobotInterface::~ArduinobotInterface()
{
  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }
}


CallbackReturn ArduinobotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS)
  {
    return result;
  }

  try
  {
    port_ = info_.hardware_parameters.at("port");
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("ArduinobotInterface"), "No Serial Port provided! Aborting");
    return CallbackReturn::FAILURE;
  }

  position_commands_.reserve(info_.joints.size());
  position_states_.reserve(info_.joints.size());
  prev_position_commands_.reserve(info_.joints.size());

  return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> ArduinobotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface> ArduinobotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Provide only a position Interafce
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
  }

  return command_interfaces;
}


CallbackReturn ArduinobotInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Starting robot hardware ...");

  // Reset commands and states
  position_commands_ = { 0,0,0};
  prev_position_commands_ = { 0,0,0};
  position_states_ = { 0,0,0};

  try
  {
    arduino_.Open(port_);
    arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_250000);
  }
  catch (...)
  {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                        "Something went wrong while interacting with port " << port_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"),
              "Hardware started, ready to take commands");
  return CallbackReturn::SUCCESS;
}


CallbackReturn ArduinobotInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Stopping robot hardware ...");

  if (arduino_.IsOpen())
  {
    try
    {
      arduino_.Close();
    }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                          "Something went wrong while closing connection with port " << port_);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ArduinobotInterface"), "Hardware stopped");
  return CallbackReturn::SUCCESS;
}


hardware_interface::return_type ArduinobotInterface::read(const rclcpp::Time &time,
                                                          const rclcpp::Duration &period)
{
  // Open Loop Control - assuming the robot is always where we command to be
  position_states_ = position_commands_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinobotInterface::write(const rclcpp::Time &time,
                                                           const rclcpp::Duration &period)
{

  if (position_commands_ == prev_position_commands_)
  {
    // Nothing changed, do not send any command
    return hardware_interface::return_type::OK;
  }

  // Validate coordinates
  for (const auto& pos : position_commands_) {
    if (!std::isfinite(pos)) {
      RCLCPP_ERROR(rclcpp::get_logger("ArduinobotInterface"), "Invalid position value detected");
      return hardware_interface::return_type::ERROR;
    }
  }

  std::stringstream msg;
  // Convert to millimeters with 3 decimal places precision
  msg << "x" << std::fixed << std::setprecision(3) << (position_commands_[0] * 1000.0) << ","
      << "y" << std::fixed << std::setprecision(3) << (position_commands_[1] * 1000.0) << ","
      << "z" << std::fixed << std::setprecision(3) << (position_commands_[2] * 1000.0) << ",";

  try
  {
    std::string command = msg.str();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("ArduinobotInterface"), 
                      "(Arduino_bot) Sending command: " << command);
    arduino_.Write(command);
    prev_position_commands_ = position_commands_;
    return hardware_interface::return_type::OK;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("ArduinobotInterface"),
                       "Failed to send command: " << e.what());
    return hardware_interface::return_type::ERROR;
  }
}
}  // namespace arduinobot_controller

PLUGINLIB_EXPORT_CLASS(arduinobot_controller::ArduinobotInterface, hardware_interface::SystemInterface)
