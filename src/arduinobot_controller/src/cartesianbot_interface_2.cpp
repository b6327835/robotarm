#include "arduinobot_controller/cartesianbot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <deque>
#include <chrono>
#include <iomanip>

namespace cartesianbot_controller
{
    struct PositionCommand {
        std::vector<double> position;
        rclcpp::Time timestamp;
    };

    class CartesianbotInterface : public hardware_interface::SystemInterface
    {
    private:
        LibSerial::SerialPort arduino_;
        std::string port_;
        std::vector<double> position_commands_;
        std::vector<double> position_states_;
        std::vector<double> prev_position_commands_;
        
        // Buffer for trajectory smoothing
        std::deque<PositionCommand> command_buffer_;
        const size_t max_buffer_size_ = 10;  // Store 10 latest commands
        const double min_movement_threshold_ = 0.0001;  // 0.1mm minimum movement
        
        // Interpolation parameters
        double max_velocity_ = 0.05;  // meters per second
        rclcpp::Time last_command_time_;
        bool is_first_command_ = true;

        bool interpolateNextPosition(std::vector<double>& interpolated_position)
        {
            if (command_buffer_.empty()) return false;

            auto current_time = rclcpp::Clock().now();
            auto& target = command_buffer_.front();
            
            // Calculate time ratio for interpolation
            double time_ratio = (current_time - last_command_time_).seconds() / 
                              (target.timestamp - last_command_time_).seconds();
            
            // Clamp ratio between 0 and 1
            time_ratio = std::min(1.0, std::max(0.0, time_ratio));
            
            // Linear interpolation
            interpolated_position.resize(position_states_.size());
            for (size_t i = 0; i < position_states_.size(); i++) {
                interpolated_position[i] = position_states_[i] + 
                    (target.position[i] - position_states_[i]) * time_ratio;
            }
            
            // Remove completed command
            if (time_ratio >= 1.0) {
                command_buffer_.pop_front();
                last_command_time_ = current_time;
                return true;
            }
            
            return false;
        }

        bool shouldSendCommand(const std::vector<double>& new_position)
        {
            for (size_t i = 0; i < new_position.size(); i++) {
                if (std::abs(new_position[i] - prev_position_commands_[i]) > min_movement_threshold_) {
                    return true;
                }
            }
            return false;
        }

    public:
        CartesianbotInterface() = default;

        ~CartesianbotInterface()
        {
            if (arduino_.IsOpen())
            {
                try
                {
                    arduino_.Close();
                }
                catch (...)
                {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CartesianbotInterface"),
                                      "Error closing port " << port_);
                }
            }
        }

        CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info)
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
            catch (const std::out_of_range& e)
            {
                RCLCPP_FATAL(rclcpp::get_logger("CartesianbotInterface"),
                            "No Serial Port provided! Aborting");
                return CallbackReturn::FAILURE;
            }

            position_commands_.resize(info_.joints.size(), 0.0);
            position_states_.resize(info_.joints.size(), 0.0);
            prev_position_commands_.resize(info_.joints.size(), 0.0);

            return CallbackReturn::SUCCESS;
        }

        std::vector<hardware_interface::StateInterface> export_state_interfaces()
        {
            std::vector<hardware_interface::StateInterface> state_interfaces;
            
            for (size_t i = 0; i < info_.joints.size(); i++)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
            }

            return state_interfaces;
        }

        std::vector<hardware_interface::CommandInterface> export_command_interfaces()
        {
            std::vector<hardware_interface::CommandInterface> command_interfaces;
            
            for (size_t i = 0; i < info_.joints.size(); i++)
            {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
            }

            return command_interfaces;
        }

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state)
        {
            RCLCPP_INFO(rclcpp::get_logger("CartesianbotInterface"), "Starting robot hardware ...");

            std::fill(position_commands_.begin(), position_commands_.end(), 0.0);
            std::fill(prev_position_commands_.begin(), prev_position_commands_.end(), 0.0);
            std::fill(position_states_.begin(), position_states_.end(), 0.0);

            command_buffer_.clear();
            is_first_command_ = true;

            try
            {
                arduino_.Open(port_);
                arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
            }
            catch (...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("CartesianbotInterface"),
                                  "Failed to open port " << port_);
                return CallbackReturn::FAILURE;
            }

            RCLCPP_INFO(rclcpp::get_logger("CartesianbotInterface"),
                      "Hardware started, ready to take commands");
            return CallbackReturn::SUCCESS;
        }

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state)
        {
            RCLCPP_INFO(rclcpp::get_logger("CartesianbotInterface"), "Stopping robot hardware ...");

            if (arduino_.IsOpen())
            {
                try
                {
                    arduino_.Close();
                }
                catch (...)
                {
                    RCLCPP_FATAL_STREAM(rclcpp::get_logger("CartesianbotInterface"),
                                      "Error closing port " << port_);
                }
            }

            RCLCPP_INFO(rclcpp::get_logger("CartesianbotInterface"), "Hardware stopped");
            return CallbackReturn::SUCCESS;
        }

        hardware_interface::return_type read(const rclcpp::Time& time,
                                           const rclcpp::Duration& period)
        {
            // We're using open-loop control, so we assume the robot reached the commanded position
            if (!command_buffer_.empty()) {
                position_states_ = command_buffer_.front().position;
            }
            return hardware_interface::return_type::OK;
        }

        hardware_interface::return_type write(const rclcpp::Time& time,
                                            const rclcpp::Duration& period)
        {
            // Add new command to buffer if position changed
            if (position_commands_ != prev_position_commands_) {
                PositionCommand cmd{position_commands_, time};
                
                if (is_first_command_) {
                    last_command_time_ = time;
                    is_first_command_ = false;
                }
                
                command_buffer_.push_back(cmd);
                if (command_buffer_.size() > max_buffer_size_) {
                    command_buffer_.pop_front();
                }
                
                prev_position_commands_ = position_commands_;
            }

            // Calculate next interpolated position
            std::vector<double> next_position;
            if (!interpolateNextPosition(next_position)) {
                return hardware_interface::return_type::OK;
            }

            // Only send command if movement is significant
            if (!shouldSendCommand(next_position)) {
                return hardware_interface::return_type::OK;
            }

            // Format command string with higher precision
            std::stringstream msg;
            msg << std::fixed << std::setprecision(3);
            msg << "x" << (next_position[0] * 1000.0) << ",";  // Convert to mm
            msg << "y" << (next_position[1] * 1000.0) << ",";
            msg << "z" << (next_position[2] * 1000.0) << ",";

            try {
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("CartesianbotInterface"), 
                    "Sending command " << msg.str());
                arduino_.Write(msg.str());
            } catch (...) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("CartesianbotInterface"),
                    "Failed to send command " << msg.str() << " to port " << port_);
                return hardware_interface::return_type::ERROR;
            }

            return hardware_interface::return_type::OK;
        }
    };

} // namespace cartesianbot_controller

PLUGINLIB_EXPORT_CLASS(cartesianbot_controller::CartesianbotInterface, 
    hardware_interface::SystemInterface)