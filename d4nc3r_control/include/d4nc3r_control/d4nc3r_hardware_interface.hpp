#ifndef D4NC3R_HARDWARE_INTERFACE_HPP_
#define D4NC3R_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "rclcpp/executors.hpp"

namespace d4nc3r_control
{

class D4nc3rHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(D4nc3rHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  // ROS 2 Node and Subscriber for encoders
  rclcpp::Node::SharedPtr subscriber_node_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr encoder_subscriber_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::mutex data_mutex_;

  // Parameters
  std::string encoder_topic_name_ = "/encoders"; // Default, can be overridden by URDF
  double ticks_per_wheel_revolution_ = 8.0; // Example: ticks for one full wheel revolution
  int left_encoder_data_index_ = 1;
  int right_encoder_data_index_ = 0;


  // Store the state of the robot
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_; // For velocity commands from diff_drive_controller

  // Joint names
  std::string left_wheel_joint_name_;
  std::string right_wheel_joint_name_;
  int left_wheel_joint_index_ = -1;
  int right_wheel_joint_index_ = -1;


  // Last known encoder values (cumulative)
  std::atomic<int32_t> current_left_encoder_ticks_{0};
  std::atomic<int32_t> current_right_encoder_ticks_{0};
  std::atomic<bool> new_encoder_data_received_{false};

  // For velocity calculation
  double prev_left_wheel_pos_ = 0.0;
  double prev_right_wheel_pos_ = 0.0;
  rclcpp::Time prev_time_;
};

}  // namespace d4nc3r_control

#endif  // D4NC3R_HARDWARE_INTERFACE_HPP_
