#include "d4nc3r_control/d4nc3r_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace d4nc3r_control
{
// Helper function to check if a string ends with a specific suffix
bool ends_with(const std::string& value, const std::string& suffix) {
    if (suffix.size() > value.size()) return false;
    return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin());
}


hardware_interface::CallbackReturn D4nc3rHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Configuring D4nc3rHardwareInterface: %s", info_.name.c_str());

  // Get parameters from URDF
  if (info_.hardware_parameters.count("encoder_topic_name"))
  {
    encoder_topic_name_ = info_.hardware_parameters["encoder_topic_name"];
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Using encoder topic: %s", encoder_topic_name_.c_str());
  }
  if (info_.hardware_parameters.count("ticks_per_wheel_revolution"))
  {
    ticks_per_wheel_revolution_ = std::stod(info_.hardware_parameters["ticks_per_wheel_revolution"]);
     RCLCPP_INFO(rclcpp::get_logger(info_.name), "Ticks per wheel revolution: %.2f", ticks_per_wheel_revolution_);
  }
   if (info_.hardware_parameters.count("left_encoder_data_index"))
  {
    left_encoder_data_index_ = std::stoi(info_.hardware_parameters["left_encoder_data_index"]);
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Using left encoder data index: %d", left_encoder_data_index_);
  }
  if (info_.hardware_parameters.count("right_encoder_data_index"))
  {
    right_encoder_data_index_ = std::stoi(info_.hardware_parameters["right_encoder_data_index"]);
    RCLCPP_INFO(rclcpp::get_logger(info_.name), "Using right encoder data index: %d", right_encoder_data_index_);
  }


  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(info_.name),
      "Expects exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::string LEFT_WHEEL_BASE_NAME = "left_wheel_joint";
  const std::string RIGHT_WHEEL_BASE_NAME = "right_wheel_joint";

  // Initialize storage
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Find left and right wheel joints based on common names or parameters
  // The info_.joints[i].name will be the fully prefixed name from the URDF.
  for (size_t i = 0; i < info_.joints.size(); ++i) {
      if (ends_with(info_.joints[i].name, LEFT_WHEEL_BASE_NAME)) {
          left_wheel_joint_name_ = info_.joints[i].name;
          left_wheel_joint_index_ = i;
          RCLCPP_INFO(rclcpp::get_logger(info_.name), "Found left wheel joint: %s at index %zu", left_wheel_joint_name_.c_str(), i);
      } else if (ends_with(info_.joints[i].name, RIGHT_WHEEL_BASE_NAME)) {
          right_wheel_joint_name_ = info_.joints[i].name;
          right_wheel_joint_index_ = i;
          RCLCPP_INFO(rclcpp::get_logger(info_.name), "Found right wheel joint: %s at index %zu", right_wheel_joint_name_.c_str(), i);
      }
  }

  if (left_wheel_joint_index_ == -1 || right_wheel_joint_index_ == -1) {
      RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Could not find 'left_wheel_joint' or 'right_wheel_joint' suffixes. Check URDF and controller config.");
      return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Check for expected state interfaces
  for (const auto& joint : info_.joints) {
    for (const auto& si : joint.state_interfaces) {
      if (si.name != hardware_interface::HW_IF_POSITION && si.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Joint '%s' has unexpected state interface '%s'", joint.name.c_str(), si.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Configuration successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn D4nc3rHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Configuring subscriber node...");
  // Create a dedicated node for the subscriber
  // The node name should be unique in the ROS graph.
  subscriber_node_ = rclcpp::Node::make_shared("d4nc3r_encoder_subscriber_node", "d4nc3r1"); // Adiciona o namespace
  if (!subscriber_node_) {
      RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Failed to create subscriber node.");
      return hardware_interface::CallbackReturn::ERROR;
  }

  encoder_subscriber_ = subscriber_node_->create_subscription<std_msgs::msg::Int32MultiArray>(
    encoder_topic_name_, 
    rclcpp::SystemDefaultsQoS(), // Or a specific QoS
    std::bind(&D4nc3rHardwareInterface::encoder_callback, this, std::placeholders::_1));
  
  if (!encoder_subscriber_) {
      RCLCPP_ERROR(rclcpp::get_logger(info_.name), "Failed to create encoder subscriber for topic: %s", encoder_topic_name_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize positions and velocities to 0
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  prev_left_wheel_pos_ = 0.0;
  prev_right_wheel_pos_ = 0.0;
  prev_time_ = subscriber_node_->now(); // Or rclcpp::Clock().now() if node not fully ready

  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Subscriber configuration successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> D4nc3rHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Exported state interfaces.");
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> D4nc3rHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Exported command interfaces (velocity).");
  return command_interfaces;
}


hardware_interface::CallbackReturn D4nc3rHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Activating D4nc3rHardwareInterface...");
  // Start the executor for the subscriber node in a separate thread
  if (subscriber_node_ && !executor_) { // Ensure node exists and executor not already started
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(subscriber_node_->get_node_base_interface()); // Use get_node_base_interface()
    executor_thread_ = std::thread([this]() { 
        // subscriber_node_ should be valid here as this is in on_activate after its creation
        // Use subscriber_node_'s logger for messages related to its executor
        RCLCPP_INFO(this->subscriber_node_->get_logger(), "Starting executor spin for subscriber node.");
        this->executor_->spin(); 
        RCLCPP_INFO(this->subscriber_node_->get_logger(), "Executor spin finished for subscriber node.");
    });
  }
  
  // Reset states
  for (size_t i = 0; i < hw_positions_.size(); ++i) {
    hw_positions_[i] = 0.0; // Or load last known absolute position if available
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  current_left_encoder_ticks_.store(0); // Assuming encoders reset or start from a known zero on activation
  current_right_encoder_ticks_.store(0);
  new_encoder_data_received_.store(false);
  prev_left_wheel_pos_ = 0.0;
  prev_right_wheel_pos_ = 0.0;
  if (subscriber_node_) { // Ensure node exists
    prev_time_ = subscriber_node_->now();
  } else {
    // Fallback if node isn't ready, though it should be by on_activate
    prev_time_ = rclcpp::Clock(RCL_ROS_TIME).now(); 
  }


  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Activation successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn D4nc3rHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Deactivating D4nc3rHardwareInterface...");
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  executor_.reset(); // Release the executor
  // Subscriber and node are typically kept until destruction or re-configuration
  RCLCPP_INFO(rclcpp::get_logger(info_.name), "Deactivation successful.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void D4nc3rHardwareInterface::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  if (msg->layout.dim.empty() || msg->layout.dim[0].size < 2) {
    RCLCPP_WARN_THROTTLE(subscriber_node_->get_logger(), *subscriber_node_->get_clock(), 1000, 
                         "Encoder data has insufficient size. Expected at least 2 elements.");
    return;
  }
  if (msg->data.size() < static_cast<size_t>(std::max(left_encoder_data_index_, right_encoder_data_index_) + 1)) {
      RCLCPP_WARN_THROTTLE(subscriber_node_->get_logger(), *subscriber_node_->get_clock(), 1000,
                           "Encoder data array too small for specified indices.");
      return;
  }

  current_left_encoder_ticks_.store(msg->data[left_encoder_data_index_]);
  current_right_encoder_ticks_.store(msg->data[right_encoder_data_index_]);
  new_encoder_data_received_.store(true);
  // RCLCPP_DEBUG(subscriber_node_->get_logger(), "Received encoder ticks: L=%d, R=%d", 
  //             current_left_encoder_ticks_.load(), current_right_encoder_ticks_.load());
}

hardware_interface::return_type D4nc3rHardwareInterface::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (ticks_per_wheel_revolution_ <= 0) {
      // Assuming subscriber_node_ is valid if read() is called in an active state.
      // If not, there's a more fundamental issue with the lifecycle.
      RCLCPP_ERROR_THROTTLE(
          subscriber_node_->get_logger(),
          *subscriber_node_->get_clock(),
          1000,  // Throttle duration in milliseconds
          "ticks_per_wheel_revolution is not positive (%.2f). Cannot calculate position.", ticks_per_wheel_revolution_);
      return hardware_interface::return_type::ERROR;
  }
  
  double left_pos = 0.0;
  double right_pos = 0.0;

  // Assuming encoder ticks are cumulative
  left_pos = (static_cast<double>(current_left_encoder_ticks_.load()) / ticks_per_wheel_revolution_) * 2.0 * M_PI;
  right_pos = (static_cast<double>(current_right_encoder_ticks_.load()) / ticks_per_wheel_revolution_) * 2.0 * M_PI;

  if (left_wheel_joint_index_ != -1) {
    hw_positions_[left_wheel_joint_index_] = left_pos;
  }
  if (right_wheel_joint_index_ != -1) {
    hw_positions_[right_wheel_joint_index_] = right_pos;
  }

  // Calculate velocities
  double delta_seconds = period.seconds();
  if (delta_seconds > 1e-6) // Avoid division by zero if period is too small
  {
    if (left_wheel_joint_index_ != -1) {
        hw_velocities_[left_wheel_joint_index_] = (left_pos - prev_left_wheel_pos_) / delta_seconds;
        prev_left_wheel_pos_ = left_pos;
    }
    if (right_wheel_joint_index_ != -1) {
        hw_velocities_[right_wheel_joint_index_] = (right_pos - prev_right_wheel_pos_) / delta_seconds;
        prev_right_wheel_pos_ = right_pos;
    }
  } else { // If period is too small, assume zero velocity change or keep last
      if (left_wheel_joint_index_ != -1) hw_velocities_[left_wheel_joint_index_] = 0.0;
      if (right_wheel_joint_index_ != -1) hw_velocities_[right_wheel_joint_index_] = 0.0;
  }
  prev_time_ = time;
  new_encoder_data_received_.store(false); 
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type D4nc3rHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace d4nc3r_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  d4nc3r_control::D4nc3rHardwareInterface, hardware_interface::SystemInterface)
