#include "topic_command_controller/topic_command_controller.hpp"

#include <algorithm>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace topic_command_controller
{

namespace
{
const char * kFieldNames[] = {"position", "velocity", "effort", "kp", "kd"};
}

controller_interface::CallbackReturn TopicCommandController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TopicCommandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joint_names_) {
    for (const auto * field : kFieldNames) {
      config.names.push_back(joint + "/" + field);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
TopicCommandController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn TopicCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  joint_index_.clear();
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    joint_index_[joint_names_[i]] = i;
  }

  sub_ = get_node()->create_subscription<ddt_msgs::msg::JointControlCommand>(
    "/command/joint_command", rclcpp::QoS(10),
    [this](const ddt_msgs::msg::JointControlCommand::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(msg_mutex_);
      latest_msg_ = msg;
    });

  RCLCPP_INFO(
    get_node()->get_logger(), "configured: %zu joints, subscribing /command/joint_command",
    joint_names_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TopicCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    latest_msg_.reset();  // stale commands from a previous activation are discarded
  }
  for (auto & itf : command_interfaces_) {
    itf.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TopicCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & itf : command_interfaces_) {
    itf.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TopicCommandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ddt_msgs::msg::JointControlCommand::SharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(msg_mutex_);
    msg = latest_msg_;
  }
  if (!msg) {
    return controller_interface::return_type::OK;  // limp (all zeros) until first command
  }

  const size_t n = msg->name.size();
  for (size_t k = 0; k < n; ++k) {
    auto it = joint_index_.find(msg->name[k]);
    if (it == joint_index_.end()) {
      continue;
    }
    const size_t base = it->second * kFields;
    const double values[kFields] = {
      k < msg->position.size() ? msg->position[k] : 0.0,
      k < msg->velocity.size() ? msg->velocity[k] : 0.0,
      k < msg->effort.size() ? msg->effort[k] : 0.0,
      k < msg->kp.size() ? msg->kp[k] : 0.0,
      k < msg->kd.size() ? msg->kd[k] : 0.0,
    };
    for (size_t f = 0; f < kFields; ++f) {
      command_interfaces_[base + f].set_value(values[f]);
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace topic_command_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  topic_command_controller::TopicCommandController, controller_interface::ControllerInterface)
