// Simulation bridge: forwards ddt_msgs/JointControlCommand from the
// command/joint_command topic into ros2_control loaned command interfaces,
// emulating the vendor d1_bringup seam so the no-ROS SDK can be tested in sim.
#ifndef TOPIC_COMMAND_CONTROLLER__TOPIC_COMMAND_CONTROLLER_HPP_
#define TOPIC_COMMAND_CONTROLLER__TOPIC_COMMAND_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ddt_msgs/msg/joint_control_command.hpp"
#include "rclcpp/rclcpp.hpp"

namespace topic_command_controller
{

class TopicCommandController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  static constexpr size_t kFields = 5;  // position, velocity, effort, kp, kd

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, size_t> joint_index_;

  rclcpp::Subscription<ddt_msgs::msg::JointControlCommand>::SharedPtr sub_;
  std::mutex msg_mutex_;
  ddt_msgs::msg::JointControlCommand::SharedPtr latest_msg_;
};

}  // namespace topic_command_controller

#endif  // TOPIC_COMMAND_CONTROLLER__TOPIC_COMMAND_CONTROLLER_HPP_
