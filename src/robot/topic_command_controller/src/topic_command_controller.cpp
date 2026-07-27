// Copyright (c) 2026 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
//
// Passthrough controller. It runs no policy: the RL brain is an external node
// (ddt_rl_deploy/deploy/rl_inference.py) that publishes command/joint_command;
// this controller only forwards those 5 fields onto the ros2_control command
// interfaces the sim/hardware bridge exposes, so the bridge's PD applies them.
// Mirrors ddt_rl_deploy/sim/simbase.py _cmd_cb (match by joint name).

#include "topic_command_controller/topic_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace topic_command_controller
{

controller_interface::CallbackReturn TopicCommandController::on_init()
{
  joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
  command_interface_types_ =
    auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
TopicCommandController::command_interface_configuration() const
{
  std::vector<std::string> names;
  for (const auto & joint : joint_names_) {
    for (const auto & itype : command_interface_types_) {
      names.push_back(joint + "/" + itype);
    }
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
TopicCommandController::state_interface_configuration() const
{
  // Passthrough reads no state; the broadcasters publish joint_states/imu.
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::CallbackReturn TopicCommandController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "the 'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  name_to_idx_.clear();
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    name_to_idx_[joint_names_[i]] = i;
  }

  sub_ = get_node()->create_subscription<CmdType>(
    "command/joint_command", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<CmdType> msg) { rt_command_.writeFromNonRT(msg); });

  RCLCPP_INFO(
    get_node()->get_logger(), "topic_command_controller: %zu joints, sub command/joint_command",
    joint_names_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TopicCommandController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto find = [this](const std::string & joint, const std::string & itype) -> CmdHandle {
    auto it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(), [&](const auto & iface) {
        return iface.get_prefix_name() == joint && iface.get_interface_name() == itype;
      });
    if (it == command_interfaces_.end()) return std::nullopt;
    return std::ref(*it);
  };

  handles_.assign(joint_names_.size(), JointHandles{});
  for (std::size_t i = 0; i < joint_names_.size(); ++i) {
    const auto & j = joint_names_[i];
    handles_[i].position = find(j, "position");
    handles_[i].velocity = find(j, "velocity");
    handles_[i].effort = find(j, "effort");
    handles_[i].kp = find(j, "kp");
    handles_[i].kd = find(j, "kd");
    if (
      !handles_[i].position || !handles_[i].velocity || !handles_[i].effort || !handles_[i].kp ||
      !handles_[i].kd) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "joint '%s' is missing one of position/velocity/effort/kp/kd command interfaces", j.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    // Start limp (kp=kd=0, zero targets) so nothing is driven until the first
    // command arrives — avoids acting on uninitialised interface values.
    handles_[i].position->get().set_value(0.0);
    handles_[i].velocity->get().set_value(0.0);
    handles_[i].effort->get().set_value(0.0);
    handles_[i].kp->get().set_value(0.0);
    handles_[i].kd->get().set_value(0.0);
  }
  rt_command_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TopicCommandController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TopicCommandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto msg_ptr = rt_command_.readFromRT();
  if (!msg_ptr || !(*msg_ptr)) {
    return controller_interface::return_type::OK;  // no command yet -> hold last write
  }
  const auto & msg = **msg_ptr;

  for (std::size_t k = 0; k < msg.name.size(); ++k) {
    auto it = name_to_idx_.find(msg.name[k]);
    if (it == name_to_idx_.end()) continue;
    JointHandles & h = handles_[it->second];
    if (k < msg.position.size()) h.position->get().set_value(msg.position[k]);
    if (k < msg.velocity.size()) h.velocity->get().set_value(msg.velocity[k]);
    if (k < msg.effort.size()) h.effort->get().set_value(msg.effort[k]);
    if (k < msg.kp.size()) h.kp->get().set_value(msg.kp[k]);
    if (k < msg.kd.size()) h.kd->get().set_value(msg.kd[k]);
  }
  return controller_interface::return_type::OK;
}

}  // namespace topic_command_controller

PLUGINLIB_EXPORT_CLASS(
  topic_command_controller::TopicCommandController, controller_interface::ControllerInterface)
