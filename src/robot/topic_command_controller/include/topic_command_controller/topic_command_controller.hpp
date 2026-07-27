// Copyright (c) 2026 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef TOPIC_COMMAND_CONTROLLER__TOPIC_COMMAND_CONTROLLER_HPP_
#define TOPIC_COMMAND_CONTROLLER__TOPIC_COMMAND_CONTROLLER_HPP_

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "ddt_msgs/msg/joint_control_command.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace topic_command_controller
{
using CmdType = ddt_msgs::msg::JointControlCommand;
using CmdHandle = std::optional<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>;

// The 5 command interfaces of one joint (same set the sim/hw bridge exports).
struct JointHandles
{
  CmdHandle position;
  CmdHandle velocity;
  CmdHandle effort;
  CmdHandle kp;
  CmdHandle kd;
};

class TopicCommandController : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_{"position", "velocity", "effort", "kp", "kd"};

  std::map<std::string, std::size_t> name_to_idx_;   // joint name -> index into handles_
  std::vector<JointHandles> handles_;

  rclcpp::Subscription<CmdType>::SharedPtr sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_;
};
}  // namespace topic_command_controller

#endif  // TOPIC_COMMAND_CONTROLLER__TOPIC_COMMAND_CONTROLLER_HPP_
