#include "motion_control/cmdmux_node.hpp"

namespace cg {
namespace cmdmux {

CmdMuxNode::CmdMuxNode() : Node("cmd_mux") {
  /* Initialize publishers and subscribers */
  // Actuator command to Arduino
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
      "/actuator_cmd", 1);

  // Multiplexer mode number
  mux_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "/mux_mode", 1, std::bind(&CmdMuxNode::muxCallback, this, std::placeholders::_1));

  // Teleop and autonomy control messages
  teleop_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
      "/teleop_cmd", 1, std::bind(&CmdMuxNode::teleopCallback, this, std::placeholders::_1));
  autonomy_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
      "/autonomy_cmd", 1, std::bind(&CmdMuxNode::autonomyCallback, this, std::placeholders::_1));

  
  /* Load parameters */
  // Initialize the currenet mode to the default mode
  this->declare_parameter<int>("default_mode", 0);
  this->get_parameter("default_mode", curr_mode_);

  // Set values for other modes based on config file
  this->declare_parameter<int>("idle_mode", 0);
  this->get_parameter("idle_mode", idle_mode_);
  this->declare_parameter<int>("autograder_mode", 1);
  this->get_parameter("autograder_mode", autograder_mode_);
  this->declare_parameter<int>("full_autonomy_mode", 2);
  this->get_parameter("full_autonomy_mode", full_autonomy_mode_);
  this->declare_parameter<int>("full_teleop_mode", 3);
  this->get_parameter("full_teleop_mode", full_teleop_mode_);
}

void modeCallback(const std_msgs::msg::Int8::SharedPtr msg) {
  // Set the current mode using the incoming mode number
  curr_mode_ = msg.data;
}

void teleopCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {

  switch (curr_mode_) {
    case idle_mode_:
      // Fix to last command
      // TODO: set these values correctly
      actuator_cmd_.wheel_velocity = 0;
      actuator_cmd_.steer_position = 0;
      actuator_cmd_.tool_position = 0;
      break;
    case autograder_mode_:
      break;
    case full_autonomy_mode_:
      break;
    case full_teleop_mode_:
      break;
    default:
      // Report the current mode
  }

}

void autonomyCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg)
{
  return;
}

} // namespace cmdmux
} // namespace cg