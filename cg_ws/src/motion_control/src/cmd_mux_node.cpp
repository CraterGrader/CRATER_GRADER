#include "motion_control/cmdmux_node.hpp"

namespace cg {
namespace cmdmux {

CmdMuxNode::CmdMuxNode() : Node("cmd_mux") {
  /* Initialize publishers and subscribers */
  // Actuator command to Arduino
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
      "/actuator_cmd", 1);

  // Multiplexer mode number
  mode_sub_ = this->create_subscription<cg_msgs::msg::MuxMode>(
      "/mux_mode", 1, std::bind(&CmdMuxNode::modeCallback, this, std::placeholders::_1));

  // Teleop and autonomy control messages
  teleop_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
      "/teleop_cmd", 1, std::bind(&CmdMuxNode::teleopCallback, this, std::placeholders::_1));
  autonomy_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
      "/autonomy_cmd", 1, std::bind(&CmdMuxNode::autonomyCallback, this, std::placeholders::_1));

  // Timer callback for idle mode
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CmdMuxNode::timerCallback, this)
  );

  // Initialize the current mode to the default mode
  curr_mode_ = cg_msgs::msg::MuxMode::IDLE;
}

void CmdMuxNode::timerCallback()
{
  // Report the current mode
  RCLCPP_INFO(this->get_logger(), "Current multiplexer mode: %d", curr_mode_);

  // Handle message based on current multiplexer mode
  if (curr_mode_ == cg_msgs::msg::MuxMode::IDLE)
  {
    // Publish last message, with wheel velocity set to zero
    cmd_msg_.wheel_velocity = 0;
  } 
  // Publish the message
  cmd_msg_.header.stamp = this->get_clock()->now();
  cmd_pub_->publish(cmd_msg_); // Keep using the same message, so last message is retained unless changed
}

void CmdMuxNode::modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg)
{

  // Check for valid input
  if (msg->mode > cg_msgs::msg::MuxMode::HIGHEST_VALID_MODE)
  {
    // Warn about invalid mode input
    RCLCPP_WARN(this->get_logger(), "Unsupported multiplexer mode request: %d", msg->mode);
    return; // Don't let the mode be updated
  }

  // Set the current mode using the incoming mode number
  curr_mode_ = msg->mode;
}

void CmdMuxNode::teleopCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {

  // Handle message based on current multiplexer mode
  if (curr_mode_ == cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    // Update only the wheel velocity and steering position inputs
    cmd_msg_.wheel_velocity = msg->wheel_velocity;
    cmd_msg_.steer_position = msg->steer_position;
  }
  else if (curr_mode_ == cg_msgs::msg::MuxMode::FULL_TELEOP)
  {
    // Update the command message directly
    cmd_msg_ = *msg;
  }
}

void CmdMuxNode::autonomyCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg)
{

  // Handle message based on current multiplexer mode
  if (curr_mode_ == cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    // Update only the tool position
    cmd_msg_.tool_position = msg->tool_position;
  }
  else if (curr_mode_ == cg_msgs::msg::MuxMode::FULL_AUTONOMY)
  {
    // Update the command message directly
    cmd_msg_ = *msg;
  }
}

} // namespace cmdmux
} // namespace cg