#include "motion_control/cmdmux_node.hpp"

namespace cg {
namespace cmdmux {

CmdMuxNode::CmdMuxNode() : Node("cmd_mux") {
  /* Initialize publishers and subscribers */
  // Actuator command to Arduino
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
      "/actuator_cmd", 1);

  // Multiplexer mode number
  mode_sub_ = this->create_subscription<std_msgs::msg::Int8>(
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

void CmdMuxNode::timerCallback()
{
  // Report the current mode
  RCLCPP_INFO(this->get_logger(), "Current multiplexer mode: %d", curr_mode_);

  // Publish last message when in Idle mode
  if (curr_mode_ == idle_mode_)
  {
    // Publish last message, with wheel velocity set to zero
    last_cmd_.header.stamp = this->get_clock()->now();
    last_cmd_.wheel_velocity = 0;
    cmd_pub_->publish(last_cmd_);
  }
}

void CmdMuxNode::modeCallback(const std_msgs::msg::Int8::SharedPtr msg)
{

  // Check for valid input
  if (curr_mode_ != idle_mode_ && curr_mode_ != autograder_mode_ && curr_mode_ != full_autonomy_mode_ && curr_mode_ != full_teleop_mode_)
  {
    // Warn about invalid mode input
    RCLCPP_WARN(this->get_logger(), "Unsupported multiplexer mode request: %d", curr_mode_);
    return;
  }

  // Set the current mode using the incoming mode number
  curr_mode_ = msg->data;
}

void CmdMuxNode::teleopCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {

  // Handle message based on current multiplexer mode
  if (curr_mode_ == idle_mode_) {
    // Publish last message only, with wheel velocity set to zero
    cmd_msg_ = last_cmd_;
    cmd_msg_.wheel_velocity = 0;
  }
  else if (curr_mode_ == autograder_mode_)
  {
    // Use only the wheel velocity and steering position inputs
    cmd_msg_.wheel_velocity = msg->wheel_velocity;
    cmd_msg_.steer_position = msg->steer_position;
    autograder_teleop_received_ = true; // Set flag

    // Make publish attempt then leave callback
    asyncPublishAutograder();
    return;
  }
  else if (curr_mode_ == full_autonomy_mode_)
  {
    // Don't publish anything
    return;
  }
  else if (curr_mode_ == full_teleop_mode_) {
    // Use the command message directly
    cmd_msg_ = *msg;
  }
  // Report the current mode
  RCLCPP_INFO(this->get_logger(), "Current multiplexer mode: %d", curr_mode_);

  // Publish the message
  cmd_msg_.header.stamp = this->get_clock()->now();
  last_cmd_ = cmd_msg_; // Update the last message published
  cmd_pub_->publish(cmd_msg_);
}

void CmdMuxNode::autonomyCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg)
{

  // Handle message based on current multiplexer mode
  if (curr_mode_ == idle_mode_)
  {
    // Publish last message only, with wheel velocity set to zero
    cmd_msg_ = last_cmd_;
    cmd_msg_.wheel_velocity = 0;
  }
  else if (curr_mode_ == autograder_mode_)
  {
    // Use only the tool position
    cmd_msg_.tool_position = msg->tool_position;
    autograder_autonomy_received_ = true; // Set flag

    // Make publish attempt then leave callback
    asyncPublishAutograder();
    return;
  }
  else if (curr_mode_ == full_autonomy_mode_)
  {
    // Use the command message directly
    cmd_msg_ = *msg;
  }
  else if (curr_mode_ == full_teleop_mode_)
  {
    // Don't publish anything
    return;
  }
  // Report the current mode
  RCLCPP_INFO(this->get_logger(), "Current multiplexer mode: %d", curr_mode_);

  // Publish the message
  cmd_msg_.header.stamp = this->get_clock()->now();
  last_cmd_ = cmd_msg_; // Update the last message published
  cmd_pub_->publish(cmd_msg_);
}

void CmdMuxNode::asyncPublishAutograder() {
  // Handle cases of some or no input received, otherwise use fully updated message
  if (!autograder_teleop_received_ && !autograder_autonomy_received_)
  {
    // Return without publishing if neither input sources have been received
    return;
  }
  else if (autograder_teleop_received_ && !autograder_autonomy_received_)
  {
    // Use teleop commands and repeat the autonomy tool commands
    cmd_msg_.tool_position = last_cmd_.tool_position;
  }
  else if (!autograder_teleop_received_ && autograder_autonomy_received_)
  {
    // Use autonomy command and repeat the teleop commands
    cmd_msg_.wheel_velocity = last_cmd_.wheel_velocity;
    cmd_msg_.steer_position = last_cmd_.steer_position;
  }

  // Reset flags
  autograder_teleop_received_ = false;
  autograder_autonomy_received_ = false;

  // Publish the message
  cmd_msg_.header.stamp = this->get_clock()->now();
  last_cmd_ = cmd_msg_; // Update the last message published
  cmd_pub_->publish(cmd_msg_);
}

} // namespace cmdmux
} // namespace cg