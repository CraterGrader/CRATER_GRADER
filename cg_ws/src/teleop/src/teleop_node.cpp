#include "teleop/teleop_node.hpp"

namespace cg {
namespace teleop {

TeleopNode::TeleopNode() : Node("teleop_node") {
  // Initialize publishers and subscribers
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
    "/teleop_cmd", 1
  );
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&TeleopNode::joyCallback, this, std::placeholders::_1)
  );
  act_cmd_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
      "/actuator_cmd", 1, std::bind(&TeleopNode::actCmdCallback, this, std::placeholders::_1));
  mode_sub_ = this->create_subscription<cg_msgs::msg::MuxMode>(
      "/mux_mode", 1, std::bind(&TeleopNode::modeCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TeleopNode::timerCallback, this)
  );

  // Load parameters
  this->declare_parameter<int>("axis_drive", 1);
  this->get_parameter("axis_drive", joy_axis_drive_i_);
  this->declare_parameter<int>("axis_steer", 0);
  this->get_parameter("axis_steer", joy_axis_steer_i_);
  this->declare_parameter<int>("tool_raise", 0);
  this->get_parameter("tool_raise", joy_raise_tool_i_);
  this->declare_parameter<int>("tool_lower", 0);
  this->get_parameter("tool_lower", joy_lower_tool_i_);
  this->declare_parameter<int>("tool_increment", 0);
  this->get_parameter("tool_increment", joy_tool_increment_);

  // Initialize the current mode to the default mode, until new message is received
  curr_mux_mode_ = cg_msgs::msg::MuxMode::IDLE;
}

void TeleopNode::timerCallback() {
  auto cmd_msg = cg_msgs::msg::ActuatorCommand();

  // Only publish if in teleop or autograder mode (still publish a full message for autograder; the tool position is ignored by cmdmux_node)
  if (curr_mux_mode_ == cg_msgs::msg::MuxMode::FULL_TELEOP || curr_mux_mode_ == cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    // Joystick states are in range [-1, 1]
    // Scale to the range [-100, 100] % of full scale velocity
    cmd_msg.wheel_velocity = std::max(-100.0, std::min(100 * joy_axis_drive_state_, 100.0));
    cmd_msg.steer_position = std::max(-100.0, std::min(100 * joy_axis_steer_state_, 100.0));
    cmd_msg.tool_position = joy_tool_height_state_;
    cmd_msg.header.stamp = this->get_clock()->now();
    cmd_pub_->publish(cmd_msg);
  }
}

void TeleopNode::modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg)
{

  // Check for valid input
  if (msg->mode > cg_msgs::msg::MuxMode::HIGHEST_VALID_MODE)
  {
    // Warn about invalid mode input
    RCLCPP_WARN(this->get_logger(), "Unsupported multiplexer mode request: %d", msg->mode);
    return; // Don't let the mode be updated
  }

  // Set the current mode using the incoming mode number
  curr_mux_mode_ = msg->mode;
}

void TeleopNode::actCmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg)
{
  // Update drive and steer commands whenever there is no direct control on the those commands
  if (curr_mux_mode_ != cg_msgs::msg::MuxMode::FULL_TELEOP && curr_mux_mode_ != cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    // Scale from range [-100, 100] % of full scale velocity to joystick value range [-1, 1]
    joy_axis_drive_state_ = std::max(-1.0, std::min(msg->wheel_velocity / 100, 1.0)); // Absolute value should get updated immediately, prior value should have no effect but kept for posterity
    joy_axis_steer_state_ = std::max(-1.0, std::min(msg->steer_position / 100, 1.0)); // Absolute value should get updated immediately, prior value should have no effect but kept for posterity
  }

  // Update tool position whenever there is no direct control on the tool position
  if (curr_mux_mode_ != cg_msgs::msg::MuxMode::FULL_TELEOP) {
    joy_tool_height_state_ = msg->tool_position; // Tool will update incrementally starting from this value
  }
}

void TeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {

  // Change drive and steer in both teleop and autograder modes
  if (curr_mux_mode_ == cg_msgs::msg::MuxMode::FULL_TELEOP || curr_mux_mode_ == cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    joy_axis_drive_state_ = msg->axes[joy_axis_drive_i_];
    joy_axis_steer_state_ = msg->axes[joy_axis_steer_i_];
  }

  // Only allow tool to change if in full teleop mode
  if (curr_mux_mode_ == cg_msgs::msg::MuxMode::FULL_TELEOP) {
    if (joy_tool_pressed_)
    {
      if (msg->buttons[joy_raise_tool_i_] == 0 && msg->buttons[joy_lower_tool_i_] == 0)
      {
        joy_tool_pressed_ = false;
      }
    } else {
    //If lower bumper pressed, then lower by 5%
    if (msg->buttons[joy_lower_tool_i_]) {
      joy_tool_pressed_ = true;  
      joy_tool_height_state_ = std::max(0.0, joy_tool_height_state_ - 5.0);
    }    
    //If raise bumper pressed, then raise by 5%
    if (msg->buttons[joy_raise_tool_i_]) {
      joy_tool_pressed_ = true;  
      joy_tool_height_state_ = std::min(100.0, joy_tool_height_state_ + 5.0);
    }
    // Mark bumpers as pressed to avoid sticky button effects
  }
  }
}

}  // namespace teleop
}  // namespace cg
