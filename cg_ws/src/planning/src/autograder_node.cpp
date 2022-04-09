#include "planning/autograder_node.hpp"

namespace cg {
namespace planning {

AutoGraderNode::AutoGraderNode() : Node("autograder_node") {
  // Initialize publishers and subscribers
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&AutoGraderNode::timerCallback, this)
  );

  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
    "/autonomy_cmd", 1
  );

  slip_sub_ = this->create_subscription<cg_msgs::msg::Slip>(
    "/slip_estimate", 1, std::bind(&AutoGraderNode::slipCallback, this, std::placeholders::_1));

  mode_sub_ = this->create_subscription<cg_msgs::msg::MuxMode>(
    "/mux_mode", 1, std::bind(&AutoGraderNode::modeCallback, this, std::placeholders::_1));


  // Load parameters
  this->declare_parameter<float>("slip_thresh", 0.3);
  this->get_parameter("slip_thresh", slip_thresh_);
  this->declare_parameter<float>("half_deadband", 0.05);
  this->get_parameter("half_deadband", half_deadband_);
  this->declare_parameter<int>("design_blade_pos", 60);
  this->get_parameter("design_blade_pos", design_blade_pos_);
  this->declare_parameter<int>("max_des_blade_pos", 90);
  this->get_parameter("max_des_blade_pos", max_des_blade_pos_);
}

void AutoGraderNode::timerCallback() {

    // Continuously publish blade position if in autograder mode
    if (curr_mux_mode_ == cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    // Pick some safe wheel and steer default commands to have full command message, cmd_mux node should ignore these commands when in autograder mode
    cmd_msg_.wheel_velocity = 0.0; // [-100.0, 100.0]
    cmd_msg_.steer_position = 0.0; // [-100.0, 100.0]
        
    // Clamp blade position to known physical limits
    cmd_msg_.tool_position = std::max(0.0, std::min(cmd_msg_.tool_position, 100.0)); // [0.0, 100.0]

    // Publish the message
    cmd_msg_.header.stamp = this->get_clock()->now();
    cmd_pub_->publish(cmd_msg_);
  }
}

void AutoGraderNode::slipCallback(const cg_msgs::msg::Slip::SharedPtr msg) {

  // Continuously update blade position if in autograder mode
  if (curr_mux_mode_ == cg_msgs::msg::MuxMode::AUTOGRADER)
  {
    // Bang-bang control: only adjust the blade postion if outside the deadband, if deadband thresholds are not met then do not change blade position
    if (msg->slip > (slip_thresh_ + half_deadband_)) {
      // Raise target blade position to maximum desired height
      cmd_msg_.tool_position = max_des_blade_pos_;
    } else if (msg->slip < (slip_thresh_ - half_deadband_)) {
      // Lower target blade position to design plane
      cmd_msg_.tool_position = design_blade_pos_;
    }
  }
}

void AutoGraderNode::modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg)
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


}  // namespace teleop
}  // namespace cg
