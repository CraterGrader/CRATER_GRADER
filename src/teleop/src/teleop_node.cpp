#include "teleop/teleop_node.hpp"

namespace cg {
namespace teleop {

TeleopNode::TeleopNode() : Node("teleop_node") {
  // Initialize publishers and subscribers
  cmd_pub_ = this->create_publisher<cg_msgs::msg::ActuatorCommand>(
    "/actuator_cmd", 1
  );
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&TeleopNode::joyCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TeleopNode::timerCallback, this)
  );
}

void TeleopNode::timerCallback() {
  auto cmd_msg = cg_msgs::msg::ActuatorCommand();
  cmd_msg.wheel_velocity = 0;
  cmd_msg.steer_velocity = 1;
  cmd_pub_->publish(cmd_msg);
}

void TeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joy_axis_drive_state_ = msg->axes[joy_axis_drive_i_];
  joy_axis_steer_state_ = msg->axes[joy_axis_steer_i_];
}

}  // namespace teleop
}  // namespace cg
