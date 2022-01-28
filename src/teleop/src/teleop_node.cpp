#include "teleop/teleop_node.hpp"

namespace cg {
namespace teleop {

TeleopNode::TeleopNode() : Node("teleop_node") {
  // Initialize publishers and subscribers
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&TeleopNode::joyCallback, this, std::placeholders::_1)
  );
}

void TeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joy_axis_drive_state_ = msg->axes[joy_axis_drive_i_];
  joy_axis_steer_state_ = msg->axes[joy_axis_steer_i_];
}

}  // namespace teleop
}  // namespace cg
