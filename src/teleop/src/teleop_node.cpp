#include "teleop/teleop_node.hpp"

namespace cg {
namespace teleop {

TeleopNode::TeleopNode() : Node("teleop_node") {
  // Initialize publishers and subscribers
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&TeleopNode::joyCallback, this, std::placeholders::_1)
  );
}

void TeleopNode::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) const {
  joy_axis_fb_state_ = msg->axes[joy_axis_fb_i_];
  joy_axis_lr_state_ = msg->axes[joy_axis_lr_i_];
}

}  // namespace teleop
}  // namespace cg
