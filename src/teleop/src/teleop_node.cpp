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
  RCLCPP_INFO(this->get_logger(), "Joy msg received");
}

}  // namespace teleop
}  // namespace cg
