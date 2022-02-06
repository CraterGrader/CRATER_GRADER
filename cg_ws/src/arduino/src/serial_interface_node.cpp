#include "arduino/serial_interface_node.hpp"

namespace cg {
namespace arduino {

SerialInterfaceNode::SerialInterfaceNode() : Node("serial_interface_node") {
  // Initialize publishers and subscribers
  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/arduino_cmd", 1
  );
  cmd_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
    "/actuator_cmd", 1, std::bind(&SerialInterfaceNode::cmdCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&SerialInterfaceNode::timerCallback, this)
  );
}

void SerialInterfaceNode::timerCallback() {
  // TODO
}

void SerialInterfaceNode::cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {
  // TODO
}

}  // namespace arduino
}  // namespace cg
