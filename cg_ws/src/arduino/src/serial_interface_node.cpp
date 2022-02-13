#include "arduino/serial_interface_node.hpp"

namespace cg {
namespace arduino {

SerialInterfaceNode::SerialInterfaceNode() : Node("serial_interface_node") {
  // Initialize publishers and subscribers
  cmd_pub_ = this->create_publisher<std_msgs::msg::Int64>(
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
  auto cmd_msg = std_msgs::msg::Int64();
  int64_t cmd_data = 0;
  // Scale wheel velocity in range [-100.0, 100.0] to [0, 255]
  uint8_t data = static_cast<uint8_t>((actuator_cmd_.wheel_velocity + 100) / 200.0 * 255);
  cmd_data |= data;
  // Scale steer position in range [-100.0, 100.0] to [0, 255]
  data =  static_cast<uint8_t>((actuator_cmd_.steer_position + 100) / 200.0 * 255);
  cmd_data |= (data << 8);

  // Scale tool position in range [-100.0, 100.0] to [0, 255]
  data =  static_cast<uint8_t>((actuator_cmd_.tool_position + 100) / 200.0 * 255);
  cmd_data |= (data << 16);

  cmd_msg.data = cmd_data;
  cmd_pub_->publish(cmd_msg);
}

void SerialInterfaceNode::cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {
  actuator_cmd_ = *msg;
}

}  // namespace arduino
}  // namespace cg
