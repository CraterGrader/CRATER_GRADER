#include "arduino/serial_interface_node.hpp"

namespace cg {
namespace arduino {

SerialInterfaceNode::SerialInterfaceNode() : Node("serial_interface_node") {
  // Initialize publishers and subscribers
  wheel_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>(
    "/arduino_cmd_wheel_vel", 1
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
  auto cmd_msg = std_msgs::msg::Float32();
  // cmd_msg.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
  // cmd_msg.data.push_back(actuator_cmd_.wheel_velocity);
  // cmd_msg.data.push_back(actuator_cmd_.steer_velocity);
  // cmd_msg.layout.dim[0].size = cmd_msg.data.size();
  // cmd_msg.layout.dim[0].stride = 1;
  cmd_msg.data = actuator_cmd_.wheel_velocity;
  wheel_vel_pub_->publish(cmd_msg);
}

void SerialInterfaceNode::cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {
  actuator_cmd_ = *msg;
}

}  // namespace arduino
}  // namespace cg
