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

  // Arduino feedback
  ard_sub_ = this->create_subscription<std_msgs::msg::Int64>(
    "/arduino_debug", 1, std::bind(&SerialInterfaceNode::ardCallback, this, std::placeholders::_1)
  );
  ard_pub_ = this->create_publisher<cg_msgs::msg::ArduinoFeedback>(
    "/arduino_feedback", 1
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
  // Scale tool position in range [0.0, 100.0] to [0, 255]
  data =  static_cast<uint8_t>((actuator_cmd_.tool_position) / 100.0 * 255);
  cmd_data |= (data << 16);

  cmd_msg.data = cmd_data;
  cmd_pub_->publish(cmd_msg);
}

void SerialInterfaceNode::cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {
  actuator_cmd_ = *msg;
}

void SerialInterfaceNode::ardCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  // Read in the message
  ard_debug_ = *msg;

  // Steer position front
  int steer_pos_front_byte = ard_debug_.data & 0xFF; // First byte
  ard_feedback_.steer_pos_front = SerialInterfaceNode::byte_to_qpps(steer_pos_front_byte, QP_TO_BYTE_STEER_SCALE, QP_TO_BYTE_STEER_OFFSET);

  // Steer position rear
  int steer_pos_rear_byte = (ard_debug_.data >> 8) & 0xFF; // Second byte
  ard_feedback_.steer_pos_rear = SerialInterfaceNode::byte_to_qpps(steer_pos_rear_byte, QP_TO_BYTE_STEER_SCALE, QP_TO_BYTE_STEER_OFFSET);

  // Tool position
  int tool_pos_byte = (ard_debug_.data >> 16) & 0xFF; // Third byte
  ard_feedback_.tool_pos = SerialInterfaceNode::byte_to_qpps(tool_pos_byte, QP_TO_BYTE_TOOL_SCALE, QP_TO_BYTE_TOOL_OFFSET);
  
  // Drive velocity front
  int drive_vel_front_byte = (ard_debug_.data >> 24) & 0xFF; // Fourth byte
  ard_feedback_.drive_vel_front = SerialInterfaceNode::byte_to_qpps(drive_vel_front_byte, QP_TO_BYTE_DRIVE_SCALE, QP_TO_BYTE_DRIVE_OFFSET);

  // Drive velocity rear
  int drive_vel_rear_byte = (ard_debug_.data >> 32) & 0xFF; // Fifth byte
  ard_feedback_.drive_vel_rear = SerialInterfaceNode::byte_to_qpps(drive_vel_rear_byte, QP_TO_BYTE_DRIVE_SCALE, QP_TO_BYTE_DRIVE_OFFSET);

  // Drive delta position front
  ard_feedback_.drive_delta_front = (ard_debug_.data >> 40) & 0xFF; // Sixth byte

  // Drive delta position rear
  ard_feedback_.drive_delta_rear = (ard_debug_.data >> 48) & 0xFF; // Sixth byte

  // Terminal byte (limit switches, heartbeat, etc.)
  ard_feedback_.term_byte = (ard_debug_.data >> 56) & 0xFF; // Sixth byte

  // Publish the message
  ard_pub_->publish(ard_feedback_);
}

long SerialInterfaceNode::byte_to_qpps(const int val, const int scale, const int zero_offset)
{
  return (val - zero_offset) * scale;
}

}  // namespace arduino
}  // namespace cg
