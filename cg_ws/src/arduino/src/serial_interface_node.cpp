#include "arduino/serial_interface_node.hpp"

namespace cg {
namespace arduino {

SerialInterfaceNode::SerialInterfaceNode() : Node("serial_interface_node") {
  // Initialize publishers and subscribers
  reset_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      "/reset_arduino", rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local()); // Create with reliable reliability and transient local durability, see https://community.rti.com/static/documentation/connext-dds/6.0.1/doc/manuals/connext_dds/getting_started/cpp11/intro_qos.html#section-gsg-qos-durability
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
    "/arduino_feedback", 1, std::bind(&SerialInterfaceNode::ardFbCallback, this, std::placeholders::_1)
  );
  enc_pub_ = this->create_publisher<cg_msgs::msg::EncoderTelemetry>(
    "/encoder_telemetry", 1
  );

  this->declare_parameter<int>("QP_TO_BYTE_STEER_SCALE", 22);
  this->get_parameter("QP_TO_BYTE_STEER_SCALE", QP_TO_BYTE_STEER_SCALE_);
  this->declare_parameter<int>("QP_TO_BYTE_STEER_OFFSET", 127);
  this->get_parameter("QP_TO_BYTE_STEER_OFFSET", QP_TO_BYTE_STEER_OFFSET_);
  this->declare_parameter<int>("QPPS_TO_BYTE_DRIVE_SCALE", 25);
  this->get_parameter("QPPS_TO_BYTE_DRIVE_SCALE", QPPS_TO_BYTE_DRIVE_SCALE_);
  this->declare_parameter<int>("QPPS_TO_BYTE_DRIVE_OFFSET", 127);
  this->get_parameter("QPPS_TO_BYTE_DRIVE_OFFSET", QPPS_TO_BYTE_DRIVE_OFFSET_);
  this->declare_parameter<int>("QP_TO_BYTE_TOOL_SCALE", 22);
  this->get_parameter("QP_TO_BYTE_TOOL_SCALE", QP_TO_BYTE_TOOL_SCALE_);
  this->declare_parameter<int>("QP_TO_BYTE_TOOL_OFFSET", 588);
  this->get_parameter("QP_TO_BYTE_TOOL_OFFSET", QP_TO_BYTE_TOOL_OFFSET_);
  this->declare_parameter<int>("QP_TO_BYTE_DELTA_POS_SCALE", 10);
  this->get_parameter("QP_TO_BYTE_DELTA_POS_SCALE", QP_TO_BYTE_DELTA_POS_SCALE_);
  this->declare_parameter<int>("QP_TO_BYTE_DELTA_POS_OFFSET", 127);
  this->get_parameter("QP_TO_BYTE_DELTA_POS_OFFSET", QP_TO_BYTE_DELTA_POS_OFFSET_);

  // Reset the arduino on start up
  reset_arduino_val_.data = 1;
  reset_pub_->publish(reset_arduino_val_);
  reset_arduino_val_.data = 0; // Immediately set reset to zero so we can't accidentally reset again
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

void SerialInterfaceNode::ardFbCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  // Read in the message
  ard_feedback_ = *msg;

  // Steer position front
  int steer_pos_front_byte = ard_feedback_.data & 0xFF; // First byte
  enc_telemetry_.steer_pos_front = byte_to_qpps(steer_pos_front_byte, QP_TO_BYTE_STEER_SCALE_, QP_TO_BYTE_STEER_OFFSET_);

  // Steer position rear
  int steer_pos_rear_byte = (ard_feedback_.data >> 8) & 0xFF; // Second byte
  enc_telemetry_.steer_pos_rear = byte_to_qpps(steer_pos_rear_byte, QP_TO_BYTE_STEER_SCALE_, QP_TO_BYTE_STEER_OFFSET_);

  // Tool position
  int tool_pos_byte = (ard_feedback_.data >> 16) & 0xFF; // Third byte
  enc_telemetry_.tool_pos = byte_to_qpps(tool_pos_byte, QP_TO_BYTE_TOOL_SCALE_, QP_TO_BYTE_TOOL_OFFSET_);
  
  // Drive velocity front
  int drive_vel_front_byte = (ard_feedback_.data >> 24) & 0xFF; // Fourth byte
  enc_telemetry_.drive_vel_front = byte_to_qpps(drive_vel_front_byte, QPPS_TO_BYTE_DRIVE_SCALE_, QPPS_TO_BYTE_DRIVE_OFFSET_);

  // Drive velocity rear
  int drive_vel_rear_byte = (ard_feedback_.data >> 32) & 0xFF; // Fifth byte
  enc_telemetry_.drive_vel_rear = byte_to_qpps(drive_vel_rear_byte, QPPS_TO_BYTE_DRIVE_SCALE_, QPPS_TO_BYTE_DRIVE_OFFSET_);

  // Drive delta position front
  int drive_delta_pos_front = (ard_feedback_.data >> 40) & 0xFF;
  enc_telemetry_.drive_delta_front = byte_to_qpps(drive_delta_pos_front, QP_TO_BYTE_DELTA_POS_SCALE_, QP_TO_BYTE_DELTA_POS_OFFSET_); // Sixth byte

  // Drive delta position rear
  int drive_delta_pos_rear = (ard_feedback_.data >> 48) & 0xFF;
  enc_telemetry_.drive_delta_rear = byte_to_qpps(drive_delta_pos_rear, QP_TO_BYTE_DELTA_POS_SCALE_, QP_TO_BYTE_DELTA_POS_OFFSET_);  // Seventh byte

  // Terminal byte (limit switches, heartbeat, etc.)
  enc_telemetry_.term_byte = (ard_feedback_.data >> 56) & 0xFF; // Eighth byte

  // Publish the message
  enc_telemetry_.header.stamp = this->get_clock()->now();
  enc_pub_->publish(enc_telemetry_);
}

long SerialInterfaceNode::byte_to_qpps(const int &val, const int &scale, const int &zero_offset)
{
  return (val - zero_offset) * scale;
}

} // namespace arduino
} // namespace cg
