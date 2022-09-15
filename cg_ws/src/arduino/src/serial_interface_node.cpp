#include "arduino/serial_interface_node.hpp"

/*
 * Team CraterGrader
 * CMU MRSD Program, Course 16-681
 * Team Members: Alex Pletta, Ryan Lee, Russell Wong, Ben Younes, John Harrington
 * Rev0: February 5, 2022
 * Code description: Facilitate low-level motion control commands from Xavier ROS2 stack to micro-ROS node on Arduino Due
 */

namespace cg {
namespace arduino {

  SerialInterfaceNode::SerialInterfaceNode() : Node("serial_interface_node"), diagnostic_updater_(this)
  {
    // Initialize publishers and subscribers
    cmd_pub_ = this->create_publisher<std_msgs::msg::Int64>(
        "/arduino_cmd", 1);
    cmd_sub_ = this->create_subscription<cg_msgs::msg::ActuatorCommand>(
        "/actuator_cmd", 1, std::bind(&SerialInterfaceNode::cmdCallback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&SerialInterfaceNode::timerCallback, this));

    // Arduino feedback
    ard_sub_ = this->create_subscription<std_msgs::msg::Int64>(
        "/arduino_feedback", 1, std::bind(&SerialInterfaceNode::ardFbCallback, this, std::placeholders::_1));
    enc_pub_ = this->create_publisher<cg_msgs::msg::EncoderTelemetry>(
        "/encoder_telemetry", 1);

    diagnostic_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 1);

    /* Parameters */
    // Diagnostics
    this->declare_parameter<double>("freq_min_ard_cmd", 1.0);
    this->get_parameter("freq_min_ard_cmd", freq_min_ard_cmd_);
    this->declare_parameter<double>("freq_max_ard_cmd", 100.0);
    this->get_parameter("freq_max_ard_cmd", freq_max_ard_cmd_);
    this->declare_parameter<double>("freq_tol_ard_cmd", 0.1);
    this->get_parameter("freq_tol_ard_cmd", freq_tol_ard_cmd_);
    this->declare_parameter<int>("freq_window_ard_cmd", 5);
    this->get_parameter("freq_window_ard_cmd", freq_window_ard_cmd_);

    this->declare_parameter<double>("freq_min_ard_fdbk", 1.0);
    this->get_parameter("freq_min_ard_fdbk", freq_min_ard_fdbk_);
    this->declare_parameter<double>("freq_max_ard_fdbk", 100.0);
    this->get_parameter("freq_max_ard_fdbk", freq_max_ard_fdbk_);
    this->declare_parameter<double>("freq_tol_ard_fdbk", 0.1);
    this->get_parameter("freq_tol_ard_fdbk", freq_tol_ard_fdbk_);
    this->declare_parameter<int>("freq_window_ard_fdbk", 5);
    this->get_parameter("freq_window_ard_fdbk", freq_window_ard_fdbk_);

    this->declare_parameter<double>("freq_min_enc_telem", 1.0);
    this->get_parameter("freq_min_enc_telem", freq_min_enc_telem_);
    this->declare_parameter<double>("freq_max_enc_telem", 100.0);
    this->get_parameter("freq_max_enc_telem", freq_max_enc_telem_);
    this->declare_parameter<double>("freq_tol_enc_telem", 0.1);
    this->get_parameter("freq_tol_enc_telem", freq_tol_enc_telem_);
    this->declare_parameter<int>("freq_window_enc_telem", 5);
    this->get_parameter("freq_window_enc_telem", freq_window_enc_telem_);

    // Motor controller scaling
    this->declare_parameter<int>("QP_TO_BYTE_STEER_SCALE", 22);
    this->get_parameter("QP_TO_BYTE_STEER_SCALE", QP_TO_BYTE_STEER_SCALE_);
    this->declare_parameter<int>("QP_TO_BYTE_STEER_OFFSET", 127);
    this->get_parameter("QP_TO_BYTE_STEER_OFFSET", QP_TO_BYTE_STEER_OFFSET_);
    this->declare_parameter<int>("QPPS_TO_BYTE_DRIVE_SCALE", 25);
    this->get_parameter("QPPS_TO_BYTE_DRIVE_SCALE", QPPS_TO_BYTE_DRIVE_SCALE_);
    this->declare_parameter<int>("QPPS_TO_BYTE_DRIVE_OFFSET", 127);
    this->get_parameter("QPPS_TO_BYTE_DRIVE_OFFSET", QPPS_TO_BYTE_DRIVE_OFFSET_);
    this->declare_parameter<int>("QP_TO_BYTE_TOOL_SCALE", 129);
    this->get_parameter("QP_TO_BYTE_TOOL_SCALE", QP_TO_BYTE_TOOL_SCALE_);
    this->declare_parameter<int>("QP_TO_BYTE_TOOL_OFFSET", 0);
    this->get_parameter("QP_TO_BYTE_TOOL_OFFSET", QP_TO_BYTE_TOOL_OFFSET_);
    this->declare_parameter<int>("QP_TO_BYTE_DELTA_POS_SCALE", 10);
    this->get_parameter("QP_TO_BYTE_DELTA_POS_SCALE", QP_TO_BYTE_DELTA_POS_SCALE_);
    this->declare_parameter<int>("QP_TO_BYTE_DELTA_POS_OFFSET", 127);
    this->get_parameter("QP_TO_BYTE_DELTA_POS_OFFSET", QP_TO_BYTE_DELTA_POS_OFFSET_);    

    // Initialize Diagnostics
    diagnostic_updater_.add("serial_interface_node", this, &SerialInterfaceNode::populateDiagnosticsStatus);
    arduino_cmd_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("/arduino_cmd", diagnostic_updater_, diagnostic_updater::FrequencyStatusParam(&freq_min_ard_cmd_, &freq_max_ard_cmd_, freq_tol_ard_cmd_, freq_window_ard_cmd_)));
    arduino_feedback_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("/arduino_feedback", diagnostic_updater_, diagnostic_updater::FrequencyStatusParam(&freq_min_ard_fdbk_, &freq_max_ard_fdbk_, freq_tol_ard_fdbk_, freq_window_ard_fdbk_)));
    encoder_telemetry_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("/encoder_telemetry", diagnostic_updater_, diagnostic_updater::FrequencyStatusParam(&freq_min_enc_telem_, &freq_max_enc_telem_, freq_tol_enc_telem_, freq_window_enc_telem_)));
  }

  void SerialInterfaceNode::populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }

  void SerialInterfaceNode::timerCallback()
  {
    auto cmd_msg = std_msgs::msg::Int64();
    int64_t cmd_data = 0;
    // Scale wheel velocity in range [-100.0, 100.0] to [0, 255]
    uint8_t data = static_cast<uint8_t>((actuator_cmd_.wheel_velocity + 100) / 200.0 * 255);
    cmd_data |= data;
    // Scale steer position in range [-100.0, 100.0] to [0, 255]
    data = static_cast<uint8_t>((actuator_cmd_.steer_position + 100) / 200.0 * 255);
    cmd_data |= (data << 8);
    // Scale tool position in range [0.0, 100.0] to [0, 255]
    data = static_cast<uint8_t>((actuator_cmd_.tool_position) / 100.0 * 255);
    cmd_data |= (data << 16);

    cmd_msg.data = cmd_data;
    cmd_pub_->publish(cmd_msg);
    arduino_cmd_freq_->tick(); // Log frequency for diagnostics
}

void SerialInterfaceNode::cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg) {
  actuator_cmd_ = *msg;
}

void SerialInterfaceNode::ardFbCallback(const std_msgs::msg::Int64::SharedPtr msg) {
  // Read in the message
  ard_feedback_ = *msg;
  arduino_feedback_freq_->tick(); // Log frequency for diagnostics

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
  encoder_telemetry_freq_->tick(); // Log frequency for diagnostics
}

long SerialInterfaceNode::byte_to_qpps(const int &val, const int &scale, const int &zero_offset)
{
  return (val - zero_offset) * scale;
}

} // namespace arduino
} // namespace cg
