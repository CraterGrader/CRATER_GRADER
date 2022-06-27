#ifndef ARDUINO__SERIAL_INTERFACE_NODE_HPP
#define ARDUINO__SERIAL_INTERFACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>
// Diagnostics
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace cg {
namespace arduino {

class SerialInterfaceNode : public rclcpp::Node {

public:
  SerialInterfaceNode();

private:
  /* Publishers and Subscribers */
  // Sending commands to arduino
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr cmd_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Reading commands from arduino
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ard_sub_;
  rclcpp::Publisher<cg_msgs::msg::EncoderTelemetry>::SharedPtr enc_pub_;

  // declare parameters
  int QP_TO_BYTE_STEER_SCALE_;
  int QP_TO_BYTE_STEER_OFFSET_;
  int QPPS_TO_BYTE_DRIVE_SCALE_;
  int QPPS_TO_BYTE_DRIVE_OFFSET_;
  int QP_TO_BYTE_TOOL_SCALE_;
  int QP_TO_BYTE_TOOL_OFFSET_;
  int QP_TO_BYTE_DELTA_POS_SCALE_;
  int QP_TO_BYTE_DELTA_POS_OFFSET_;
  
  /* Message data */
  cg_msgs::msg::ActuatorCommand actuator_cmd_; // Actuator command to send to arduino
  std_msgs::msg::Int64 ard_feedback_;
  cg_msgs::msg::EncoderTelemetry enc_telemetry_;

  /* Diagnostics */
  diagnostic_updater::Updater diagnostic_updater_;
  void populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat); // Function for updating status information

  // Log topic frequency for /arduino_cmd
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> arduino_cmd_freq_;
  double freq_min_ard_cmd_;
  double freq_max_ard_cmd_;
  double freq_tol_ard_cmd_;
  int freq_window_ard_cmd_;

  // Log topic frequency for /arduino_feedback
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> arduino_feedback_freq_;
  double freq_min_ard_fdbk_;
  double freq_max_ard_fdbk_;
  double freq_tol_ard_fdbk_;
  int freq_window_ard_fdbk_;

  // Log topic frequency for /encoder/telemetry
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> encoder_telemetry_freq_;
  double freq_min_enc_telem_;
  double freq_max_enc_telem_;
  double freq_tol_enc_telem_;
  int freq_window_enc_telem_;

  /* Callbacks */
  void timerCallback();
  // Callback for actuator command
  void cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  // Callback for arduino feedback
  void ardFbCallback(const std_msgs::msg::Int64::SharedPtr msg);

  /* Helper functions */
  /**
   * @brief Converts a byte from [0,255] to the original qpps integer value.
   * 
   * @param val The byte value to convert.
   * @param scale The scaling used for original conversion to a byte.
   * @param zero_offset The offset used for original conversion to a byte.
   * @return long The converted original value.
   */
  long byte_to_qpps(const int &val, const int &scale, const int &zero_offset);
};

}  // namespace arduino
}  // namespace cg

#endif  // ARDUINO__SERIAL_INTERFACE_NODE_HPP
