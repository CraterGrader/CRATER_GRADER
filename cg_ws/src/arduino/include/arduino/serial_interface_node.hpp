#ifndef ARDUINO__SERIAL_INTERFACE_NODE_HPP
#define ARDUINO__SERIAL_INTERFACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
#include <cg_msgs/msg/encoder_telemetry.hpp>

namespace cg {
namespace arduino {

class SerialInterfaceNode : public rclcpp::Node {

public:
  SerialInterfaceNode();

private:
  /* Publishers and Subscribers */
  // Sending commands to arduino
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr cmd_pub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Reading commands from arduino
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr ard_sub_;
  rclcpp::Publisher<cg_msgs::msg::EncoderTelemetry>::SharedPtr ard_pub_;
  #define QP_TO_BYTE_STEER_SCALE (22)
  #define QP_TO_BYTE_STEER_OFFSET (127)
  #define QP_TO_BYTE_DRIVE_SCALE (25) // DOUBLE CHECK VALUE FROM LIMITING
  #define QP_TO_BYTE_DRIVE_OFFSET (127) 
  #define QP_TO_BYTE_TOOL_SCALE (22)
  #define QP_TO_BYTE_TOOL_OFFSET (588) // DOUBLE CHECK VALUE FROM LIMITING
  
  /* Message data */
  cg_msgs::msg::ActuatorCommand actuator_cmd_;
  std_msgs::msg::Int64 ard_feedback_;
  cg_msgs::msg::EncoderTelemetry enc_telemetry_;

  /* Callbacks */
  void timerCallback();
  // Callback for actuator command
  void cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  // Callback for arduino feedback
  void ardCallback(const std_msgs::msg::Int64::SharedPtr msg);

  /* Helper functions */
  /**
   * @brief Converts a byte from [0,255] to the original qpps integer value.
   * 
   * @param val The byte value to convert.
   * @param scale The scaling used for original conversion to a byte.
   * @param zero_offset The offset used for original conversion to a byte.
   * @return long The converted original value.
   */
  long byte_to_qpps(const int val, const int scale, const int zero_offset);
};

}  // namespace arduino
}  // namespace cg

#endif  // ARDUINO__SERIAL_INTERFACE_NODE_HPP
