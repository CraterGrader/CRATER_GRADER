#ifndef ARDUINO__SERIAL_INTERFACE_NODE_HPP
#define ARDUINO__SERIAL_INTERFACE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int64.hpp>
#include <cg_msgs/msg/actuator_command.hpp>

namespace cg {
namespace arduino {

class SerialInterfaceNode : public rclcpp::Node {

public:
  SerialInterfaceNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr cmd_pub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Message data */
  cg_msgs::msg::ActuatorCommand actuator_cmd_;

  /* Callbacks */
  void timerCallback();
  // Callback for actuator command
  void cmdCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
};

}  // namespace arduino
}  // namespace cg

#endif  // ARDUINO__SERIAL_INTERFACE_NODE_HPP
