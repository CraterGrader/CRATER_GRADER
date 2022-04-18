#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/mux_mode.hpp>
#include <cg_msgs/msg/actuator_command.hpp>
// Diagnostics
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

namespace cg {
namespace cmdmux {

class CmdMuxNode : public rclcpp::Node {

public:
  CmdMuxNode();

private: 
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::ActuatorCommand>::SharedPtr cmd_pub_;

  rclcpp::Subscription<cg_msgs::msg::MuxMode>::SharedPtr mode_sub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr teleop_sub_;
  rclcpp::Subscription<cg_msgs::msg::ActuatorCommand>::SharedPtr autonomy_sub_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish

  /* Message data */
  cg_msgs::msg::ActuatorCommand cmd_msg_;

  /* Callbacks */
  void modeCallback(const cg_msgs::msg::MuxMode::SharedPtr msg);
  void teleopCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  void autonomyCallback(const cg_msgs::msg::ActuatorCommand::SharedPtr msg);
  void timerCallback(); // For looping publish in idle mode

  /* Variables */
  uint8_t curr_mux_mode_;

  /* Diagnostics */
  diagnostic_updater::Updater diagnostic_updater_;
  void populateDiagnosticsStatus(diagnostic_updater::DiagnosticStatusWrapper &stat); // Function for updating status information

  // Log topic frequency for /actuator_cmd
  std::unique_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> actuator_cmd_freq_;
  double freq_min_act_cmd_;
  double freq_max_act_cmd_;
  double freq_tol_act_cmd_;
  int freq_window_act_cmd_;

}; // class CmdMuxNode

} // namespace cmdmux
} // namespace cg
