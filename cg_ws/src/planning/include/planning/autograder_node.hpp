#ifndef TELEOP__TELEOP_NODE_HPP
#define TELEOP__TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>

namespace cg {
namespace planning {

class AutoGraderNode : public rclcpp::Node {

public:
  AutoGraderNode();

private:
  /* Publishers and Subscribers */
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  // Callback for joystick input
  void timerCallback();

};


}  // namespace planning
}  // namespace cg

#endif  // PLANNIN_PLANNING_NODE_HPP
