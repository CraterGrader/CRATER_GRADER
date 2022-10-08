#pragma once 

#include <rclcpp/rclcpp.hpp>

namespace cg {
namespace planning {

class BehaviorExecutive : public rclcpp::Node {

public:
  BehaviorExecutive();

private: 
  /* Publishers and Subscribers */

  rclcpp::TimerBase::SharedPtr timer_; // For looping publish

  /* Message data */


  /* Callbacks */
  void timerCallback(); // For looping publish

  /* Variables */

}; // class BehaviorExecutive

} // namespace planning
} // namespace cg
