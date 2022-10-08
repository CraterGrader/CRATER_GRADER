#ifndef PLANNING__BEHAVIOR_EXECUTIVE_HPP
#define PLANNING__BEHAVIOR_EXECUTIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mapping/map.hpp>

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
  cg::mapping::Map<float> height_map;

}; // class BehaviorExecutive

} // namespace planning
} // namespace cg

#endif // PLANNING__BEHAVIOR_EXECUTIVE_HPP
