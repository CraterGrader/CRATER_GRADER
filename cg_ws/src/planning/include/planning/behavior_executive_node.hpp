#ifndef PLANNING__BEHAVIOR_EXECUTIVE_HPP
#define PLANNING__BEHAVIOR_EXECUTIVE_HPP

#include <rclcpp/rclcpp.hpp>
#include <mapping/map.hpp>
#include <cg_msgs/srv/site_map.hpp> // Service for receiving SiteMap height data

namespace cg {
namespace planning {

class BehaviorExecutive : public rclcpp::Node {

public:
  BehaviorExecutive();

private: 
  /* Publishers and Subscribers */

  /* Services */
  // Create callback groups for service call in timer: https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr timer_; // For controlled looping map updates
  rclcpp::Client<cg_msgs::srv::SiteMap>::SharedPtr site_map_client_;
  bool updateMapFromService(bool verbose);

  long int timer_callback_ms_ = 1000;
  long int service_response_timeout_sec_ = 2;

  /* Message data */

  /* Callbacks */
  void timerCallback(); // For looping publish

  /* Variables */
  cg::mapping::Map<float> height_map_;

}; // class BehaviorExecutive

} // namespace planning
} // namespace cg

#endif // PLANNING__BEHAVIOR_EXECUTIVE_HPP
