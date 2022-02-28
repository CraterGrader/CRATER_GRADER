#ifndef UWB_BEACON_RTLS__TELEOP_NODE_HPP
#define UWB_BEACON_RTLS__TELEOP_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <cg_msgs/msg/beacon_multi_tag.hpp>
#include <cg_msgs/msg/beacon_tag.hpp>

namespace cg {
namespace uwb_beacon_rtls {

class UWBBeaconRTLSNode : public rclcpp::Node {

public:
  UWBBeaconRTLSNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<cg_msgs::msg::BeaconMultiTag>::SharedPtr rtls_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  // Callback for joystick input
  void timerCallback();


  // Define MultiTag data

};


}  // namespace uwb_beacon_rtls
}  // namespace cg

#endif  // UWB_BEACON_RTLS__TELEOP_NODE_HPP
