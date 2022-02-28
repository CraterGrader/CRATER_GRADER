#include "uwb_beacon_rtls/uwb_beacon_rtls_node.hpp"

namespace cg {
namespace uwb_beacon_rtls {

UWBBeaconRTLSNode::UWBBeaconRTLSNode() : Node("uwb_beacon_rtls_node") {
  // Initialize publishers and subscribers
  rtls_pub_ = this->create_publisher<cg_msgs::msg::BeaconMultiTag>(
    "/uwb_beacon_rtls", 1
  );

  //Update the tag locations every 100 milliseconds
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&UWBBeaconRTLSNode::timerCallback, this)
  );

  // Load parameters


}

void UWBBeaconRTLSNode::timerCallback() {
  auto multi_tag_msg = cg_msgs::msg::BeaconMultiTag();

  rclcpp::Time timestamp = this->get_clock()->now();
  multi_tag_msg.header.stamp = timestamp;
  rtls_pub_->publish(multi_tag_msg);
}

}  // namespace uwb_beacon_rtls
}  // namespace cg
