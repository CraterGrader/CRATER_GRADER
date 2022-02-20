#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "imu/imu_viz_node.hpp"

namespace cg {
namespace imu {

ImuVizNode::ImuVizNode() : Node("imu_viz_node") {
  // Initialize publishers and subscribers
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 1, std::bind(&ImuVizNode::imuCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ImuVizNode::timerCallback, this)
  );
}

void ImuVizNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  tf2::fromMsg(msg->orientation, orientation_);
}

void ImuVizNode::timerCallback() {
  // TODO
}

}  // namespace imu
}  // namespace cg
