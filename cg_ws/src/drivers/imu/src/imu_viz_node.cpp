#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void ImuVizNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // tf2::fromMsg(msg->orientation, orientation_);
  orientation_ = msg->orientation;
}

void ImuVizNode::timerCallback() {
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = this->get_clock()->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "imu_viz";
  transform_stamped.transform.rotation = orientation_;
  transform_stamped.transform.translation.x = 1;
  transform_stamped.transform.translation.y = 1;
  transform_stamped.transform.translation.z = 1;
  tf_broadcaster_->sendTransform(transform_stamped);
}

}  // namespace imu
}  // namespace cg
