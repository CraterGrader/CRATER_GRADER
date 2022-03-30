#ifndef BEARING__BEARING_NODE_HPP
#define BEARING__BEARING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/transform_datatypes.h"
#include <chrono>
#include <vector>
#include <string>
#include <math.h>

namespace cg {
namespace bearing {

class BearingNode : public rclcpp::Node {

public:
  BearingNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr bearing_pub_;

  /* Callbacks */
  // Timer callback
  void timerCallback();

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> full_tf_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace bearing
}  // namespace cg

#endif  // BEARING_BEARING_NODE_HPP
