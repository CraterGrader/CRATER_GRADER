#ifndef BEARING__BEARING_NODE_HPP
#define BEARING__BEARING_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
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
  // rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bearing_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

  /* Callbacks */
  // Timer callback
  void timerCallback();
  void poseUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  // Stores moving average
  std::vector<double> rolling_sin;
  std::vector<double> rolling_cos;
  std::vector<double> rolling_bearing;
  std::vector<double> angle_90_inc_x = {std::cos(0), std::cos(M_PI/2), std::cos(M_PI), std::cos(3*M_PI/2)};
  std::vector<double> angle_90_inc_y = {std::sin(0), std::sin(M_PI/2), std::sin(M_PI), std::sin(3*M_PI/2)};
  std::vector<double> tag_x = {3.49563, 6.939225, 3.44706, 0.02744};
  std::vector<double> tag_y = {7.279027, 3.562075, -0.00215, 3.6699};

  // Stores current robot x and y
  double robot_x;
  double robot_y;
  double robot_pitch_rad;

  int pub_freq;
  double tf_discard_time;
  int rolling_avg_buffer;
  double bearing_covariance;

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> full_tf_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

}  // namespace bearing
}  // namespace cg

#endif  // BEARING_BEARING_NODE_HPP
