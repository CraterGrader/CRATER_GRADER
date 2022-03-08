#ifndef IMU__VN_IMU_NODE_HPP
#define IMU__VN_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "vn/sensors.h"

namespace cg {
namespace imu {

class VnImuNode : public rclcpp::Node {

public:
  VnImuNode();

private:
  /* VectorNav */
  vn::sensors::VnSensor vs_;

  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> viz_tf_broadcaster_;

  /* Callbacks */
  void timerCallback();

  bool publish_viz_;
};


}  // namespace imu
}  // namespace cg

#endif  // IMU__VN_IMU_NODE_HPP
