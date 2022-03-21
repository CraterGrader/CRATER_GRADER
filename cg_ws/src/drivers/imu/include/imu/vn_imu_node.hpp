#ifndef IMU__VN_IMU_NODE_HPP
#define IMU__VN_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
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

  void loadParamToVector3(const std::string & param_name,
      geometry_msgs::msg::Vector3 & v);

  bool publish_viz_;
  geometry_msgs::msg::Vector3 linear_acc_zero_offsets_;
  geometry_msgs::msg::Vector3 angular_vel_zero_offsets_;
  geometry_msgs::msg::Vector3 orientation_zero_offsets_;
  geometry_msgs::msg::Vector3 linear_acc_variances_;
  geometry_msgs::msg::Vector3 angular_vel_variances_;
  geometry_msgs::msg::Vector3 orientation_variances_;
};


}  // namespace imu
}  // namespace cg

#endif  // IMU__VN_IMU_NODE_HPP
