#ifndef IMU__IMU_VIZ_NODE_HPP
#define IMU__IMU_VIZ_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "vn/sensors.h"

namespace cg {
namespace imu {

class ImuVizNode : public rclcpp::Node {

public:
  ImuVizNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /* Callbacks */
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timerCallback();

  tf2::Quaternion orientation_;

};


}  // namespace imu
}  // namespace cg

#endif  // IMU__IMU_VIZ_NODE_HPP
