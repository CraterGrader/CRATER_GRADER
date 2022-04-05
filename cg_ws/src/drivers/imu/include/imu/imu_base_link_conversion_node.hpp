#ifndef IMU__IMU_BASE_LINK_CONVERSION_NODE_HPP
#define IMU__IMU_BASE_LINK_CONVERSION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace cg {
namespace imu {

class ImuBaseLinkConversionNode : public rclcpp::Node {

public:
  ImuBaseLinkConversionNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr base_link_imu_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_link_imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  void imuLinkImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void timerCallback();

  sensor_msgs::msg::Imu imu_link_msg_;
};


}  // namespace imu
}  // namespace cg

#endif  // IMU__IMU_BASE_LINK_CONVERSION_NODE_HPP
