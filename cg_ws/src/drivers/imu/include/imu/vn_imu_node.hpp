#ifndef IMU__VN_IMU_NODE_HPP
#define IMU__VN_IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace cg {
namespace imu {

class VnImuNode : public rclcpp::Node {

public:
  VnImuNode();

private:
  /* Publishers and Subscribers */
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /* Callbacks */
  // Callback for joystick input
  void timerCallback();

};


}  // namespace imu
}  // namespace cg

#endif  // IMU__VN_IMU_NODE_HPP
