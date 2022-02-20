#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "imu/vn_imu_node.hpp"

namespace cg {
namespace imu {

VnImuNode::VnImuNode() : Node("vn_imu_node") {
  // Initialize IMU sensor
  std::string device_name;
  this->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  this->get_parameter("device_name", device_name);
  int baud_rate;
  this->declare_parameter<int>("baud_rate", 115200);
  this->get_parameter("baud_rate", baud_rate);
  int freq;
  this->declare_parameter<int>("pub_freq", 20);
  this->get_parameter("pub_freq", freq);
  try {
    vs_.connect(device_name, baud_rate);
  } catch (const vn::not_found & e) {
    RCLCPP_FATAL(this->get_logger(), "VectorNav IMU not found at %s", device_name.c_str());
    throw std::runtime_error("VectorNav IMU not found");
  }
  RCLCPP_INFO(this->get_logger(), "Connected to VectorNav IMU at %s with baud rate %d", device_name.c_str(), baud_rate);

  // Initialize publishers and subscribers
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu", 1
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/freq),
    std::bind(&VnImuNode::timerCallback, this)
  );
}

void VnImuNode::timerCallback() {
  sensor_msgs::msg::Imu msg;
  // Read IMU data
  auto vs_data_register = vs_.readYawPitchRollMagneticAccelerationAndAngularRates();

  // Set orientation
  tf2::Quaternion q;
  q.setRPY(
    M_PI/180.0*vs_data_register.yawPitchRoll[2],
    M_PI/180.0*vs_data_register.yawPitchRoll[1],
    0.0  // Intentionally ignore yaw assuming no magnetometer data on the moon, and set to zero
  );
  q.normalize();
  msg.orientation = tf2::toMsg(q);

  // Set linear acceleration
  msg.linear_acceleration.x = vs_data_register.accel[0];
  msg.linear_acceleration.y = vs_data_register.accel[1];
  msg.linear_acceleration.z = vs_data_register.accel[2];

  // Set angular velocity
  msg.angular_velocity.x = vs_data_register.gyro[0];
  msg.angular_velocity.y = vs_data_register.gyro[1];
  msg.angular_velocity.z = vs_data_register.gyro[2];

  // TODO the IMU message also has fields for covariance matrix values, ideally we should be setting these fields as well
  // The IMU message defaults to "zero" covariance matrices which means "unknown covariance"
  // If we want to use open-source ROS packages for localization, we may need to estimate the covariance
  // and set the values accordingly
  imu_pub_->publish(msg);
}

}  // namespace imu
}  // namespace cg
