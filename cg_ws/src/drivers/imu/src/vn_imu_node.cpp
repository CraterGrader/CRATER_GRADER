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
    std::chrono::milliseconds(100),
    std::bind(&VnImuNode::timerCallback, this)
  );
}

void VnImuNode::timerCallback() {
  // TODO
}

}  // namespace imu
}  // namespace cg
