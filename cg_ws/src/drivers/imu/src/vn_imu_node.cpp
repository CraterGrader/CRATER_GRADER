#include "imu/vn_imu_node.hpp"

namespace cg {
namespace imu {

VnImuNode::VnImuNode() : Node("vn_imu_node") {
  // Initialize IMU sensor
  vs_.connect("/dev/ttyUSB0", 115200);  // TODO parametrize device name and baud rate

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
