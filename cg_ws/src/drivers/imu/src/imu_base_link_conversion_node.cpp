#include "imu/imu_base_link_conversion_node.hpp"

namespace cg {
namespace imu {

ImuBaseLinkConversionNode::ImuBaseLinkConversionNode() : Node("imu_base_link_conversion_node") {
  // Initialize publishers and subscribers
  base_link_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data/base_link", 1
  );
  imu_link_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data/imu_link", rclcpp::SensorDataQoS(), std::bind(&ImuBaseLinkConversionNode::imuLinkImuCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&ImuBaseLinkConversionNode::timerCallback, this)
  );
}

void ImuBaseLinkConversionNode::imuLinkImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  imu_link_msg_ = *msg;
}

void ImuBaseLinkConversionNode::timerCallback() {
  // TODO
}

}  // namespace imu
}  // namespace cg
