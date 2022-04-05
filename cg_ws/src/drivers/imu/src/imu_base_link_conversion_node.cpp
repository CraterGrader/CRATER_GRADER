#include <tf2/transform_datatypes.h>
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
  std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  std::string target_frame = "base_link";
  std::string source_frame = "imu_link";
  while (rclcpp::ok()) {
    try {
      base_link_imu_link_tf_ = tf_buffer->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero
      );
      break;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(
        this->get_logger(),  "Could not lookupTransform with target %s, source %s: %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void ImuBaseLinkConversionNode::imuLinkImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
  imu_link_msg_ = *msg;
}

void ImuBaseLinkConversionNode::timerCallback() {
  // tf2::Quaternion q(
  //   imu_link_msg_.orientation.x,
  //   imu_link_msg_.orientation.y,
  //   imu_link_msg_.orientation.z,
  //   imu_link_msg_.orientation.w
  // );
  // double roll, pitch, yaw;
  // tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  // RCLCPP_INFO(this->get_logger(), "%f, %f, %f", roll, pitch, yaw);
}

}  // namespace imu
}  // namespace cg
