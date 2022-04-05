#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer_interface.h>
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
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  std::shared_ptr<tf2_ros::TransformListener> tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::string target_frame = "base_link";
  std::string source_frame = "imu_link";
  while (rclcpp::ok()) {
    try {
      base_link_imu_link_tf_ = tf_buffer_->lookupTransform(
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
  try {
    sensor_msgs::msg::Imu base_link_msg;
    doTransform(imu_link_msg_, base_link_msg, base_link_imu_link_tf_);
    base_link_imu_pub_->publish(base_link_msg);
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(
      this->get_logger(), "Could not transform Imu msg with error: %s", ex.what()
    );
  }
}

void ImuBaseLinkConversionNode::doTransform(
    const sensor_msgs::msg::Imu &imu_in,
    sensor_msgs::msg::Imu &imu_out,
    const geometry_msgs::msg::TransformStamped &t_in
) const {
  imu_out.header = t_in.header;
}

void ImuBaseLinkConversionNode::transformCovariance(
    const std::vector<double> &in,
    std::vector<double> &out,
    const Eigen::Quaternion<double> &r
) const {
  // TODO
}


}  // namespace imu
}  // namespace cg
