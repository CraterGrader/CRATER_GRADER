#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
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
  this->declare_parameter<bool>("publish_viz", false);
  this->get_parameter("publish_viz", publish_viz_);

  // Load zero offsets and variances
  loadParamToVector3("zero_offset.linear_acc", linear_acc_zero_offsets_);
  loadParamToVector3("zero_offset.angular_vel", angular_vel_zero_offsets_);
  loadParamToVector3("zero_offset.orientation", orientation_zero_offsets_);
  loadParamToVector3("variance.linear_acc", linear_acc_variances_);
  loadParamToVector3("variance.angular_vel", angular_vel_variances_);
  loadParamToVector3("variance.orientation", orientation_variances_);

  try {
    vs_.connect(device_name, baud_rate);
  } catch (const vn::not_found & e) {
    RCLCPP_FATAL(this->get_logger(), "VectorNav IMU not found at %s", device_name.c_str());
    throw std::runtime_error("VectorNav IMU not found");
  }
  RCLCPP_INFO(this->get_logger(), "Connected to VectorNav IMU at %s with baud rate %d", device_name.c_str(), baud_rate);

  // Initialize publishers and subscribers
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
    "/imu/data", 1
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/freq),
    std::bind(&VnImuNode::timerCallback, this)
  );
  viz_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void VnImuNode::timerCallback() {
  sensor_msgs::msg::Imu msg;
  // Read IMU data
  auto vs_data_register = vs_.readYawPitchRollMagneticAccelerationAndAngularRates();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "imu_link";

  // Set orientation
  tf2::Quaternion q;
  q.setRPY(
    M_PI/180.0*vs_data_register.yawPitchRoll[2] - orientation_zero_offsets_.x,
    M_PI/180.0*vs_data_register.yawPitchRoll[1] - orientation_zero_offsets_.y,
    0.0  // Intentionally ignore yaw assuming no magnetometer data on the moon, and set to zero
  );
  q.normalize();
  msg.orientation = tf2::toMsg(q);
  msg.orientation_covariance[0] = orientation_variances_.x;
  msg.orientation_covariance[4] = orientation_variances_.y;
  msg.orientation_covariance[8] = orientation_variances_.z;

  // Set linear acceleration
  msg.linear_acceleration.x = vs_data_register.accel[0] - linear_acc_zero_offsets_.x;
  msg.linear_acceleration.y = vs_data_register.accel[1] - linear_acc_zero_offsets_.y;
  msg.linear_acceleration.z = vs_data_register.accel[2] - linear_acc_zero_offsets_.z;
  msg.linear_acceleration_covariance[0] = linear_acc_variances_.x;
  msg.linear_acceleration_covariance[4] = linear_acc_variances_.y;
  msg.linear_acceleration_covariance[8] = linear_acc_variances_.z;

  // Set angular velocity
  msg.angular_velocity.x = vs_data_register.gyro[0] - angular_vel_zero_offsets_.x;
  msg.angular_velocity.y = vs_data_register.gyro[1] - angular_vel_zero_offsets_.y;
  msg.angular_velocity.z = vs_data_register.gyro[2] - angular_vel_zero_offsets_.z;
  msg.angular_velocity_covariance[0] = angular_vel_variances_.x;
  msg.angular_velocity_covariance[4] = angular_vel_variances_.y;
  msg.angular_velocity_covariance[8] = angular_vel_variances_.z;

  imu_pub_->publish(msg);

  // Broadcast TF for visualization
  if (publish_viz_) {
    // Flip by 180 degrees for visualization in RViz (IMU frame has z-axis pointing DOWN)
    tf2::Quaternion viz_orientation = tf2::Quaternion(0,1,0,0) * q;
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = this->get_clock()->now();
    transform_stamped.header.frame_id = "map";
    transform_stamped.child_frame_id = "imu_viz";
    transform_stamped.transform.rotation = tf2::toMsg(viz_orientation);

    // Fix translation so that it is not coincident with map (and can be easier visualized)
    transform_stamped.transform.translation.x = 1;
    transform_stamped.transform.translation.y = 1;
    transform_stamped.transform.translation.z = 1;
    viz_tf_broadcaster_->sendTransform(transform_stamped);
  }
}

void VnImuNode::loadParamToVector3(const std::string & param_name,
    geometry_msgs::msg::Vector3 & v) {
  this->declare_parameter<double>(param_name+".x", 0.0);
  this->get_parameter(param_name+".x", v.x);
  this->declare_parameter<double>(param_name+".y", 0.0);
  this->get_parameter(param_name+".y", v.y);
  this->declare_parameter<double>(param_name+".z", 0.0);
  this->get_parameter(param_name+".z", v.z);
}

}  // namespace imu
}  // namespace cg
