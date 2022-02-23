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
  viz_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
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

  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "imu";

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

}  // namespace imu
}  // namespace cg
