#include "localization/beacon_transformer.hpp"

using std::placeholders::_1;

namespace cg {
namespace uwb_beacon_rtls {


BeaconTransformer::BeaconTransformer() : Node("beacon_transformer")
{

  /* Publishers and Subscribers */
  beacon_subscription_0_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/uwb_beacon/raw_tags/tag_0", 10, std::bind(&BeaconTransformer::beacon_callback_0, this, _1));

  beacon_subscription_1_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/uwb_beacon/raw_tags/tag_1", 10, std::bind(&BeaconTransformer::beacon_callback_1, this, _1));

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 10, std::bind(&BeaconTransformer::imu_callback, this, _1));
      
  tag_1_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/tag_1_updated", 1);

  tag_0_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/tag_0_updated", 1);

  int pub_freq{100};
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&BeaconTransformer::tf_Callback, this)
  );

  // Tf listeners
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void BeaconTransformer::beacon_callback_0(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr beacon_msg)
{
    updated_pose_0_ = *beacon_msg;

    if (got_tf)
    {
      tf2::doTransform(updated_pose_0_, updated_pose_0_, tag_0_transformStamped);
      tag_0_pub_->publish(updated_pose_0_);
    }
    else if (got_imu) {

      // Flip IMU 180 and then set orientation to tag
      Eigen::Quaternion<double> r(0.0, 1.0, 0.0, 0.0);
      Eigen::Quaternion<double> imu_flipped_orientation = r * Eigen::Quaternion<double>(
        imu_last.orientation.w, imu_last.orientation.x, imu_last.orientation.y, imu_last.orientation.z) * r.inverse();

      updated_pose_0_.pose.pose.orientation.w = imu_flipped_orientation.w();
      updated_pose_0_.pose.pose.orientation.x = imu_flipped_orientation.x();
      updated_pose_0_.pose.pose.orientation.y = imu_flipped_orientation.y();
      updated_pose_0_.pose.pose.orientation.z = imu_flipped_orientation.z();
      tag_0_pub_->publish(updated_pose_0_);
    }

}

void BeaconTransformer::beacon_callback_1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr beacon_msg)
{
    updated_pose_1_ = *beacon_msg;

    if (got_tf)
    {
      tf2::doTransform(updated_pose_1_, updated_pose_1_, tag_1_transformStamped);
      tag_0_pub_->publish(updated_pose_1_);
    }
    else if (got_imu) {

      // Flip IMU 180 and then set orientation to tag
      Eigen::Quaternion<double> r(0.0, 1.0, 0.0, 0.0);
      Eigen::Quaternion<double> imu_flipped_orientation = r * Eigen::Quaternion<double>(
        imu_last.orientation.w, imu_last.orientation.x, imu_last.orientation.y, imu_last.orientation.z) * r.inverse();

      updated_pose_1_.pose.pose.orientation.w = imu_flipped_orientation.w();
      updated_pose_1_.pose.pose.orientation.x = imu_flipped_orientation.x();
      updated_pose_1_.pose.pose.orientation.y = imu_flipped_orientation.y();
      updated_pose_1_.pose.pose.orientation.z = imu_flipped_orientation.z();
      tag_1_pub_->publish(updated_pose_1_);
    }

}

void BeaconTransformer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    imu_last = *imu_msg;
    got_imu = true;
}

// Update tf transforms 
void BeaconTransformer::tf_Callback() 
{        
  tf_update(tag_0_frame, base_link_frame, tag_0_transformStamped);
  tf_update(tag_0_frame, base_link_frame, tag_1_transformStamped);
  got_tf = true;
}

// Update given tf transform
void BeaconTransformer::tf_update(std::string toFrameRel, std::string fromFrameRel, geometry_msgs::msg::TransformStamped &transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      toFrameRel, fromFrameRel,
      tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return;
  }
}


}  // namespace uwb_beacon_rtls
}  // namespace cg