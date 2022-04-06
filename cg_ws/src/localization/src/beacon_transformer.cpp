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
    "/imu/data/base_link", 10, std::bind(&BeaconTransformer::imu_callback, this, _1));
      
  tag_1_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/uwb_beacon/base_link_transformed/tag_0", 1);

  tag_0_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/uwb_beacon/base_link_transformed/tag_1", 1);

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
      tag_0_transformStamped.transform.rotation = base_link_transform.transform.rotation;
      tf2::doTransform(updated_pose_0_, updated_pose_0_, tag_0_transformStamped);
      tag_0_pub_->publish(updated_pose_0_);
    }
    else if (got_imu) {
      tag_0_transformStamped.transform.rotation = imu_last.orientation;
      tf2::doTransform(updated_pose_0_, updated_pose_0_, tag_0_transformStamped);
      tag_0_pub_->publish(updated_pose_0_);
    }
}

void BeaconTransformer::beacon_callback_1(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr beacon_msg)
{
    updated_pose_1_ = *beacon_msg;
    if (got_tf)
    {
      tag_1_transformStamped.transform.rotation = base_link_transform.transform.rotation;
      tf2::doTransform(updated_pose_1_, updated_pose_1_, tag_1_transformStamped);
      tag_1_pub_->publish(updated_pose_1_);
    }
    else if (got_imu) {
      tag_1_transformStamped.transform.rotation = imu_last.orientation;
      tf2::doTransform(updated_pose_1_, updated_pose_1_, tag_1_transformStamped);
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
  tf_update(base_link_frame, tag_0_frame, tag_0_transformStamped);
  tf_update(base_link_frame, tag_1_frame, tag_1_transformStamped);
  tf_update(map_frame, base_link_frame, base_link_transform);
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