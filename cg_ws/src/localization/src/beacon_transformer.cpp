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

  average_tag_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/uwb_beacon/average_tag", 1);

  int pub_freq{5};
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&BeaconTransformer::tf_Callback, this)
  );

  average_tag_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&BeaconTransformer::average_Beacon_Callback, this)
  );

  // Tf listeners
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void BeaconTransformer::beacon_callback_0(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr beacon_msg)
{
    updated_pose_0_ = *beacon_msg;
    raw_pose_0_ = *beacon_msg;
    if (got_tf)
    {
      tag_0_transformStamped.transform.rotation = base_link_transform.transform.rotation;
      tf2::doTransform(updated_pose_0_, updated_pose_0_, tag_0_transformStamped);
      tag_0_pub_->publish(updated_pose_0_);
      pub_tag_0 = true;
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
    raw_pose_1_ = *beacon_msg;
    if (got_tf)
    {
      tag_1_transformStamped.transform.rotation = base_link_transform.transform.rotation;
      tf2::doTransform(updated_pose_1_, updated_pose_1_, tag_1_transformStamped);
      tag_1_pub_->publish(updated_pose_1_);
      pub_tag_1 = true;
    }
    else if (got_imu) {
      tag_1_transformStamped.transform.rotation = imu_last.orientation;
      tf2::doTransform(updated_pose_1_, updated_pose_1_, tag_1_transformStamped);
      tag_1_pub_->publish(updated_pose_1_);
    }
}

void BeaconTransformer::average_Beacon_Callback() 
{      

  float tag_0_weight = 0.5;
  float tag_1_weight = 0.5;

  if (pub_tag_1 && pub_tag_0) 
  {
    
    float tag_0_dist  = sqrt(pow(tag_0_transformStamped.transform.translation.x - base_link_transform.transform.translation.x, 2) +
      pow(tag_0_transformStamped.transform.translation.y - base_link_transform.transform.translation.y, 2));

    float tag_1_dist = sqrt(pow(tag_1_transformStamped.transform.translation.x - base_link_transform.transform.translation.x, 2) +
      pow(tag_1_transformStamped.transform.translation.y - base_link_transform.transform.translation.y, 2));

    tag_0_weight = (1 - tag_0_dist) / (tag_1_dist + tag_0_dist);
    tag_1_weight = 1 - tag_0_weight;

  }

    average_pose_.pose.pose.position.x = tag_0_weight * (raw_pose_0_.pose.pose.position.x) + tag_1_weight * (raw_pose_1_.pose.pose.position.x);
    average_pose_.pose.pose.position.y = tag_0_weight * (raw_pose_0_.pose.pose.position.y) + tag_1_weight * (raw_pose_1_.pose.pose.position.y);
    average_pose_.pose.pose.position.z = tag_0_weight * (raw_pose_0_.pose.pose.position.z) + tag_1_weight * (raw_pose_1_.pose.pose.position.z);
    // average_pose_.pose.pose.position.z = 0; // TODO z position is temporarily fixed at 0 until we can get better z estimates 
    average_pose_.header = raw_pose_0_.header;
    average_pose_.pose.covariance = raw_pose_0_.pose.covariance;
    average_tag_pub_->publish(average_pose_);

}

void BeaconTransformer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    imu_last = *imu_msg;
    got_imu = true;
}

// Update tf transforms 
void BeaconTransformer::tf_Callback() 
{        
  bool got_tf1 = tf_update(map_frame, tag_0_frame, tag_0_transformStamped);
  bool got_tf2 = tf_update(map_frame, tag_1_frame, tag_1_transformStamped);
  bool got_tf3 = tf_update(map_frame, base_link_frame, base_link_transform);
  got_tf = got_tf1 && got_tf2 && got_tf3;
}

// Update given tf transform
bool BeaconTransformer::tf_update(std::string toFrameRel, std::string fromFrameRel, geometry_msgs::msg::TransformStamped &transform)
{
  try {
    transform = tf_buffer_->lookupTransform(
      toFrameRel, fromFrameRel,
      tf2::TimePointZero);
      return true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      this->get_logger(), "Could not transform %s to %s: %s",
      toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
    return false;
  }
}


}  // namespace uwb_beacon_rtls
}  // namespace cg