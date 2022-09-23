#include "localization/ts_prism_transformer.hpp"

using std::placeholders::_1;

namespace cg {
namespace total_station_rtls {


TSPrismTransformer::TSPrismTransformer() : Node("ts_prism_transformer")
{

  /* Publishers and Subscribers */
  ts_prism_subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/total_station_prism", 10, std::bind(&TSPrismTransformer::ts_prism_callback, this, _1));

  imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data/base_link", 10, std::bind(&TSPrismTransformer::imu_callback, this, _1));
      
  transformed_ts_prism_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
  "/transformed_total_station_prism", 1);

  int pub_freq{10};
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&TSPrismTransformer::tf_Callback, this)
  );

  // Tf listeners
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

}

void TSPrismTransformer::ts_prism_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr prism_msg)
{
    updated_pose_ = *prism_msg;
    if (got_tf)
    {
      // ts_prism_transformStamped.transform.rotation = base_link_transform.transform.rotation;

      base_link_transform.transform.translation.x = 0;
      base_link_transform.transform.translation.y = 0;
      base_link_transform.transform.translation.z = 0; 
      // std::cout << ts_prism_transformStamped.transform.translation.x << ", " 
      //           << ts_prism_transformStamped.transform.translation.y << ", " 
      //           << ts_prism_transformStamped.transform.translation.z << std::endl ;
      ts_prism_transformStamped.transform.rotation.w = 1;
      ts_prism_transformStamped.transform.rotation.x = 0;
      ts_prism_transformStamped.transform.rotation.y = 0;
      ts_prism_transformStamped.transform.rotation.z = 0; 
      tf2::doTransform(updated_pose_, updated_pose_, base_link_transform);
      tf2::doTransform(updated_pose_, updated_pose_, ts_prism_transformStamped);
      transformed_ts_prism_pub_->publish(updated_pose_);
    }
    else if (got_imu) {
      ts_prism_transformStamped.transform.rotation = imu_last.orientation;
      tf2::doTransform(updated_pose_, updated_pose_, ts_prism_transformStamped);
      transformed_ts_prism_pub_->publish(updated_pose_);
    }
}

void TSPrismTransformer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    imu_last = *imu_msg;
    got_imu = true;
}

// Update tf transforms 
void TSPrismTransformer::tf_Callback() 
{        
  bool got_tf1 = tf_update(base_link_frame, prism_frame, ts_prism_transformStamped);
  // if (got_tf1) std::cout << "GOT TF1" << std::endl;
  bool got_tf2 = tf_update(map_frame, base_link_frame, base_link_transform);
  // if (got_tf2) std::cout << "GOT TF2" << std::endl;
  got_tf = got_tf1 && got_tf2;
}

// Update given tf transform
bool TSPrismTransformer::tf_update(std::string toFrameRel, std::string fromFrameRel, geometry_msgs::msg::TransformStamped &transform)
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


}  // namespace total_station_rtls
}  // namespace cg