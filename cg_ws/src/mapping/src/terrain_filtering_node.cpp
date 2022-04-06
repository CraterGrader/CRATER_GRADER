#include "mapping/terrain_filtering_node.hpp"

namespace cg {
namespace mapping {

TerrainFilteringNode::TerrainFilteringNode() : Node("terrain_filtering_node"){
  // create ptr to listener 
  tfBuffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

  // Parameters 
  // TODO: add to a parameters file
  source_frame_ = "realsense_frame";
  target_frame_ = "odom";

  // Initialize publishers and subscribers
  filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1
  );
  raw_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/camera/depth/color/points", rclcpp::SensorDataQoS(), std::bind(&TerrainFilteringNode::rawPointsCallback, this, std::placeholders::_1)
  );
}

void TerrainFilteringNode::rawPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tfBuffer_->lookupTransform(target_frame_, source_frame_, tf2::TimePointZero);
    RCLCPP_INFO(this->get_logger(), "transformStamped"); 

    cloud_in_ = *msg;
        
    tf2::doTransform(cloud_in_, cloud_out_, transformStamped);
    cloud_out_.header.frame_id = target_frame_;

    filtered_points_pub_->publish(cloud_out_);
    
    } 
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(),"%s", ex.what());
    }
  
}

}  // namespace mapping
}  // namespace cg
