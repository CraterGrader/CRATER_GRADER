#include "mapping/site_map_node.hpp"

namespace cg {
namespace mapping {

SiteMapNode::SiteMapNode() : Node("site_map_node") {
  // Initialize publishers and subscribers
  new_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1, std::bind(&SiteMapNode::newPtsCallback, this, std::placeholders::_1));
  visualization_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/site_map_viz", 1);
  // telem_sub_ = this->create_subscription<cg_msgs::msg::EncoderTelemetry>(
  //   "/encoder_telemetry", 1, std::bind(&SiteMapNode::telemCallback, this, std::placeholders::_1));

  // Timer callback
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SiteMapNode::timerCallback, this));

  // Load parameters
  this->declare_parameter<int>("height", 5);
  this->get_parameter("height", height_);

  this->declare_parameter<int>("width", 5);
  this->get_parameter("width", width_);

  this->declare_parameter<float>("resolution", 1.0);
  this->get_parameter("resolution", resolution_);

  cg::mapping::SiteMap temp(height_,width_,resolution_);
  siteMap_ = temp;
}

void SiteMapNode::timerCallback(){
  // get a temp map from siteMap
  std::vector<float> tempMap = siteMap_.getHeightMap();
  size_t iterator = 0; 

  // Pack data
  pcl::PointCloud<pcl::PointXYZ> myCloud;

  for (int col=0; col<static_cast<int>(siteMap_.getWidth()); col++) {
    for (int row=0; row < static_cast<int>(siteMap_.getHeight()); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = (col * siteMap_.getResolution()) + siteMap_.getXTransform() + (siteMap_.getResolution()/2); // cell x-coordinate in map frame
      newPoint.y = row * siteMap_.getResolution() + siteMap_.getYTransform() + (siteMap_.getResolution()/2); // cell y-coordinate in map frame
      newPoint.z =  tempMap[iterator]; // cell height in map frame
      myCloud.points.push_back(newPoint);
      iterator++; 
    }
  }

  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 site_map_viz_msg;
  pcl::toROSMsg(myCloud, site_map_viz_msg);

  // Publish the message
  site_map_viz_msg.header.frame_id = "map";
  site_map_viz_msg.header.stamp = this->get_clock()->now();
  visualization_pub_->publish(site_map_viz_msg);
}

void SiteMapNode::newPtsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

  std::vector<cg::mapping::mapPoint> incomingPts(temp_cloud->points.size());

  for (size_t i=0; i < temp_cloud->points.size(); i++){
    incomingPts[i].x = temp_cloud->points[i].x; 
    incomingPts[i].y = temp_cloud->points[i].y; 
    incomingPts[i].z = temp_cloud->points[i].z; 
  }

  // BIN POINTS 
  siteMap_.binPts(incomingPts);

  // UPDATE CELLS 
  // siteMap_.updateCellsMean();
  siteMap_.updateCellsBayes();
  // siteMap_.updateCellsMean()
}

// void SiteMapNode::telemCallback(const cg_msgs::msg::EncoderTelemetry::SharedPtr msg){
//     driveSpeed = static_cast<float> msg->drive_vel_rear; 
// }

}  // namespace mapping
}  // namespace cg
