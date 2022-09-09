#include "mapping/site_map_node.hpp"

namespace cg {
namespace mapping {

SiteMapNode::SiteMapNode() : Node("site_map_node") {
  // Initialize publishers and subscribers
  new_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1, std::bind(&SiteMapNode::newPtsCallback, this, std::placeholders::_1));
  visualization_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/site_map_viz", 1);

  // Timer callback
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SiteMapNode::timerCallback, this));

  // Load parameters
  // TODO: CONVERT TO PARAMS
  size_t cHeight = 70;
  size_t cWidth = 70; 
  float cResolution = 0.1;

  cg::mapping::SiteMap temp(cHeight,cWidth,cResolution);
  siteMap_ = temp;
}

void SiteMapNode::timerCallback(){
  // Report the current mode
  // RCLCPP_INFO(this->get_logger(), "visualizing site map...");

  // get a temp map from siteMap
  std::vector<float> tempMap = siteMap_.getHeightMap();
  size_t iterator = 0; 

  // Pack data
  pcl::PointCloud<pcl::PointXYZ> myCloud;

  for (int col=0; col<static_cast<int>(siteMap_.getWidth()); col++) {
    for (int row=0; row < static_cast<int>(siteMap_.getHeight()); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = col * siteMap_.getResolution(); // cell x-coordinate in map frame
      newPoint.y = row * siteMap_.getResolution(); // cell y-coordinate in map frame
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
  siteMap_.updateCells();

}

// TODO SERVICE FOR SITEMAP OBJECT PUBLISH

  // PUBLISH PTR FOR THE SITEMAP OBJECT SO PLANNER CAN ACCESS


}  // namespace mapping
}  // namespace cg
