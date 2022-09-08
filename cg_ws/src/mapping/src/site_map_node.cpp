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

  // TODO: FIGURE OUT DEBUG VIZ FOR MAP
    // COULD BE HEIGHTMAP FROM ANYBOTICS
    // COULD BE BITMAP IMAGE
    // COULD BE CUSTOM VIZ

  // Load parameters
  // TODO: CONVERT TO PARAMS
  size_t cHeight = 70;
  size_t cWidth = 70; 
  float cResolution = 0.1;

  cg::mapping::SiteMap temp(cHeight,cWidth,cResolution);
  siteMap_ = temp;

  mapTemp.resize(siteMap_.getNcells());

}

void SiteMapNode::timerCallback()
{
  // dummy map data
  int width = 70; // [-]
  int height = 70; // [-]
  float cell_res = 0.1; // [m]
  // Report the current mode
  RCLCPP_INFO(this->get_logger(), "visualizing site map...");

  // Pack data
  pcl::PointCloud<pcl::PointXYZ> myCloud;
  
  
  for (int col=static_cast<int>(-1 * width/2); col<static_cast<int>(width/2); col++) {
    for (int row = static_cast<int>(-height/2); row < static_cast<int>(height/2); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = col * cell_res; // cell x-coordinate in map frame
      newPoint.y = row * cell_res; // cell y-coordinate in map frame
      newPoint.z = (rand() * 0.2) / RAND_MAX; // cell height in map frame
      myCloud.points.push_back(newPoint);
    }
  }

  // fill cloud with random points
  // for (int v = 0; v < 1000; ++v)
  // {
  //   pcl::PointXYZ newPoint;
  //   newPoint.x = (rand() * 5.0) / RAND_MAX;
  //   newPoint.y = (rand() * 5.0) / RAND_MAX;
  //   newPoint.z = (rand() * 0.5) / RAND_MAX;
  //   myCloud.points.push_back(newPoint);
  // }

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

  mapTemp = siteMap_.getHeightMap();

  size_t temp = siteMap_.getNcells(); 

  RCLCPP_INFO_STREAM(this->get_logger(), " ");
  RCLCPP_INFO_STREAM(this->get_logger(), " Number of Cells " << temp );

  for (size_t i=0; i < mapTemp.size(); i++){
    RCLCPP_INFO_STREAM(this->get_logger(), " " << mapTemp[i] );
  }

}

// TODO ADD TIMER CALLBACK 

  // PUBLISH HEIGHTMAP AT SOME FREQUENCY FOR VIZUALIZATION


// TODO SERVICE FOR SITEMAP OBJECT PUBLISH

  // PUBLISH PTR FOR THE SITEMAP OBJECT SO PLANNER CAN ACCESS


}  // namespace teleop
}  // namespace cg
