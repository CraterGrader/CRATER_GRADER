#include "mapping/site_map_node.hpp"

namespace cg {
namespace mapping {

SiteMapNode::SiteMapNode() : Node("site_map_node") {
  // Initialize publishers and subscribers

  new_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/terrain/filtered", 1, std::bind(&SiteMapNode::newPtsCallback, this, std::placeholders::_1));

  // TODO: FIGURE OUT DEBUG VIZ FOR MAP
    // COULD BE HEIGHTMAP FROM ANYBOTICS
    // COULD BE BITMAP IMAGE
    // COULD BE CUSTOM VIZ

  // Load parameters
  // TODO: CONVERT TO PARAMS
  size_t cHeight = 7;
  size_t cWidth = 7; 
  float cResolution = 1.0;

  cg::mapping::SiteMap temp(cHeight,cWidth,cResolution);
  siteMap_ = temp;

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

  std::vector<float> mapTemp = siteMap_.getHeightMap();

  for (size_t i=0; i < mapTemp.size(); i++){
    RCLCPP_INFO_STREAM(this->get_logger(), " " << mapTemp[i] );
  }
  RCLCPP_INFO_STREAM(this->get_logger(), " ");

}

// TODO ADD TIMER CALLBACK 

  // PUBLISH HEIGHTMAP AT SOME FREQUENCY FOR VIZUALIZATION


// TODO SERVICE FOR SITEMAP OBJECT PUBLISH

  // PUBLISH PTR FOR THE SITEMAP OBJECT SO PLANNER CAN ACCESS


}  // namespace teleop
}  // namespace cg
