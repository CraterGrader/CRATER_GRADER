#include "mapping/site_map_node.hpp"

namespace cg {
namespace mapping {

SiteMapNode::SiteMapNode() : Node("site_map_node") {
  // Initialize publishers and subscribers
  new_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain/filtered", 1, std::bind(&SiteMapNode::new_pts_callback, this, std::placeholders::_1));
  visualization_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/site_map_viz", 1);

  // Viz Timer callback
  viz_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SiteMapNode::map_viz_callback, this));

  // Initialize services
  site_map_server_ = this->create_service<cg_msgs::srv::SiteMap>(
      "site_map_server",
      std::bind(&SiteMapNode::sendSiteMap, this, std::placeholders::_1, std::placeholders::_2));
  save_map_server_ = this->create_service<cg_msgs::srv::SaveMap>(
      "save_map_server",
      std::bind(&SiteMapNode::saveMap, this, std::placeholders::_1, std::placeholders::_2));

  // Load parameters
  this->declare_parameter<int>("height", 5);
  this->get_parameter("height", height_);
  this->declare_parameter<int>("width", 5);
  this->get_parameter("width", width_);
  this->declare_parameter<float>("resolution", 1.0);
  this->get_parameter("resolution", resolution_);
  this->declare_parameter<float>("xTransform", 5);
  this->get_parameter("xTransform", xTransform_);
  this->declare_parameter<float>("yTransform", 5);
  this->get_parameter("yTransform", yTransform_);
  this->declare_parameter<float>("unseenGridHeight", 5);
  this->get_parameter("unseenGridHeight", unseenGridHeight_);
  this->declare_parameter<float>("incomingPointVariance", 5);
  this->get_parameter("incomingPointVariance", incomingPointVariance_);
  this->declare_parameter<float>("cellStartingVariance", 5);
  this->get_parameter("cellStartingVariance", cellStartingVariance_);
  this->declare_parameter<float>("minCellVariance", 5);
  this->get_parameter("minCellVariance", minCellVariance_);

  cg::mapping::SiteMap temp(height_, 
                              width_, 
                              resolution_, 
                              xTransform_, 
                              yTransform_, 
                              unseenGridHeight_, 
                              incomingPointVariance_, 
                              cellStartingVariance_, 
                              minCellVariance_);
  siteMap_ = temp;
}

void SiteMapNode::map_viz_callback(){
  // get a temp map from siteMap
  std::vector<float> tempMap = siteMap_.getHeightMap();
  size_t iterator = 0; 

  // Pack data
  pcl::PointCloud<pcl::PointXYZ> myCloud;

  for (int col=0; col<static_cast<int>(siteMap_.getWidth()); col++) {
    for (int row=0; row < static_cast<int>(siteMap_.getHeight()); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = (row * siteMap_.getResolution()) + siteMap_.getXTransform() + (siteMap_.getResolution()/2); // cell x-coordinate in map frame
      newPoint.y = col * siteMap_.getResolution() + siteMap_.getYTransform() + (siteMap_.getResolution()/2); // cell y-coordinate in map frame
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

void SiteMapNode::new_pts_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

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
  siteMap_.updateCellsBayes();
}

void SiteMapNode::sendSiteMap(cg_msgs::srv::SiteMap::Request::SharedPtr req, cg_msgs::srv::SiteMap::Response::SharedPtr res)
{
  (void)req; // No request input for cg_msgs/srv/SiteMap.srv, but service needs both Request and Response args so just "touch" the request to hide unused parameter warning
  cg_msgs::msg::SiteMap map_msg = siteMap_.toMsg();
  res->site_map = map_msg;
  res->success = true;
  siteMap_.updateMapCoverage();
  siteMap_.normalizeHeightMap();
  res->map_fully_explored = siteMap_.getSiteMapFullStatus();
}

void SiteMapNode::saveMap(cg_msgs::srv::SaveMap::Request::SharedPtr req, cg_msgs::srv::SaveMap::Response::SharedPtr res) {
  // Update map parameters and data
  saveMap_.updateDimensions(siteMap_.getHeight(), siteMap_.getWidth(), siteMap_.getResolution());
  saveMap_.setCellData(siteMap_.getHeightMap());
  // Save the map using input filepath and return status
  res->map_saved = saveMap_.write_map_to_file(req->filepath);
}

}  // namespace mapping
}  // namespace cg
