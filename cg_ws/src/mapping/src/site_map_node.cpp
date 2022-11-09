#include "mapping/site_map_node.hpp"

namespace cg {
namespace mapping {

SiteMapNode::SiteMapNode() : Node("site_map_node") {


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
  this->declare_parameter<int>("height", 50);
  this->get_parameter("height", height_);
  this->declare_parameter<int>("width", 50);
  this->get_parameter("width", width_);
  this->declare_parameter<float>("resolution", 0.1);
  this->get_parameter("resolution", resolution_);
  this->declare_parameter<float>("xTransform", 1.0);
  this->get_parameter("xTransform", xTransform_);
  this->declare_parameter<float>("yTransform", 1.0);
  this->get_parameter("yTransform", yTransform_);
  this->declare_parameter<float>("unseenGridHeight", 0.0);
  this->get_parameter("unseenGridHeight", unseenGridHeight_);
  this->declare_parameter<float>("incomingPointVariance", 0.05);
  this->get_parameter("incomingPointVariance", incomingPointVariance_);
  this->declare_parameter<float>("cellStartingVariance", 1.0);
  this->get_parameter("cellStartingVariance", cellStartingVariance_);
  this->declare_parameter<float>("minCellVariance", 0.0001);
  this->get_parameter("minCellVariance", minCellVariance_);

  std::string load_height_map_filepath;
  bool load_height_map_from_filepath;
  this->declare_parameter<std::string>("load_height_map_filepath", "/root/CRATER_GRADER/cg_ws/src/mapping/config/half_auton_half_manual_exploration.csv");
  this->get_parameter("load_height_map_filepath", load_height_map_filepath);
  this->declare_parameter<bool>("load_height_map_from_filepath", false);
  this->get_parameter("load_height_map_from_filepath", load_height_map_from_filepath);
  this->declare_parameter<bool>("static_map", false);
  this->get_parameter("static_map", static_map_);

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
  // std::cout << "Constructed SiteMap Objects" << std::endl;

  // Load map from file
  if (load_height_map_from_filepath) {
    bool load_height_map_initialized = fileMap_.load_map_from_file(load_height_map_filepath);
    if (!load_height_map_initialized) {
      RCLCPP_FATAL(this->get_logger(), "Loading from file map error");
      rclcpp::shutdown();
    }

    if ((fileMap_.getHeight() != siteMap_.getHeight()) \
          || (fileMap_.getWidth() != siteMap_.getWidth()) \
          || (fileMap_.getResolution() != siteMap_.getResolution())) {
      RCLCPP_FATAL(this->get_logger(), "Map dimensions do not align!\n    Site map <height, width, resolution, data.size()> <%ld, %ld, %f, %ld>\n    File map <height, width, resolution, data.size()>: <%ld, %ld, %f, %ld>", siteMap_.getHeight(), siteMap_.getWidth(), siteMap_.getResolution(), siteMap_.getHeightMap().size(), fileMap_.getHeight(), fileMap_.getWidth(), fileMap_.getResolution(), fileMap_.getCellData().size());
      rclcpp::shutdown();
    }

    // Set the cell data
    siteMap_.setHeightMap(fileMap_.getCellData());
    
    // Assume loaded map has full coverage
    std::vector<int> full_coverage(siteMap_.getHeight() * siteMap_.getHeight(), 1);
    siteMap_.setSeenMap(full_coverage);
    siteMap_.updateMapCoverage();
  }

  // Initialize publishers and subscribers
  new_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain/filtered", 1, std::bind(&SiteMapNode::new_pts_callback, this, std::placeholders::_1));
  visualization_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/viz/mapping/site_map_viz", 1);
  visualization_seen_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/viz/mapping/site_map_seen_viz", 1);
  visualization_variance_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/viz/mapping/site_map_variance", 1);
  // std::cout << "Started sub" << std::endl;

}

void SiteMapNode::map_viz_callback(){
  // HEIGHT MAP
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // get a temp map from siteMap
  std::vector<float> tempMap = siteMap_.getHeightMap();
  size_t iterator0 = 0;

  // std::cout << "Started viz callback" << std::endl;


  // Pack data
  // TODO turn this into util
  pcl::PointCloud<pcl::PointXYZ> myCloud;

  for (int col=0; col<static_cast<int>(siteMap_.getWidth()); col++) {
    for (int row=0; row < static_cast<int>(siteMap_.getHeight()); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = (row * siteMap_.getResolution()) + siteMap_.getXTransform() + (siteMap_.getResolution()/2); // cell x-coordinate in map frame
      newPoint.y = col * siteMap_.getResolution() + siteMap_.getYTransform() + (siteMap_.getResolution()/2); // cell y-coordinate in map frame
      newPoint.z =  tempMap[iterator0]; // cell height in map frame
      myCloud.points.push_back(newPoint);
      iterator0++;
    }
  }

  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 site_map_viz_msg;
  pcl::toROSMsg(myCloud, site_map_viz_msg);

  // Publish the message
  site_map_viz_msg.header.frame_id = "map";
  site_map_viz_msg.header.stamp = this->get_clock()->now();
  visualization_pub_->publish(site_map_viz_msg);


  // Seen MAP
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // get a temp map from siteMap
  std::vector<int> tempMapSeen = siteMap_.getSeenMap();
  size_t iterator1 = 0;

  // Pack data
  // TODO turn this into util
  pcl::PointCloud<pcl::PointXYZ> myCloudSeen;

  for (int col=0; col<static_cast<int>(siteMap_.getWidth()); col++) {
    for (int row=0; row < static_cast<int>(siteMap_.getHeight()); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = (row * siteMap_.getResolution()) + siteMap_.getXTransform() + (siteMap_.getResolution()/2); // cell x-coordinate in map frame
      newPoint.y = col * siteMap_.getResolution() + siteMap_.getYTransform() + (siteMap_.getResolution()/2); // cell y-coordinate in map frame
      newPoint.z = static_cast<float>(tempMapSeen[iterator1]); // cell height in map frame
      myCloudSeen.points.push_back(newPoint);
      iterator1++;
    }
  }

  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 site_map_viz_seen_msg;
  pcl::toROSMsg(myCloudSeen, site_map_viz_seen_msg);

  // Publish the message
  site_map_viz_seen_msg.header.frame_id = "map";
  site_map_viz_seen_msg.header.stamp = this->get_clock()->now();
  visualization_seen_map_pub_->publish(site_map_viz_seen_msg);



  // Variance Map
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  // get a temp map from siteMap
  std::vector<float> tempMapVariance = siteMap_.getVarianceMapFloats();
  size_t iterator2 = 0;

  // Pack data
  // TODO turn this into util
  pcl::PointCloud<pcl::PointXYZ> myCloudVariance;

  for (int col=0; col<static_cast<int>(siteMap_.getWidth()); col++) {
    for (int row=0; row < static_cast<int>(siteMap_.getHeight()); row++) {
      pcl::PointXYZ newPoint;
      newPoint.x = (row * siteMap_.getResolution()) + siteMap_.getXTransform() + (siteMap_.getResolution()/2); // cell x-coordinate in map frame
      newPoint.y = col * siteMap_.getResolution() + siteMap_.getYTransform() + (siteMap_.getResolution()/2); // cell y-coordinate in map frame
      newPoint.z = tempMapVariance[iterator2]; // cell height in map frame
      myCloudVariance.points.push_back(newPoint);
      iterator2++;
    }
  }

  // Convert to ROS data type
  sensor_msgs::msg::PointCloud2 site_map_viz_variance_msg;
  pcl::toROSMsg(myCloudVariance, site_map_viz_variance_msg);

  // Publish the message
  site_map_viz_variance_msg.header.frame_id = "map";
  site_map_viz_variance_msg.header.stamp = this->get_clock()->now();
  visualization_variance_map_pub_->publish(site_map_viz_variance_msg);

}

void SiteMapNode::new_pts_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

  // std::cout << "Started new pts callback" << std::endl;


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
  // if the map should be updated
  if(!static_map_){
    // BIN POINTS
    siteMap_.binPts(incomingPts);
    // UPDATE CELLS
    siteMap_.updateCellsBayes();
  }
}

void SiteMapNode::sendSiteMap(cg_msgs::srv::SiteMap::Request::SharedPtr req, cg_msgs::srv::SiteMap::Response::SharedPtr res)
{
  (void)req; // No request input for cg_msgs/srv/SiteMap.srv, but service needs both Request and Response args so just "touch" the request to hide unused parameter warning
  siteMap_.normalizeHeightMap();
  cg_msgs::msg::SiteMap map_msg = siteMap_.toMsg();
  res->site_map = map_msg;
  res->success = true;
  res->map_fully_explored = siteMap_.getSiteMapFullStatus();
  res->map_coverage_ratio = siteMap_.getSiteMapCoverage();
  for (int seen: siteMap_.getSeenMap()) {
    res->seen_map.push_back(seen);
  }
}

void SiteMapNode::saveMap(cg_msgs::srv::SaveMap::Request::SharedPtr req, cg_msgs::srv::SaveMap::Response::SharedPtr res) {
  // Update map parameters and data
  fileMap_.updateDimensions(siteMap_.getHeight(), siteMap_.getWidth(), siteMap_.getResolution());
  fileMap_.setCellData(siteMap_.getHeightMap());
  // Save the map using input filepath and return status
  res->map_saved = fileMap_.write_map_to_file(req->filepath);
}

}  // namespace mapping
}  // namespace cg
