#include "bearing/bearing_node.hpp"

namespace cg {
namespace bearing {

BearingNode::BearingNode() : Node("bearing_node") {
  int pub_freq;
  this->declare_parameter<int>("pub_freq", 10);
  this->get_parameter("pub_freq", pub_freq);

  // Initialize publishers and subscribers
  bearing_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/bearing", 1
  );

  // Create listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Call on_timer function every second
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&BearingNode::timerCallback, this)
  );
}

void BearingNode::timerCallback() {
  // Radians
  geometry_msgs::msg::PoseWithCovarianceStamped bearing; 

  // Frames to be used
  std::string fromTag_base = "april_tag";
  std::string toLink = "base_link";

  // Static frames to be used - TODO: Move this out of a callback to improve speed
  // Not sure if good idea as launch file opens node in random order
  std::string fromMap = "tag_map";
  std::string toCamera = "camera";
  std::string toTag_base = "tag";
  
  // Store bearing estimates and distances
  std::vector<double> bearings;
  std::vector<double> tag_dist_to_cam;

  tf2::Transform tf_tag_to_link;
  tf2::Transform tf_map_to_tag;
  tf2::Transform tf_map_to_link;

  // Get overall transform from map to base link
  geometry_msgs::msg::Transform map_to_link;

  // Look up for the transformation from tag to base_link
  for (int i = 0; i < 4; i++) {
    // Transform to obtain tf
    geometry_msgs::msg::TransformStamped tag_to_link;
    geometry_msgs::msg::TransformStamped map_to_tag;
    geometry_msgs::msg::TransformStamped cam_to_tag;

    // Get the full fromTag frame id
    std::string fromTag = fromTag_base + std::to_string(i);

    // Get camera to tag transform - matters the most, needs to not only include most recent transform
    try {
      rclcpp::Time now = this->get_clock()->now();
      cam_to_tag = tf_buffer_->lookupTransform(
        toCamera, fromTag,
        tf2::TimePointZero);
      rclcpp::Time tf_time = cam_to_tag.header.stamp;
      if (tf_time < now_time - 0.05) {
        continue;
      }
    } catch (tf2::TransformException & ex) {
      continue;
    }

    // Get tag to base link transform
    try {
      tag_to_link = tf_buffer_->lookupTransform(
        toLink, fromTag,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      continue;
    }

    // Get map to tag transform
    std::string toTag = toTag_base + std::to_string(i);
    try {
      map_to_tag = tf_buffer_->lookupTransform(
        toTag, fromMap,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      continue;
    }

    // Convert messages to tf
    tf2::fromMsg(tag_to_link.transform, tf_tag_to_link);
    tf2::fromMsg(map_to_tag.transform, tf_map_to_tag);

    // Likely source of error
    tf_map_to_link = tf_map_to_tag * tf_tag_to_link;

    // Get Yaw
    double roll, pitch, yaw;
    tf_map_to_link.getBasis().getRPY(roll, pitch, yaw);
    RCLCPP_INFO(this->get_logger(), "The yaw is %f", yaw);

    // Calculate and append bearing
    RCLCPP_INFO(this->get_logger(), "%f\n", yaw * 180 / 3.1416);

    bearings.push_back(yaw * 180/3.1416);

    // Get cartesian distance from camera to tag for weighting
    tag_dist_to_cam.push_back(sqrt(pow(cam_to_tag.transform.translation.x,2) + 
      pow(cam_to_tag.transform.translation.y,2) + 
      pow(cam_to_tag.transform.translation.z,2)));
  }

  // Calculated a form of weighted average using distance from camera to tag
  double weighted_bearing = 0;
  double total_dist = 0;

  for (int j = 0; j < (int)bearings.size(); j++) {
    weighted_bearing = bearings[j] * 1/tag_dist_to_cam[j];
    total_dist += 1/tag_dist_to_cam[j];
    RCLCPP_INFO(this->get_logger(), "I heard: '%f and %f'", weighted_bearing, total_dist);
  }
  if (bearings.size() > 0) {
    bearing.pose.pose.position.x = 0.0;
    bearing.pose.pose.position.y = 0.0;
    bearing.pose.pose.position.z = 0.0;

    bearing.pose.pose.orientation.x = 0.0;
    bearing.pose.pose.orientation.y = 0.0;
    bearing.pose.pose.orientation.z = (float)(weighted_bearing / total_dist);
    bearing.pose.pose.orientation.w = 1.0;

    bearing.header.stamp = this->get_clock()->now();
    bearing.header.frame_id = "bearing";

    bearing.pose.covariance =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

    // Publish the estimated bearing
    bearing_pub_->publish(bearing);
  }
}

}  // namespace teleop
}  // namespace cg
