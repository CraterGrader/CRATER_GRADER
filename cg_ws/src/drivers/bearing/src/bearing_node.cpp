#include "bearing/bearing_node.hpp"

namespace cg {
namespace bearing {

BearingNode::BearingNode() : Node("bearing_node") {
  // Publishing frequency for callback function
  int pub_freq;
  this->declare_parameter<int>("pub_freq", 10);
  this->get_parameter("pub_freq", pub_freq);

  // Rolling average buffer length for yaw
  this->declare_parameter<int>("rolling_avg_buffer", 5);
  this->get_parameter("rolling_avg_buffer", this->rolling_avg_buffer);

  // Time before transforms are considered old and discarded [s]
  this->declare_parameter<double>("tf_discard_time", 0.3);
  this->get_parameter("tf_discard_time", this->tf_discard_time);

  // Initialize publishers and subscribers
  bearing_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/bearing", 1
  );

  full_tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Create listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Call on_timer function at a specified frequency
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&BearingNode::timerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void BearingNode::timerCallback() {
  // Radians
  geometry_msgs::msg::PoseWithCovarianceStamped bearing; 

  // Frames to be used
  std::string fromTag_base = "april_tag";
  std::string toLink = "rob_base_link";

  // Static frames to be used - TODO: Move this out of a callback to improve speed
  // Not sure if good idea as launch file opens node in random order
  std::string fromMap = "tag_map";
  std::string toCamera = "camera";
  std::string toTag_base = "tag";

  // Bearing published relative to map frame
  std::string tf_map_frame = "map";
  
  // Store bearing estimates and distances
  std::vector<double> bearings;
  std::vector<double> tag_dist_to_cam;

  tf2::Transform tf_tag_to_link;
  tf2::Transform tf_map_to_tag;
  tf2::Transform tf_map_to_link;

  // Get overall transform from map to base link
  geometry_msgs::msg::TransformStamped map_to_link;

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
      //rclcpp::Time now = this->get_clock()->now();
      cam_to_tag = tf_buffer_->lookupTransform(
        toCamera, fromTag,
        tf2::TimePointZero);
      rclcpp::Time tf_time = cam_to_tag.header.stamp;
      // If the camera to tag transform is more than 0.3 seconds old, discard
      double dt = (this->get_clock()->now() - tf_time).seconds();
      //RCLCPP_INFO(this->get_logger(), "Time gap was %f", dt);
      if (dt > this->tf_discard_time) {
        continue;
      }
    } catch (tf2::TransformException & ex) {
      if (i == 2) {
      }
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
    tf_map_to_link = tf_map_to_tag.inverse() * tf_tag_to_link.inverse();

    // Publish overall tf - TESTING
    map_to_link.header.stamp = this->get_clock()->now();
    map_to_link.header.frame_id = fromMap;
    map_to_link.child_frame_id = "baseL";
    map_to_link.transform = toMsg(tf_map_to_link);

    // Send the transformation - TESTING
    full_tf_pub_->sendTransform(map_to_link);

    // Get Yaw
    double roll, pitch, yaw;
    tf_map_to_link.getBasis().getRPY(roll, pitch, yaw);

    // Calculate and append bearing
    bearings.push_back(yaw);

    // Get cartesian distance from camera to tag for weighting
    tag_dist_to_cam.push_back(sqrt(pow(cam_to_tag.transform.translation.x,2) + 
      pow(cam_to_tag.transform.translation.y,2) + 
      pow(cam_to_tag.transform.translation.z,2)));
  }
  if (bearings.size() > 0) {
    // Calculated a form of weighted average using distance from camera to tag
    double weighted_bearing = 0;
    double inv_total_dist = 0;

    for (int j = 0; j < (int)bearings.size(); j++) {
      weighted_bearing += bearings[j] * 1/tag_dist_to_cam[j];
      inv_total_dist += 1/tag_dist_to_cam[j];
    }
    weighted_bearing /= inv_total_dist;
    weighted_bearing += M_PI/2;

    // Note bearing - comment out after final product
    RCLCPP_INFO(this->get_logger(), "Bearing Angle [deg]: %f\n", weighted_bearing * 180 / M_PI);

    double avg_bearing;
    this->rolling_avg.push_back(weighted_bearing);
    if (this->rolling_avg.size() > (long unsigned int)rolling_avg_buffer) {
      this->rolling_avg.erase(rolling_avg.begin());
    }
    avg_bearing = std::accumulate(this->rolling_avg.begin(), 
                    this->rolling_avg.end(), 0.0) / this->rolling_avg.size();

    bearing.pose.pose.position.x = 0.0;
    bearing.pose.pose.position.y = 0.0;
    bearing.pose.pose.position.z = 0.0;

    // Convert yaw back into a quarternion for orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, avg_bearing);
    bearing.pose.pose.orientation.x = q.x();
    bearing.pose.pose.orientation.y = q.y();
    bearing.pose.pose.orientation.z = q.z();
    bearing.pose.pose.orientation.w = q.w();

    bearing.header.stamp = this->get_clock()->now();
    bearing.header.frame_id = tf_map_frame;

    bearing.pose.covariance =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.1};

    // Publish the estimated bearing
    bearing_pub_->publish(bearing);
  }
}

}  // namespace teleop
}  // namespace cg
