#include "bearing/bearing_node.hpp"

namespace cg {
namespace bearing {

BearingNode::BearingNode() : Node("bearing_node") {
  // Publishing frequency for callback function
  this->declare_parameter<int>("pub_freq", 10);
  this->get_parameter("pub_freq", pub_freq);

  // Rolling average buffer length for yaw
  this->declare_parameter<int>("rolling_avg_buffer", 5);
  this->get_parameter("rolling_avg_buffer", this->rolling_avg_buffer);

  // Time before transforms are considered old and discarded [s]
  this->declare_parameter<double>("tf_discard_time", 0.3);
  this->get_parameter("tf_discard_time", this->tf_discard_time);

  // Bearing covariance
  this->declare_parameter<double>("bearing_covariance", 0.005);
  this->get_parameter("bearing_covariance", this->bearing_covariance);

  // Initialize publishers and subscribers
  bearing_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/bearing", 1
  );

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

  // Not sure if good idea as launch file opens node in random order
  std::string fromMap = "tag_map";
  std::string toCamera = "camera";
  std::string toTag_base = "tag";

  // Bearing published relative to map frame
  std::string tf_map_frame = "map";
  
  // Store bearing estimates and distances
  std::vector<double> bearings;
  std::vector<double> tag_dist_to_cam;

  // Declare tf frames
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
      cam_to_tag = tf_buffer_->lookupTransform(
        toCamera, fromTag,
        tf2::TimePointZero);
      rclcpp::Time tf_time = cam_to_tag.header.stamp;
      // If the camera to tag transform is more than 0.3 seconds old, discard
      double dt = (this->get_clock()->now() - tf_time).seconds();

      if (dt > this->tf_discard_time) {
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
    tf_map_to_link = tf_map_to_tag.inverse() * tf_tag_to_link.inverse();

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
    // Find the best tag to get angle from
    double best_bearing = bearings[0];
    double min_dist = tag_dist_to_cam[0];

    for (int j = 0; j < static_cast<int>(bearings.size()); j++) {
      if (tag_dist_to_cam[j] < min_dist) {
        best_bearing = bearings[j];
        min_dist = tag_dist_to_cam[j];
      }
    }
    
    // Variables for average bearing over the buffer length
    double avg_bearing;
    double sum_y;
    double sum_x;
    
    // Instead of averaging the bearing, sum the unit vectors
    this->rolling_sin.push_back(std::sin(best_bearing));
    this->rolling_cos.push_back(std::cos(best_bearing));

    // Erase the oldest bearing if the buffer is longer than the limit
    if (this->rolling_sin.size() > (long unsigned int)rolling_avg_buffer) {
      this->rolling_sin.erase(rolling_sin.begin());
      this->rolling_cos.erase(rolling_cos.begin());
    }

    // Average the unit vectors and get angle
    sum_y = std::accumulate(this->rolling_sin.begin(), this->rolling_sin.end(), 0.0);
    sum_x = std::accumulate(this->rolling_cos.begin(), this->rolling_cos.end(), 0.0);
    avg_bearing = std::atan2(sum_y, sum_x);

    // Set the PoseWithCovarianceStamped message's position to 0
    bearing.pose.pose.position.x = 0.0;
    bearing.pose.pose.position.y = 0.0;
    bearing.pose.pose.position.z = 0.0;

    // Convert yaw back into a quarternion for orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, avg_bearing);

    // Set the quarternion in the message
    bearing.pose.pose.orientation.x = q.x();
    bearing.pose.pose.orientation.y = q.y();
    bearing.pose.pose.orientation.z = q.z();
    bearing.pose.pose.orientation.w = q.w();

    // Set header and frame name
    bearing.header.stamp = this->get_clock()->now();
    bearing.header.frame_id = tf_map_frame;

    // Set covariance
    bearing.pose.covariance =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, this->bearing_covariance};

    // Publish the estimated bearing  
    bearing_pub_->publish(bearing);
  }
  else {
    RCLCPP_WARN(this->get_logger(), "No bearing tags were detected.");
  }
}

}  // namespace teleop
}  // namespace cg
