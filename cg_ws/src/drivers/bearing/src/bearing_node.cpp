#include "bearing/bearing_node.hpp"

namespace cg {
namespace bearing {

BearingNode::BearingNode() : Node("bearing_node") {
  // Publishing frequency for callback function
  this->declare_parameter<int>("pub_freq", 30);
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
  // bearing_pub_ = this->create_publisher<std_msgs::msg::Float32>(
  bearing_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/bearing", 1
  );
  pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered/ekf_global_node", 1, std::bind(&BearingNode::poseUpdateCallback,
    this, std::placeholders::_1));

  // Create listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize robot position to invalid value
  robot_x = -1;
  robot_y = -1;
  robot_pitch = -1;

  // Call on_timer function at a specified frequency
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000/pub_freq),
    std::bind(&BearingNode::timerCallback, this)
  );

  RCLCPP_INFO(this->get_logger(), "Node started");
}

void BearingNode::poseUpdateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  auto odom_pose = *msg;
  double x = odom_pose.pose.pose.position.x;
  double y = odom_pose.pose.pose.position.y;
  double z = odom_pose.pose.pose.position.z;
  double x = odom_pose.pose.pose.orientation.x;
  double y = odom_pose.pose.pose.orientation.y;
  double z = odom_pose.pose.pose.orientation.z;
  double w = odom_pose.pose.pose.orientation.w;
  robot_pitch = std::atan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z);
  robot_x = x;
  robot_y = y;
}

void BearingNode::timerCallback() {
  // Radians
  geometry_msgs::msg::PoseWithCovarianceStamped bearing;
  // std_msgs::msg::Float32 bearing;

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

    // Calculate tag to base link
    // tf_map_to_link = tf_map_to_tag.inverse() * tf_tag_to_link.inverse();

    // Get x and y location of the tag relative to the base link
    double link_to_tag_x = tag_to_link.transform.translation.x * x*std::cos(robot_pitch) + 
                            tag_to_link.transform.translation.z * z*std::sin(robot_pitch);
    double link_to_tag_y = tag_to_link.transform.translation.y;

    if (robot_x == -1) {4
      RCLCPP_INFO(this->get_logger(), "No robot position from EKF yet");
      return;
    }

    double yaw;
    switch(i) {
      case 0:
        yaw = M_PI/2 + std::tan((robot_x - tag_x[i])/(tag_y[i] - robot_y));
        break;
      case 1:
        yaw = std::tan((tag_y[i] - robot_y)/(tag_x[i] - robot_x));
        break;
      case 2:
        yaw = -M_PI/2 + std::tan((tag_x[i] - robot_x)/(robot_y - tag_y[i]));
        break;
      case 3:
        yaw = M_PI + std::tan((robot_y - tag_y[i])/(robot_x - tag_x[i]));
        break;

      default:
        // code block
    double camera_tag_yaw_offset = std::tan(link_to_tag_x/link_to_tag_y);

    yaw += camera_tag_yaw_offset;
}

    // Get Yaw
    // double roll, pitch, yaw;
    // tf_map_to_link.getBasis().getRPY(roll, pitch, yaw);

    // Calculate and append bearing
    bearings.push_back(yaw);

    // Get cartesian distance from camera to tag for weighting
    //   tag_dist_to_cam.push_back(sqrt(pow(cam_to_tag.transform.translation.x,2) +
    //     pow(cam_to_tag.transform.translation.y,2) +
    //     pow(cam_to_tag.transform.translation.z,2)));
  }
  if (bearings.size() > 0) {
    // Find the best tag to get angle from
    double best_bearing = bearings[0];
    // double min_dist = tag_dist_to_cam[0];
    double min_offset_from_90s = M_PI/2;
    for (int j = 0; j < static_cast<int>(bearings.size()); j++) {
      double x_angle = std::cos(bearings[j]);
      double y_angle = std::sin(bearings[j]);
      double offset_from_90 = std::min(std::min(std::pow(x_angle - angle_90_inc_x[0], 2) + std::pow(y_angle - angle_90_inc_y[0], 2),
                                      std::pow(x_angle - angle_90_inc_x[1], 2) + std::pow(y_angle - angle_90_inc_y[1], 2)),
                                      std::min(std::pow(x_angle - angle_90_inc_x[2], 2) + std::pow(y_angle - angle_90_inc_y[2], 2),
                                      std::pow(x_angle - angle_90_inc_x[3], 2) + std::pow(y_angle - angle_90_inc_y[3], 2)));
      if (offset_from_90 < min_offset_from_90s) {
        best_bearing = bearings[j];
        min_offset_from_90s = offset_from_90;
      }
    }
    
    // Variables for average bearing over the buffer length
    double avg_bearing;
    std::vector<double> sorted_bearings;
    double sum_y_angle = 0.0;
    double sum_x_angle = 0.0;
    
    // Instead of averaging the bearing, sum the unit vectors
    // this->rolling_bearing.push_back(best_bearing);
    this->rolling_sin.push_back(std::sin(best_bearing));
    this->rolling_cos.push_back(std::cos(best_bearing));

    // Erase the oldest bearing if the buffer is longer than the limit
    if (this->rolling_sin.size() > (long unsigned int)rolling_avg_buffer) {
      // this->rolling_bearing.erase(rolling_bearing.begin());
      this->rolling_sin.erase(rolling_sin.begin());
      this->rolling_cos.erase(rolling_cos.begin());

    }

    // if ((int)rolling_avg_buffer > 2 && (int)this->rolling_bearing.size() == rolling_avg_buffer) {
    //   std::vector<int> indices(rolling_bearing.size());
    //   std::iota(indices.begin(), indices.end(), 0);
    //   std::sort(indices.begin(), indices.end(),
    //             [&](int A, int B) -> bool {
    //                 return rolling_bearing[A] < rolling_bearing[B];
    //             });

    //   // Average the unit vectors and get angle
    //   for (int i = 0; i < (int)rolling_avg_buffer - 4; i++) {
    //     sum_y_angle += this->rolling_sin[indices[i+2]];
    //     sum_x_angle += this->rolling_cos[indices[i+2]];
    //   }
    // }
    // else {
    sum_y_angle = std::accumulate(this->rolling_sin.begin(), this->rolling_sin.end(), 0.0);
     sum_x_angle = std::accumulate(this->rolling_cos.begin(), this->rolling_cos.end(), 0.0);
    // }
    avg_bearing = std::atan2(sum_y_angle, sum_x_angle);
    
    // Set the PoseWithCovarianceStamped message's position to 0
    bearing.pose.pose.position.x = 0.0;
    bearing.pose.pose.position.y = 0.0;
    bearing.pose.pose.position.z = 0.0;

    // Convert yaw back into a quarternion for orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, avg_bearing);

    // Set the quarternion in the message
    bearing.pose.pose.orientation.x = q.x();
    bearing.pose.pose.orientation.y = q.y();
    bearing.pose.pose.orientation.z = q.z();
    bearing.pose.pose.orientation.w = q.w();

    // Set header and frame name
    bearing.header.stamp = this->get_clock()->now();
    bearing.header.frame_id = tf_map_frame;

    // Set covariance
    bearing.pose.covariance =  {0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                                0.0,  0.0,  0.0,  0.0,  0.0,  this->bearing_covariance};

    // Publish the estimated bearing  
    bearing_pub_->publish(bearing);

    // Test for bearing in degrees
    // bearing.data = avg_bearing * 180.0 / 3.14159;
    // bearing_pub_->publish(bearing);
  }
  else {
    RCLCPP_WARN(this->get_logger(), "No bearing tags were detected.");
  }
}

}  // namespace teleop
}  // namespace cg
