#include "planning/common.hpp"

namespace cg {
namespace planning {

cg_msgs::msg::Point2D create_point2d(const double& x, const double& y) {
  cg_msgs::msg::Point2D pt;
  pt.x = x;
  pt.y = y;
  return pt;
}

cg_msgs::msg::Pose2D create_pose2d(const double& x, const double& y, const double& yaw) {
  cg_msgs::msg::Pose2D pose;
  pose.pt = create_point2d(x, y);

  // Normalize yaw within 0,2*M_PI for distance thresholding and debugging
  float norm_yaw = std::fmod(yaw, 2.0*M_PI);
  if (norm_yaw < 0.0f) norm_yaw += 2.0*M_PI;

  pose.yaw = norm_yaw;
  return pose;
}

float euclidean_distance(const cg_msgs::msg::Point2D& pt1, const cg_msgs::msg::Point2D& pt2) {
  return std::sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y));
}

// float angle_difference(const float angle_1, const float angle_2) {
//   float angle_difference = std::fmod((angle_1 - angle_2) + M_PI, 2.0*M_PI) - M_PI;
//   return angle_difference;
// }

float rad2deg(float rad) {
  return 180 * (rad / M_PI);
}

float deg2rad(float deg) {
  return M_PI * (deg / 180);
}


/**
 * @brief This transforms a point into a local frame, where both point and local frame are relative to the global frame.
 * 
 * @param global_pt The point to be transformed to the local frame, relative to global frame
 * @param local_frame The frame to transform the point into, should be relative to global frame
 * @return cg_msgs::msg::Point2D 
 */
cg_msgs::msg::Point2D transformPointGlobalToLocal(const cg_msgs::msg::Point2D &global_pt, const cg_msgs::msg::Pose2D &local_frame) {

  // Create local frame matrix
  Eigen::Vector2f trans{local_frame.pt.x, local_frame.pt.y};
  Eigen::Transform<float, 2, Eigen::Affine> T;
  T = Eigen::Translation<float, 2>(trans);

  T.rotate(Eigen::Rotation2Df(local_frame.yaw));

  // Invert
  T = T.inverse();

  Eigen::Matrix3f mat;
  mat = T.matrix();

  // // Create vector
  Eigen::Vector3f global_vec{static_cast<float>(global_pt.x), static_cast<float>(global_pt.y), 1.0f};

  // Multiply to get point transformed to local frame
  Eigen::Vector3f local_pt;
  local_pt = mat * global_vec;
  return create_point2d(local_pt.coeff(0), local_pt.coeff(1));
  // return create_point2d(0.0, 0.0);
}

cg_msgs::msg::Point2D transformPoint(const cg_msgs::msg::Point2D &source_pt, const cg_msgs::msg::Pose2D &pose) {

  // Generate Transformation matrix from pose, based on yaw z-rotation
  Eigen::Matrix3d pose_mat;
  pose_mat << cos(pose.yaw), -sin(pose.yaw), pose.pt.x,
        sin(pose.yaw), cos(pose.yaw), pose.pt.y,
        0, 0, 1;
  Eigen::Vector3d source_vec;
  source_vec << source_pt.x, source_pt.y, 1;
    
  Eigen::Vector3d transformed_vec = pose_mat * source_vec;

  return create_point2d(transformed_vec.coeff(0), transformed_vec.coeff(1));
}

cg_msgs::msg::Pose2D transformPose(
  const cg_msgs::msg::Pose2D &source_pose, 
  const cg_msgs::msg::Pose2D &transforming_pose) {

  cg_msgs::msg::Point2D transformed_point = transformPoint(source_pose.pt, transforming_pose);
  return create_pose2d(transformed_point.x, transformed_point.y, source_pose.yaw + transforming_pose.yaw);
}

// Find smallest difference between two angles, all units in radians
double smallest_angle_difference_signed(double angle1, double angle2) {
  // Normalize angles to [0, 2*pi]
  float normalized_angle1 = std::fmod(static_cast<float>(angle1), 2.0 * M_PI);
  float normalized_angle2 = std::fmod(static_cast<float>(angle2), 2.0 * M_PI);

  // Normalize absolute difference to [0, 2*pi]
  float diff_head = std::fmod(fabs(normalized_angle1 - normalized_angle2), 2.0 * M_PI);

  // Use smaller overall angle, i.e. in [0, pi]
  if (diff_head > (M_PI)) {
    diff_head = fabs((2.0 * M_PI) - diff_head);
  }

  // Check z-component of 3d cross product of headings; 2d case is identical to a trig identity!
  float z_component = sin(normalized_angle1) * cos(normalized_angle2) - cos(normalized_angle1) * sin(normalized_angle2);
  // float sin_identity = sin(normalized_angle1 - normalized_angle2); // Trig identity!

  // Check cross product result to get signed angle
  if (z_component < 0) {
    diff_head = -diff_head;
  }

  return static_cast<double>(diff_head);
}

bool samePoseWithinThresh(const cg_msgs::msg::Pose2D &pose1, const cg_msgs::msg::Pose2D &pose2, const float thresh_pos, const double thresh_head) {
  // Calculate position difference
  float diff_pos = euclidean_distance(pose1.pt, pose2.pt);

  // Calculate heading difference
  double diff_head = static_cast<double>(std::fabs(static_cast<float>(smallest_angle_difference_signed(pose1.yaw, pose2.yaw))));

  if (diff_pos <= thresh_pos && diff_head <= thresh_head) {
    return true;
  }
  return false;
}

int getClosestTrajIndex(const cg_msgs::msg::Trajectory &target_trajectory, const nav_msgs::msg::Odometry &current_state, int prev_traj_idx){

  tf2::Quaternion q = tf2::Quaternion(
      current_state.pose.pose.orientation.x,
      current_state.pose.pose.orientation.y,
      current_state.pose.pose.orientation.z,
      current_state.pose.pose.orientation.w);

  cg_msgs::msg::Pose2D cur_pose = cg::planning::create_pose2d(
      current_state.pose.pose.position.x,
      current_state.pose.pose.position.y,
      tf2::getYaw(q));

  cg_msgs::msg::Point2D cur_point = cg::planning::create_point2d(
      current_state.pose.pose.position.x,
      current_state.pose.pose.position.y);

  double min_dist = std::numeric_limits<double>::infinity();
  int min_idx = -1;
  size_t search_window = 1; // Number of points to search ahead from
  for (size_t i = prev_traj_idx; i < std::min(prev_traj_idx + search_window, target_trajectory.path.size()); ++i) {
    double dist = cg::planning::euclidean_distance(target_trajectory.path[i].pt, cur_pose.pt);
    if (dist < min_dist) {
        min_dist = dist;
        min_idx = i;
    }
  }

  // check if we are at last index, if so, dont exceed 
  if (static_cast<long unsigned int>(min_idx + 1) == target_trajectory.path.size()){
    return min_idx;
  }

  // check if we are ahead of a forward drive, or behind a backup drive 
  // make local frame using path and yaw
  cg_msgs::msg::Point2D point_rel_to_traj = cg::planning::transformPointGlobalToLocal(cur_point, target_trajectory.path[min_idx]);
  // get x componant of pose in traj frame
  double x_of_point_in_traj_frame = point_rel_to_traj.x;
  // get velocity componant
  double velocity_of_traj = target_trajectory.velocity_targets[min_idx];
  // if product is positive remove the index, because we are in front of a forward drive pose, or behind a reverse drive node
  if ((velocity_of_traj * x_of_point_in_traj_frame) > 0){
    min_idx += 1;
  }
  return min_idx;
}

} // planning namespace
} // cg namespace
