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

cg_msgs::msg::Point2D transformPointLocalToGlobal(const cg_msgs::msg::Point2D &global_pt, const cg_msgs::msg::Pose2D &local_frame) {

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
double smallest_angle_difference(double angle1, double angle2) {
  // Normalize angles to [0, 2*pi]
  double normalized_angle1 = std::fmod(angle1, 2.0 * M_PI);
  double normalized_angle2 = std::fmod(angle2, 2.0 * M_PI);

  // Normalize absolute difference to [0, 2*pi]
  double diff_head = std::fmod(fabs(normalized_angle1 - normalized_angle2), 2.0 * M_PI);

  // Use smaller overall angle, i.e. in [0, pi]
  if (diff_head > (2.0 * M_PI)) {
      diff_head = fabs(diff_head - (2.0 * M_PI));
  }
  return diff_head;
}

bool samePoseWithinThresh(const cg_msgs::msg::Pose2D &pose1, const cg_msgs::msg::Pose2D &pose2, const float thresh_pos, const double thresh_head) {
  // Calculate position difference
  float diff_pos = euclidean_distance(pose1.pt, pose2.pt);

  // Calculate heading difference
  double diff_head = smallest_angle_difference(pose1.yaw, pose2.yaw);

  if (diff_pos <= thresh_pos && diff_head <= thresh_head) {
    return true;
  }
  return false;
}

} // planning namespace
} // cg namespace
