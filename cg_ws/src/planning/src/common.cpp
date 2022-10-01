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
  pose.yaw = yaw;
  return pose;
}

float euclidean_distance(const cg_msgs::msg::Point2D& pt1, const cg_msgs::msg::Point2D& pt2) {
  return std::sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y));
}

float rad2deg(float rad) {
  return 180 * (rad / M_PI);
}

float deg2rad(float deg) {
  return M_PI * (deg / 180);
}

cg_msgs::msg::Point2D transformPoint(const cg_msgs::msg::Point2D &source_pt, const cg_msgs::msg::Pose2D &pose) {

  // Generate Rotation matrix from pose
  Eigen::Matrix2d pose_mat;
  pose_mat << cos(pose.yaw), -sin(pose.yaw), 0, pose.pt.x,
        sin(pose.yaw), cos(pose.yaw), 0, pose.pt.y,
        0, 0, 1, 0,
        0, 0, 0, 1;

  Eigen::Vector2d source_vec(source_pt.x, source_pt.y, 0, 1);  
  Eigen::Vector2d transformed_vec = pose_mat * source_vec;

  return create_point2d(transformed_vec.coeff(0), transformed_vec.coeff(1));
}

} // planning namespace
} // cg namespace
