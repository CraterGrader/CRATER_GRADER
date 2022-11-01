#ifndef PLANNING__COMMON_HPP
#define PLANNING__COMMON_HPP

#include <cmath> // sqrt, fmod
#include <Eigen/Dense> // matrix multiplication
#include <Eigen/Geometry> // homogenous transforms
#include <cg_msgs/msg/point2_d.hpp>
#include <cg_msgs/msg/pose2_d.hpp>
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <limits>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace cg {
namespace planning {

// Functions
cg_msgs::msg::Point2D create_point2d(const double& x, const double& y);
cg_msgs::msg::Pose2D create_pose2d(const double& x, const double& y, const double &yaw);

float euclidean_distance(const cg_msgs::msg::Point2D& pt1, const cg_msgs::msg::Point2D& pt2);

float rad2deg(float rad);
float deg2rad(float deg);

// float angle_difference(const float angle_1, const float angle_2);

cg_msgs::msg::Point2D transformPointGlobalToLocal(
    const cg_msgs::msg::Point2D &global_pt,
    const cg_msgs::msg::Pose2D &local_frame);

cg_msgs::msg::Point2D transformPoint(
  const cg_msgs::msg::Point2D &source_pt, 
  const cg_msgs::msg::Pose2D &pose);

cg_msgs::msg::Pose2D transformPose(
  const cg_msgs::msg::Pose2D &source_pose, 
  const cg_msgs::msg::Pose2D &transforming_pose);

// Find smallest difference between two angles, all units in radians
double smallest_angle_difference_signed(double angle1, double angle2);

// Checks if trajectory_end_pose is within distance threshold of goal_pose
bool samePoseWithinThresh(
    const cg_msgs::msg::Pose2D &pose1, const cg_msgs::msg::Pose2D &pose2,
    const float thresh_pos, const double thresh_head);

// checks what the new closest index should be on a trajectory based on localization
int getClosestTrajIndex(const cg_msgs::msg::Trajectory &target_trajectory, const nav_msgs::msg::Odometry &current_state, int prev_traj_idx);

} // planning namespace
} // cg namespace

#endif // PLANNING__COMMON_HPP
