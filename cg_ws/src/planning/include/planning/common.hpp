#ifndef PLANNING__COMMON_HPP
#define PLANNING__COMMON_HPP

#include <cmath> // sqrt
#include <cg_msgs/msg/point2_d.hpp>
#include <cg_msgs/msg/pose2_d.hpp>
#include <cg_msgs/msg/trajectory.hpp>

namespace cg {
namespace planning {

// Functions
cg_msgs::msg::Point2D create_point2d(const double& x, const double& y);
cg_msgs::msg::Pose2D create_pose2d(const double& x, const double& y, const double &yaw);

float euclidean_distance(const cg_msgs::msg::Point2D& pt1, const cg_msgs::msg::Point2D& pt2);

} // planning namespace
} // cg namespace

#endif // PLANNING__COMMON_HPP
