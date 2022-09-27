#ifndef PLANNING__COMMON_HPP
#define PLANNING__COMMON_HPP

#include <cmath> // sqrt
#include <cg_msgs/msg/point2_d.hpp>

namespace cg {
namespace planning {

// Data types
struct Point2D {
  float x; // [m], continuous in global map frame
  float y; // [m], continuous in global map frame
};

struct Pose2D{
  Point2D pt;
  float yaw; // [rad], continuous in global map frame
};

// Functions
cg_msgs::msg::Point2D create_point2d(const double& x, const double& y);

float euclidean_distance(
  const cg_msgs::msg::Point2D& pt1,
  const cg_msgs::msg::Point2D& pt2);

} // planning namespace
} // cg namespace

#endif // PLANNING__COMMON_HPP
