#ifndef PLANNING__COMMON_HPP
#define PLANNING__COMMON_HPP

#include <cmath> // sqrt

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
float euclidean_distance(Point2D pt1, Point2D pt2);

} // planning namespace
} // cg namespace

#endif // PLANNING__COMMON_HPP
