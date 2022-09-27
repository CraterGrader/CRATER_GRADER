#include "planning/common.hpp"

namespace cg {
namespace planning {

cg_msgs::msg::Point2D create_point2d(const double& x, const double& y) {
  cg_msgs::msg::Point2D pt;
  pt.x = x;
  pt.y = y;
  return pt;
}

float euclidean_distance(const cg_msgs::msg::Point2D& pt1, const cg_msgs::msg::Point2D& pt2) {
  return std::sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y));
}

} // planning namespace
} // cg namespace
