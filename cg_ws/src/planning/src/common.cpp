#include "planning/common.hpp"

namespace cg {
namespace planning {

float euclidean_distance(Point pt1, Point pt2) {
  return std::sqrt((pt2.x - pt1.x) * (pt2.x - pt1.x) + (pt2.y - pt1.y) * (pt2.y - pt1.y));
}

} // planning namespace
} // cg namespace
