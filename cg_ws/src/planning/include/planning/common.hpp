#include <cmath> // sqrt

namespace cg {
namespace planning {

// Data types
struct Point {
  float x; // [m], continuous in global map frame
  float y; // [m], continuous in global map frame
};

struct Pose {
  Point pt;
  float yaw; // [rad], continuous in global map frame
};

// Functions
float euclidean_distance(Point pt1, Point pt2);

} // planning namespace
} // cg namespace
