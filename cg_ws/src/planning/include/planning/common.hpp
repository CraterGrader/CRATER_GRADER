#include <cmath> // sqrt

namespace cg {
namespace planning {

// Data types
struct Point {
  float x;
  float y;
};

// Functions
float euclidean_distance(Point pt1, Point pt2);

} // planning namespace
} // cg namespace
