#ifndef MAPPING__MAP_UTIL_HPP
#define MAPPING__MAP_UTIL_HPP

#include <cmath> // floor
#include <Eigen/Dense> // matrix multiplication
#include <cg_msgs/msg/point2_d.hpp> // use of cg:msgs::Point2D
#include <cg_msgs/msg/pose2_d.hpp> // use of cg:msgs::Pose2D
// #include <mapping/map.hpp>

namespace cg {
namespace mapping {

bool pointInMap(float x, float y, size_t width, size_t height, float resolution);
bool indexInMap(size_t x, size_t y, size_t width, size_t height);
bool heightInRange(float height, float minHeight, float maxHeight);
int pointToDiscreteCoord(float pos, float resolution);
int discreteCoordsToCellIndex(size_t x, size_t y, size_t width);
int continousCoordsToCellIndex(float x, float y, size_t width, float resolution);
float convertMaptoSiteMapFrame(float pos, float offset);
cg_msgs::msg::Point2D transformPoint(const cg_msgs::msg::Point2D &source_pt, const cg_msgs::msg::Pose2D &dest_frame);

// bool mapVsMapThreshold(const cg::mapping::Map<float> &map1, const cg::mapping::Map<float> &map2, float threshold);

// TODO: add replanTransport(map, desired map, threshold)

float rad2deg(float rad);
float deg2rad(float deg);

} // mapping namespace
} // cg namespace

#endif // MAPPING__MAP_UTIL_HPP
