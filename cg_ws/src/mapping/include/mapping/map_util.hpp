#ifndef MAPPING__MAP_UTIL_HPP
#define MAPPING__MAP_UTIL_HPP

#include <cmath> // floor

namespace cg {
namespace mapping {

bool pointInMap(float x, float y, size_t width, size_t height, float resolution);
bool indexInMap(size_t x, size_t y, size_t width, size_t height);
bool heightInRange(float height, float minHeight, float maxHeight);
int pointToDiscreteCoord(float pos, float resolution);
int discreteCoordsToCellIndex(size_t x, size_t y, size_t width);
int continousCoordsToCellIndex(float x, float y, size_t width, float resolution);
float convertMaptoSiteMapFrame(float pos, float offset);

} // mapping namespace
} // cg namespace

#endif // MAPPING__MAP_UTIL_HPP
