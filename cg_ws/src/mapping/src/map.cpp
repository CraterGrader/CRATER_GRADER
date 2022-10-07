#include <mapping/map.hpp>

namespace cg {
namespace mapping {

Map::Map(size_t height, size_t width, float resolution) {
  height_ = height;
  width_ = width;
  resolution_ = resolution;
}

Map::Map(size_t height, size_t width, float resolution, std::vector<float> cell_data)
{
  height_ = height;
  width_ = width;
  resolution_ = resolution;
  cell_data_ = cell_data;
}

bool Map::pointInMap(float x, float y) const {
  return cg::mapping::pointInMap(x, y, width_, height_, resolution_);
}

size_t Map::continousCoordsToCellIndex(float x, float y) const {
  size_t idx = cg::mapping::continousCoordsToCellIndex(x, y, width_, resolution_);
  assert(validIdx(idx));
  return idx;
}

float Map::getDataAtIdx(size_t idx) const {
  assert(validIdx(idx));
  return cell_data_.at(idx);
}

bool Map::validIdx(size_t idx) const {
  return idx < (height_ * width_) && idx < cell_data_.size();
}

} // mapping namespace
} // cg namespace
