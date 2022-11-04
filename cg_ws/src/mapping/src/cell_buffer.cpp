#include "mapping/cell_buffer.hpp"

namespace cg {
namespace mapping {

CellBuffer::CellBuffer(float unseenGridHeight) {
  (void)unseenGridHeight;
  height_ = 0.0f;
  doesBufferHaveNewData_ = false;
}

void CellBuffer::bufferHasBeenUpdated() {
  doesBufferHaveNewData_ = false;
  height_ = 0.0f;
}

void CellBuffer::offset_height(float offset) { height_ -= offset; }

void CellBuffer::addPoint(indexPoint pt) {
  // update the height based on incoming point
  // do a check to see if the incoming point is an extreema
  if (std::fabs(pt.z) > std::fabs(height_)) {
    height_ = pt.z;
  }
  // let other maps know the buffer has new data here
  bufferHasNewData();
}

} // namespace mapping
} // namespace cg
