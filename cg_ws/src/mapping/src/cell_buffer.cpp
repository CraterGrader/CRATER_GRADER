#include "mapping/cell_buffer.hpp"

namespace cg {
namespace mapping {

CellBuffer::CellBuffer(float unseenGridHeight){
      height_ = unseenGridHeight;
      doesBufferHaveNewData_ = false;
}

void CellBuffer::addPoint(indexPoint pt){
    // update the height based on incoming point
    height_ = pt.z;
    // let other maps know the buffer has new data here
    bufferHasNewData();
}

} // mapping namespace
} // cg namespace
