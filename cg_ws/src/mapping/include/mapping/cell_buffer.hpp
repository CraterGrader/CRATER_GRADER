#ifndef MAPPING__CELL_BUFFER_HPP
#define MAPPING__CELL_BUFFER_HPP

#include <mapping/index_point.hpp>
#include <cmath> // fabs

namespace cg {
namespace mapping {

class CellBuffer {
  public:
  // constructor
  CellBuffer(float unseenGridHeight);

  // methods
  void addPoint(cg::mapping::indexPoint pt);
  float getHeight(){return height_;}
  void bufferHasNewData(){doesBufferHaveNewData_ = true;}
  void bufferHasBeenUpdated();
  bool doesBufferHaveNewData(){return doesBufferHaveNewData_;}

  private:
  // attributes
  bool doesBufferHaveNewData_;
  float height_; // the buffer which stores the height

};

} // mapping namespace
} // cg namespace

#endif // MAPPING__CELL_BUFFER_HPP
