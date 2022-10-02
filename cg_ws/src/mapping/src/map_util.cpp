#include "mapping/site_map.hpp"

namespace cg {
namespace mapping {

bool pointInMap(float x, float y, size_t width, size_t height, float resolution){
    // width, height := number of cells (N x M) 
    // resolution := meter / cell (square) 
    // x & y := continuous position
    // position must be in site_map frame 
    if (0 < x && x < width * resolution && 0 < y && y < height * resolution){
        return true;
    }
    return false;
}

bool indexInMap(size_t x, size_t y, size_t width, size_t height){
    // width, height := number of cells (N x M) 
    // resolution := meter / cell (square) 
    // x & y := continuous position
    // position must be in site_map frame 
    if (0 < x && x < width && 0 < y && y < height){
        return true;
    }
    return false;
}

bool heightInRange(float height, float minHeight, float maxHeight){
    // height: z-height, meters 
    // minHeight: minimum height, meters everything below removed
    // maxHeight: maximum height, meters everything above removed
    if (height > minHeight && height < maxHeight){
        return true;
    }
    return false;
}

int binLength(float pos, float resolution){
  // for a continous point along an axis, return its int location which should align with map
  // pos, location in dimension, meters
  // resolution := meter / cell (square), meter
  return (int) floor(pos/resolution);
}

int discreteCoordsToCellIndex(size_t x, size_t y, size_t width){
    // x: map location in x, unitless 
    // y: map location in y, unitless
    // width: width of map, in number of cells 
    return x + (y * width);
}

int continousCoordsToCellIndex(float x, float y, size_t width, float resolution){
    // width := number of cells (M), unitless
    // resolution := meter / cell (square), meter
    // x & y := continuous position, meters in site_map frame 
    size_t x_discrete = cg::mapping::binLength(x, resolution);
    size_t y_discrete = cg::mapping::binLength(y, resolution);
    return discreteCoordsToCellIndex(x_discrete, y_discrete, width);
}

float convertMaptoSiteMapFrame(float pos, float offset){
    // for a given axis, e.g. x-axis, use offset to convert from map -> site_map frame 
    // assumed no roation 
    // pos: position along an axis, meters 
    // offset: site_map origin offset from map origin 
    return (pos-offset);
}

} // mapping namespace
} // cg namespace
