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

int coordsToCellIndex(float x, float y, size_t width, float resolution){
    // width := number of cells (N x M) 
    // resolution := meter / cell (square) 
    // x & y := continuous position
    // position must be in site_map frame 
    return int(x/resolution) + int((y/resolution)*(width/resolution));
}

} // mapping namespace
} // cg namespace
