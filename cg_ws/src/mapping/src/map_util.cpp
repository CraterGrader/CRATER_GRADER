#include "mapping/site_map.hpp"

namespace cg {
namespace mapping {

// Note: width, height := number of cells (N x M), resolution := meter / cell (square), x & y := continuous position

bool pointInMap(float x, float y, float width, float height, float resolution){
    // Something like:
    if (0 < x && x < width * resolution && 0 < y && y < height * resolution)
        return true;
    return false;
}

int coordsToCellIndex(float x, float y, float width, float resolution){
    // Something like:
    // TODO: verify row major order
    return int(x / resolution) + int( (y / resolution) * (width / resolution) );
}


} // mapping namespace
} // cg namespace
