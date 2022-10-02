#ifndef MAPPING__MAP_HPP
#define MAPPING__MAP_HPP

#include <vector> // holding map data
#include <cassert> // checking for valid input
#include <mapping/map_util.hpp> // helper functions

namespace cg {
namespace mapping {

class Map {
  public:
    // Constructor with empty data
    Map(size_t height, size_t width, float resolution);
    // Constructor with data
    Map(size_t height, size_t width, float resolution, std::vector<float> cell_data);

    // Helpers
    bool pointInMap(float x, float y);
    size_t continousCoordsToCellIndex(float x, float y);
    float getDataAtIdx(size_t idx);
    bool validIdx(size_t idx);
    float getResolution(){return resolution_;}

  private:
    // attributes
    size_t height_;    // the vertical number of cells in the map
    size_t width_;     // the horizontal number of cells in the map
    float resolution_; // the resolution of each grid cell 
    std::vector<float> cell_data_; // the actual map data; should have height_ * width_ = cell_data_.size()
};

} // mapping namespace
} // cg namespace

#endif // MAPPING__MAP_HPP
