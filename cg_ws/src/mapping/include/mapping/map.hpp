#ifndef MAPPING__MAP_HPP
#define MAPPING__MAP_HPP

#include <vector> // holding map data
#include <string> // for frame name
#include <cassert> // checking for valid input
#include <cmath> // math.floor function
#include <mapping/map_util.hpp> // helper functions
#include <cg_msgs/msg/point2_d.hpp> // use of cg:msgs::Point2D

namespace cg {
namespace mapping {

template <class T>
class Map {
  public:
    // Constructor with empty data
    Map(size_t height, size_t width, float resolution);
    // Constructor with data
    Map(size_t height, size_t width, float resolution, std::vector<T> cell_data_);

    // Helpers
    bool validPoint(const cg_msgs::msg::Point2D &pt) const;
    size_t continousCoordsToCellIndex(const cg_msgs::msg::Point2D &pt) const;
    cg_msgs::msg::Point2D indexToContinuousCoords(size_t idx) const; 

    T getDataAtIdx(size_t idx) const;
    bool validIdx(size_t idx) const;

    // getter functions 
    float getResolution() const {return resolution_;}
    size_t getWidth() const {return width_;}
    size_t getHeight() const {return height_;}
    std::vector<T> getCellData() const {return cell_data_;}

    // setter functions
    void setCellData(std::vector<T> input_data) {cell_data_ = input_data;}
    void updateCellElement(T new_element, size_t idx);

  private:
    // attributes
    size_t height_;    // the vertical number of cells in the map
    size_t width_;     // the horizontal number of cells in the map
    float resolution_; // the resolution of each grid cell 
    std::vector<T> cell_data_; // the actual map data; should have height_ * width_ = cell_data_.size()
};

template <class T>
Map<T>::Map(size_t height, size_t width, float resolution) {
  height_ = height;
  width_ = width;
  resolution_ = resolution;
}

template <class T>
Map<T>::Map(size_t height, size_t width, float resolution, std::vector<T> cell_data)
{
  height_ = height;
  width_ = width;
  resolution_ = resolution;
  cell_data_ = cell_data;
}

template <class T>
bool Map<T>::validPoint(const cg_msgs::msg::Point2D &pt) const {
  return cg::mapping::pointInMap(pt.x, pt.y, width_, height_, resolution_);
}

template <class T>
size_t Map<T>::continousCoordsToCellIndex(const cg_msgs::msg::Point2D &pt) const {
  size_t idx = cg::mapping::continousCoordsToCellIndex(pt.x, pt.y, width_, resolution_);
  return idx;
}

template <class T>
cg_msgs::msg::Point2D Map<T>::indexToContinuousCoords(size_t idx) const {
  cg_msgs::msg::Point2D pt;
  pt.x = resolution_* (idx % width_);
  pt.y = resolution_ * std::floor(idx/width_);
  return pt;
}


template <class T>
T Map<T>::getDataAtIdx(size_t idx) const {
  assert(validIdx(idx));
  return cell_data_.at(idx);
}

template <class T>
bool Map<T>::validIdx(size_t idx) const {
  return idx < (height_ * width_) && idx < cell_data_.size();
}

} // mapping namespace
} // cg namespace

#endif // MAPPING__MAP_HPP
