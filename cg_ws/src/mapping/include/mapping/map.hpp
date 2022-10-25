#ifndef MAPPING__MAP_HPP
#define MAPPING__MAP_HPP

#include <vector> // holding map data, and map i/o
#include <string> // for frame name, and map i/o
#include <iostream> // map i/o
#include <fstream> // map i/o
#include <cassert> // checking for valid input
#include <cmath> // math.floor function
#include <mapping/map_util.hpp> // helper functions
#include <cg_msgs/msg/point2_d.hpp> // use of cg:msgs::Point2D

namespace cg {
namespace mapping {

template <class T>
class Map {
  public:
    // Empty constructor
    Map(){};
    // Constructor with empty data
    Map(size_t height, size_t width, float resolution);
    // Constructor with data
    Map(size_t height, size_t width, float resolution, std::vector<T> cell_data_);

    // Helpers
    size_t continousCoordsToCellIndex(const cg_msgs::msg::Point2D &pt) const;
    cg_msgs::msg::Point2D indexToContinuousCoords(size_t idx) const; 

    // Checkers
    bool validIdx(size_t idx) const;
    bool validPoint(const cg_msgs::msg::Point2D &pt) const;

    // Getter functions 
    T getDataAtIdx(size_t idx) const;
    float getResolution() const {return resolution_;}
    size_t getWidth() const {return width_;}
    size_t getHeight() const {return height_;}
    std::vector<T> getCellData() const {return cell_data_;}

    // Setter functions
    bool setCellData(std::vector<T> input_data);
    void updateCellElement(T new_element, size_t idx);
    void updateDimensions(size_t height, size_t width, float resolution);

    /**
     * @brief Helper to write map data to a filepath
     * Notes:
     * - File name will be appended with '_repeat' if the filepath already exists
     * - CSV file format is structured as a single column with N data entries starting in fourth entry (e.g. index 3). Spaces in and/or after the data are ignored. For example:
     *    height
     *    width
     *    resolution
     *    data1
     *    data2
     *      :
     *    dataN
     *
     * @param filepath The filepath to write to
     */
    bool write_map_to_file(const std::string &filepath);

    /**
     * @brief Helper to read map data from a filepath
     * Notes:
     * - Will return without modifying if: file doesn't exist OR if height*width dimensions do not match N
     * - CSV file format is structured as a single column with N data entries starting in fourth entry (e.g. index 3). Spaces in and/or after the data are ignored. For example:
     *    height
     *    width
     *    resolution
     *    data1
     *    data2
     *      :
     *    dataN
     *
     * @param filepath The filepath to read from
     */
    bool load_map_from_file(const std::string &filepath);

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
void Map<T>::updateDimensions(size_t height, size_t width, float resolution)
{
  height_ = height;
  width_ = width;
  resolution_ = resolution;
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
  bool idx_in_bounds = idx < (height_ * width_);
  bool data_exists_at_idx = idx < cell_data_.size();
  return idx_in_bounds && data_exists_at_idx;
}

template <class T>
bool Map<T>::setCellData(std::vector<T> input_data) {
  // Don't update data if the dimensions are wrong
  if (input_data.size() != (height_ * width_)) {
    return false;
  }
  // Otherwise, ok to update
  cell_data_ = input_data;
  return true;
}

template <class T>
bool Map<T>::write_map_to_file(const std::string& filepath) {
  // Modify name if file already exists
  std::string write_filepath = filepath;
  while (cg::mapping::file_exists(write_filepath)) {
    std::string::size_type pos = write_filepath.find('.');
    write_filepath = write_filepath.substr(0, pos) + "_repeat" + ".csv";
  }

  // Open file for writing
  std::ofstream myfile;
  myfile.open(write_filepath);
  
  if (!myfile) {
    // There was a problem opening the file
    std::cout << "[MAP I/O ERROR] Could not write to file: " << filepath << std::endl;
    return false;
  }

  // Write the data
  myfile << height_ << std::endl;
  myfile << width_ << std::endl;
  myfile << resolution_ << std::endl;
  for (float val : cell_data_) {
    myfile << val << std::endl;
  }

  // Close file
  myfile.close();
  return true;
}

template <class T>
bool Map<T>::load_map_from_file(const std::string& filepath){

  // Read lines
  std::vector<std::string> content;
  std::string line;
  std::fstream file(filepath, std::ios::in);
  if (file.is_open()) {
    while (getline(file, line)) {
      content.push_back(line);
    }
  }
  else {
    std::cout << "[MAP I/O ERROR] Could not open the file: " << filepath << std::endl;
    return false;
  }
  file.close();

  // Pack data
  size_t csv_height = static_cast<size_t>(stoi(content[0]));
  size_t csv_width = static_cast<size_t>(stoi(content[1]));
  float csv_resolution = static_cast<float>(stof(content[2]));

  std::vector<float> csv_data;
  for (unsigned int i = 3; i < content.size(); i++) {
    if (content[i].length() > 0) {
      csv_data.push_back(static_cast<float>(stof(content[i])));
    }
  }

  if (csv_height * csv_width != csv_data.size()) {
    std::cout << "[MAP I/O ERROR] Dimensions do not align for: csv height = " << csv_height << ", csv width = " << csv_width << ", csv data size = " << csv_data.size() << " with file: " << filepath << std::endl;
    return false;
  }

  // Set data
  height_ = csv_height;
  width_ = csv_width;
  resolution_ = csv_resolution;
  cell_data_ = csv_data;
  return true;
}

} // mapping namespace
} // cg namespace

#endif // MAPPING__MAP_HPP
