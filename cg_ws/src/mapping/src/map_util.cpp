#include "mapping/map_util.hpp"

namespace cg {
namespace mapping {

bool pointInMap(float x, float y, size_t width, size_t height, float resolution){
    // width, height := number of cells (N x M) 
    // resolution := meter / cell (square) 
    // x & y := continuous position
    // position must be in site_map frame 
    return (0 <= x && x < width * resolution && 0 <= y && y < height * resolution);
}

bool indexInMap(size_t x, size_t y, size_t width, size_t height){
    // width, height := number of cells (N x M) 
    // resolution := meter / cell (square) 
    // x & y := discrete position indices
    // position must be in site_map frame 

    // Unsigned ints, always >= 0
    return (x < width && y < height);
}

bool heightInRange(float height, float minHeight, float maxHeight){
    // height: z-height, meters 
    // minHeight: minimum height, meters everything below removed
    // maxHeight: maximum height, meters everything above removed
    return (height >= minHeight && height <= maxHeight); 
}

int pointToDiscreteCoord(float pos, float resolution){
  // for a continous point along an axis, return its int location which should align with map
  // pos, location in dimension, meters
  // resolution := meter / cell (square), meter
  return static_cast<int>(floor(pos/resolution));
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
    size_t x_discrete = cg::mapping::pointToDiscreteCoord(x, resolution);
    size_t y_discrete = cg::mapping::pointToDiscreteCoord(y, resolution);
    return discreteCoordsToCellIndex(x_discrete, y_discrete, width);
}

float convertMaptoSiteMapFrame(float pos, float offset){
    // for a given axis, e.g. x-axis, use offset to convert from map -> site_map frame 
    // assumed no roation 
    // pos: position along an axis, meters 
    // offset: site_map origin offset from map origin 
    return pos-offset;
}

float rad2deg(float rad) {
  return 180 * (rad / M_PI);
}

float deg2rad(float deg) {
  return M_PI * (deg / 180);
}

cg_msgs::msg::Point2D transformPoint(const cg_msgs::msg::Point2D &source_pt, const cg_msgs::msg::Pose2D &source_frame_rel_dest) {

    // Source frame is relative to destination frame
    // Generate Transformation matrix from pose, based on yaw z-rotation  
    Eigen::Matrix4f pose_mat;
    pose_mat << std::cos(source_frame_rel_dest.yaw), -std::sin(source_frame_rel_dest.yaw), 0, source_frame_rel_dest.pt.x,
        std::sin(source_frame_rel_dest.yaw), std::cos(source_frame_rel_dest.yaw), 0, source_frame_rel_dest.pt.y,
        0, 0, 1, 0,
        0, 0, 0, 1;

    Eigen::Vector4f source_vec;
    source_vec << source_pt.x, source_pt.y, 0, 1;
    
    Eigen::Vector4f transformed_vec = pose_mat * source_vec;

    cg_msgs::msg::Point2D dest_pt;
    dest_pt.x = transformed_vec.coeff(0);
    dest_pt.y = transformed_vec.coeff(1);
    return dest_pt;
}

/**
 * @brief
 *
 * @param map1
 * @param map2
 * @param threshold
 * @return true
 * @return false
 */
bool mapSimilarityWithinThreshold(const std::vector<float> &map1, const std::vector<float> &map2, float threshold){
    // compare each cell check within threshold
    for (size_t i=0; i<(map1.size()); i++){
        if (fabs(map1[i] - map2[i]) > threshold){
            return false;
        }
    }
    return true;
}

bool file_exists(const std::string& filepath) {
  if (FILE *file = fopen(filepath.c_str(), "r")) {
    fclose(file);
    return true;
  } else {
    return false;
  }   
}

} // mapping namespace
} // cg namespace
