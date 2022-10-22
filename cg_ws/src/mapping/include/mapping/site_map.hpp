#ifndef MAPPING__SITE_MAP_HPP
#define MAPPING__SITE_MAP_HPP

#include <vector>
#include <math.h>
#include <assert.h>
#include <numeric>
#include <cg_msgs/msg/site_map.hpp>
#include "mapping/map_util.hpp"
#include "mapping/index_point.hpp"
#include "mapping/map_point.hpp"
#include "mapping/cell_buffer.hpp"
#include "mapping/cell_bayes.hpp"

namespace cg {
namespace mapping {

class SiteMap {
  public:

  // constructor
  SiteMap();
  SiteMap(size_t cHeight, 
          size_t cWidth, 
          float cResolution, 
          float xTransform, 
          float yTransform, 
          float unseenGridHeight, 
          float incomingPointVariance, 
          float cellStartingVariance, 
          float minCellVariance);

  // methods
  void binPts(std::vector<mapPoint> rawPts); // method to bin points into map
  void updateCellsBayes(); // method to update cells based on modified points
  void updateMapCoverage(); // method to check if map is all seen or not
  std::vector<cg::mapping::indexPoint> postProcess(std::vector<cg::mapping::indexPoint> ptsCheck); // method to do outlier rejection on pts

  // conversion methods
  cg_msgs::msg::SiteMap toMsg() const;
  void setHeightMapFromMsg(const cg_msgs::msg::SiteMap& msg);

  // getter funcitons 
  size_t getNcells() const {return height_*width_;}
  size_t getWidth() const {return width_;}
  size_t getHeight() const {return height_;}
  float getResolution() const {return resolution_;}
  float getXTransform() const {return xTransform_;}
  float getYTransform() const {return yTransform_;}
  bool getSiteMapFullStatus() const {return siteMapFull_;}
  std::vector<CellBuffer> getBufferMap() const {return bufferMap_;}
  std::vector<float> getHeightMap() const {return heightMap_;}

  private:
  // map views
  std::vector<float> heightMap_;       // "view 1"
  std::vector<CellBuffer> bufferMap_;  // "view 2" 
  std::vector<CellBayes> varianceMap_; // "view 3" 
  std::vector<int> seenPointsMap_; // "view 4" 

  // variables
  bool siteMapFull_ = false; 

  // attributes
  size_t height_ = 1; // the vertical number of cells in the map 
  size_t width_ = 1; // the horizontal number of cells in the map
  float resolution_ = 1.0; // the "resolution" in meters which links the map dimensions to its real world size
  float xTransform_ = 1.0;
  float yTransform_ = 1.0;
  float unseenGridHeight_ = 0.0;
  float incomingPointVariance_ = 0.0;
  float cellStartingVariance_ = 0.0;
  float minCellVariance_ = 0.0;
};

} // mapping namespace
} // cg namespace

#endif // MAPPING__SITE_MAP_HPP
