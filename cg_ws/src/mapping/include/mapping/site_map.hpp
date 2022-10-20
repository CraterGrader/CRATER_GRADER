#ifndef MAPPING__SITE_MAP_HPP
#define MAPPING__SITE_MAP_HPP

#include <vector>
#include <math.h>
#include <assert.h>
#include <numeric>
#include <cg_msgs/msg/site_map.hpp>
#include "mapping/map_util.hpp"

namespace cg {
namespace mapping {

struct mapPoint {
  // a struct which simply defines a points position in continous 3-space
    float x;
    float y;
    float z;
};

struct indexPoint {
  // a struct which simply defines a points position in the "2D" map, with a z-height
    int x;
    int y;
    float z;
};

class CellHistory {
  public:
  // constructor
  CellHistory(){window_.resize(windowSize_, 0.0);};

  // methods
  void addPoint(indexPoint pt);
  float getMean();
  float getFirstElement(){return window_.front();}
  bool filterIsEmpty(){return filterEmpty_;}
  
  void filterUpdate(){filterUpdate_ = true;}
  void filterUpdated(){filterUpdate_ = false;}
  bool filterIsUpdated(){return filterUpdate_;}

  private:
  // attributes
  bool filterEmpty_ = true;
  bool filterUpdate_ = false;
  bool filterFull_ = false; // flag which states which "get mean" return to use
  size_t fingerIndex_ = 0; // index which tracks data in filter
  size_t windowSize_ = 1; // the size of the filter window
  std::vector<float> window_; // the window which stores data 
  float prevAvg = 0.0; // previous average used to do constant time updating of a window
  float lastVal = 0.0; // previous value used to do constant time updating of a window

};

class CellBayes {
  public:
  // constructor
  CellBayes(){};
  CellBayes(float cellStartingVariance, float minCellVariance);

  // methods
  void updateElvationStatic(float ptHeight, float ptVariance);
  void updateVarianceStatic(float ptVariance);

  //getters
  float getCellElevation(){return cellElevation_;}
  float getCellVariance(){return cellVariance_;}

  private:
  // attributes  
  float cellElevation_ = 0.0f; 
  float cellVariance_ = 1000.0f;
  float minCellVariance_ = 0.001f;

};

class SiteMap {
  public:

  // constructor
  SiteMap();
  SiteMap(size_t cHeight, size_t cWidth, float cResolution);
  SiteMap(size_t cHeight, 
          size_t cWidth, 
          float cResolution, 
          float filterMaxTerrain, 
          float filterMinTerrain, 
          float xTransform, 
          float yTransform, 
          float unseenGridHeight, 
          float incomingPointVariance, 
          float cellStartingVariance, 
          float minCellVariance);

  // methods
  void binPts(std::vector<mapPoint> rawPts); // method to bin points into map
  void updateCellsMean(); // method to update cells based on modified points
  void updateCellsBayes(); // method to update cells based on modified points
  void updateMapCoverage(); // method to check if map is all seen or not
  std::vector<cg::mapping::indexPoint> postProcess(std::vector<cg::mapping::indexPoint> ptsCheck); // method to do outlier rejection on pts

  // conversion methods
  cg_msgs::msg::SiteMap toMsg() const;
  void setHeightMapFromMsg(const cg_msgs::msg::SiteMap& msg);

  // getter funcitons 
  float getResolution() const {return resolution_;}
  size_t getNcells() const {return height_*width_;}
  size_t getWidth() const {return width_;}
  size_t getHeight() const {return height_;}
  std::vector<CellHistory> getFilterMap() const {return filterMap_;}
  std::vector<float> getHeightMap() const {return heightMap_;}
  float getXTransform() const {return xTransform_;}
  float getYTransform() const {return yTransform_;}
  bool getSiteMapFullStatus() const {return siteMapFull_;}

  private:
  // Uncomment this line when not testing 
  std::vector<float> heightMap_;       // "view 1"
  std::vector<CellHistory> filterMap_; // "view 2" 
  std::vector<CellBayes> varianceMap_; // "view 3" 
  bool siteMapFull_ = false; 
  float zTransform_ = 0.0f;

  // DEBUG: test maps
  // std::vector<float> heightMap_{0,    0,    0,    0,    0,    0,    0,    0,    0, 
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0,
  //                               0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,
  //                               0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,
  //                               0,    0,  1.0, -2.0, -4.0, -2.0,  1.0,    0,    0,
  //                               0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,
  //                               0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0,
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0};
  // std::vector<float> heightMap_{0,    0,    0,    0,    0,    0,    0,    0,    0,    0, 
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
  //                               0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,    0,
  //                               0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,    0,
  //                               0,    0,  1.0, -2.0, -4.0, -2.0,  1.0,    0,    0,    0,
  //                               0,    0,  1.0,    0, -2.0,    0,  1.0,    0,    0,    0,
  //                               0,    0,    0,  1.0,  1.0,  1.0,    0,    0,    0,    0,
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
  //                               0,    0,    0,    0,    0,    0,    0,    0,    0,    0};

  // attributes
  size_t height_; // the vertical number of cells in the map 
  size_t width_; // the horizontal number of cells in the map
  float resolution_; // the "resolution" in meters which links the map dimensions to its real world size
  float filterMaxTerrain_ = 1.0f; // the maximum value of points which can be added to the map, relative to map frame
  float filterMinTerrain_ = -1.0f; // the minimum value of points which can be added to the map, relative to map frame
  float xTransform_ = 1.0f;
  float yTransform_ = 1.0f;
  float unseenGridHeight_ = 0.0f;
  float incomingPointVariance_ = 0.05f;
  float cellStartingVariance_ = 0.05f;
  float minCellVariance_ = 0.00001f;

};

} // mapping namespace
} // cg namespace

#endif // MAPPING__SITE_MAP_HPP
