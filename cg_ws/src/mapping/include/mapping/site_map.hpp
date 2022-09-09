#include <vector>
#include <math.h>
#include <assert.h>
#include <numeric>

namespace cg {
namespace mapping {

struct mapPoint {
    float x;
    float y;
    float z;
};

struct indexPoint {
    int x;
    int y;
    float z;
};

class CellHistory {
  public:
  // constructor
  CellHistory(){
    window_.resize(windowSize_, 0.0);
  };

  // methods
  void addPoint(indexPoint pt);
  float getMean();

  private:

  // attributes  
  bool filterFull_ = false;
  size_t fingerIndex_ = 0;
  size_t windowSize_ = 1000;
  std::vector<float> window_;
  float prevAvg = 0.0;
  float lastVal = 0.0;

};

class SiteMap {
  public:

  // constructor
  SiteMap();
  SiteMap(size_t cHeight, size_t cWidth, float cResolution);

  // methods
  void binPts(std::vector<mapPoint> rawPts); // method to bin points into map
  void updateCells(); // method to update cells based on modified points
  float binLen(float pos, float resolution);
  std::vector<cg::mapping::indexPoint> postProcess(std::vector<cg::mapping::indexPoint> ptsCheck); // method to possibly do outlier rejection and associated tasks on pts 

  // getter funcitons 
  float getResolution(){return resolution_;}
  size_t getNcells(){return height_*width_;}
  size_t getWidth(){return width_;}
  size_t getHeight(){return height_;}
  // size_t getHeightMapSize(){return heightMap_.size();} // for testing atm 
  std::vector<CellHistory> getFilterMap(){return filterMap_;}
  std::vector<float> getHeightMap(){return heightMap_;}

  // private:
  std::vector<float> heightMap_;       // "view 1"
  std::vector<CellHistory> filterMap_; // "view 2" 

  // attributes
  size_t height_;
  size_t width_;
  float resolution_;
  size_t decimation_ = 100;
  float filterMaxTerrain_ = 0.25;
  float filterMinTerrain_ = -0.25;

};

} // mapping namespace
} // cg namespace
