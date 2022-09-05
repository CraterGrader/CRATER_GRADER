#include "mapping/site_map.hpp"

namespace cg {
namespace mapping {

void CellHistory::addPoint(indexPoint pt){
  window_[fingerIndex_] = pt.z;
  fingerIndex_ = fingerIndex_ + 1;

  if (fingerIndex_ == windowSize_){
    fingerIndex_= 0;
    filterFull_= true;
  }
}

SiteMap::SiteMap(size_t cHeight, size_t cWidth, float cResolution){  
  height_ = cHeight;
  width_ = cWidth;
  resolution_ = cResolution;
  heightMap_.resize(height_*width_, 0.0f);
  filterMap_.resize(height_*width_, CellHistory());
}

float SiteMap::binLen(float pos, float resolution){
  return floor(((pos+resolution)/2/resolution));
}

float CellHistory::getMean(){
  if (!filterFull_){
    if (fingerIndex_ != 0){
      return (float) (std::accumulate(window_.begin(), window_.end(), 0.0f) / fingerIndex_);
    }
  }
  else{
    return (float) (std::accumulate(window_.begin(), window_.end(), 0.0f) / windowSize_);
  }
  return 0.0f;
}

void SiteMap::binPts(std::vector<mapPoint> rawPts){
  std::vector<cg::mapping::indexPoint> descretePoints(rawPts.size());

  for (size_t i=0 ; i < rawPts.size(); i++){
    descretePoints[i].x = binLen(rawPts[i].x, getResolution());
    descretePoints[i].y = binLen(rawPts[i].y, getResolution());
    descretePoints[i].z = rawPts[i].z;
  }
  // use postProcess method 
  // TODO: ADD POSTPROCESSING
  // update view 2, filtermap
  for (size_t i=0 ; i < descretePoints.size(); i++){
    size_t index = descretePoints[i].x + (descretePoints[i].y * getWidth());
    filterMap_[index].addPoint(descretePoints[i]);
  }
}

void SiteMap::postProcess(){
  // take a given point, and throw out if necessary  
}

void SiteMap::updateCells(){
  // use view 2 filter map to update values in view 1 height map
  for (size_t i=0; i<getNcells(); i++){
    heightMap_[i] = filterMap_[i].getMean();
  }
}

} // mapping namespace
} // cg namespace
