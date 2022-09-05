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
  std::vector<cg::mapping::indexPoint> processedPts = postProcess(descretePoints);

  // update view 2, filtermap
  for (size_t i=0 ; i < processedPts.size(); i++){
    size_t index = processedPts[i].x + (processedPts[i].y * getWidth());
    filterMap_[index].addPoint(processedPts[i]);
  }
}

std::vector<cg::mapping::indexPoint> SiteMap::postProcess(std::vector<cg::mapping::indexPoint> ptsCheck){
  // out of bounds check 

  size_t badCount = 0; 
  std::vector<bool> goodBad(ptsCheck.size(), true);

  for (size_t i=0 ; i < ptsCheck.size(); i++){
    if ((ptsCheck[i].x >= (int) getWidth()) || (ptsCheck[i].y >= (int) getHeight()) || (ptsCheck[i].y < 0.0) || (ptsCheck[i].x < 0.0)){
      badCount++; 
      goodBad[i] = false;
    }
  }

  std::vector<cg::mapping::indexPoint> goodPts;
  goodPts.resize(ptsCheck.size()-badCount);

  size_t goodCounter = 0; 

  for (size_t i=0 ; i < ptsCheck.size(); i++){
    if (goodBad[i]){
      goodPts[goodCounter] = ptsCheck[i];
      goodCounter++;
    }
  }

  return goodPts;
}

void SiteMap::updateCells(){
  // use view 2 filter map to update values in view 1 height map
  for (size_t i=0; i<getNcells(); i++){
    heightMap_[i] = filterMap_[i].getMean();
  }
}

} // mapping namespace
} // cg namespace
