#include "mapping/site_map.hpp"

namespace cg {
namespace mapping {

void CellHistory::addPoint(indexPoint pt){

  // if the filter is full, save the value getting removed 
  if (filterFull_){ lastVal = window_[fingerIndex_]; }

  // overide point with incoming point
  window_[fingerIndex_] = pt.z;
  fingerIndex_ = fingerIndex_ + 1;

  // if the window is at end, loop back to beginning 
  if (fingerIndex_ == windowSize_){
    fingerIndex_= 0;

    // if first time the filter is filled 
    if (!filterFull_){
      filterFull_= true;

      // initialize prevAvg for const time mean updating
      prevAvg = getMean();
    }
  }
}

// default constructor
SiteMap::SiteMap(){}

SiteMap::SiteMap(size_t cHeight, size_t cWidth, float cResolution){  
  height_ = cHeight;
  width_ = cWidth;
  // todo add check to ensure that resolution is possible with map size
  resolution_ = cResolution;
  heightMap_.resize(height_*width_, 0.0f);
  filterMap_.resize(height_*width_, CellHistory());
}

float SiteMap::binLen(float pos, float resolution){
  return floor(pos/resolution);
  // return floor(((pos+resolution)/2/resolution));
}

float CellHistory::getMean(){
  if (!filterFull_){
    if (fingerIndex_ != 0){ // if filter is not full BUT we have data

      // TODO: make this NOT iterate over entire window
      return (float) (std::accumulate(window_.begin(), window_.end(), 0.0f) / fingerIndex_);
    }
  }
  else{ // if filter is full and we are only constant time updating

    // return (float) (std::accumulate(window_.begin(), window_.end(), 0.0f) / windowSize_);

    prevAvg = prevAvg + ((window_[fingerIndex_] - lastVal)/windowSize_);

    return prevAvg;
    
    }

  // if filter is NOT FULL AND THERE ARE NO POINTS IN THE WINDOW, THEN RETURN ZERO 
  return 0.0f;
}

void SiteMap::binPts(std::vector<mapPoint> rawPts){
  std::vector<cg::mapping::indexPoint> descretePoints(rawPts.size()/decimation_);

  for (size_t i=0 ; i < (rawPts.size()/decimation_); i++){
    descretePoints[i].x = binLen(rawPts[i*decimation_].x, getResolution());
    descretePoints[i].y = binLen(rawPts[i*decimation_].y, getResolution());
    descretePoints[i].z = rawPts[i*decimation_].z;
  }
  
  // use postProcess method 
  std::vector<cg::mapping::indexPoint> processedPts = postProcess(descretePoints);

  // update view 2, filtermap
  for (size_t i=0 ; i < processedPts.size(); i++){

    size_t index = processedPts[i].y + (processedPts[i].x * getWidth());
    // size_t index = processedPts[i].x + (processedPts[i].y * getWidth());
    
    filterMap_[index].addPoint(processedPts[i]);

  }
}

std::vector<cg::mapping::indexPoint> SiteMap::postProcess(std::vector<cg::mapping::indexPoint> ptsCheck){
  // out of bounds check 

  size_t badCount = 0; 
  std::vector<bool> goodBad(ptsCheck.size(), true);

  for (size_t i=0 ; i < ptsCheck.size(); i++){
    if ((ptsCheck[i].x >= (int) getWidth()) 
          || (ptsCheck[i].y >= (int) getHeight()) 
          || (ptsCheck[i].y < 0.0) 
          || (ptsCheck[i].x < 0.0)
          || (ptsCheck[i].z > filterMaxTerrain_)
          || (ptsCheck[i].z < filterMinTerrain_)){
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
