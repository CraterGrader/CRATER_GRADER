#include "mapping/site_map.hpp"

namespace cg {
namespace mapping {

void CellHistory::addPoint(indexPoint pt){

  filterEmpty_ = false;

  // if the filter is full, save the value getting removed before its removed
  if (filterFull_){ lastVal = window_[fingerIndex_]; }

  // overide point with incoming point & increment the index 
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

void CellBayes::updateElvationStatic(float ptHeight, float ptVariance){
  // Probabilistic Terrain Mapping for Mobile Robots With Uncertain Localization
  // https://ieeexplore.ieee.org/document/8392399
   cellElevation_ = (getCellVariance() * ptHeight + ptVariance * getCellElevation()) / (getCellVariance() + ptVariance);
}

void CellBayes::updateVarianceStatic(float ptVariance){
  // Probabilistic Terrain Mapping for Mobile Robots With Uncertain Localization
  // https://ieeexplore.ieee.org/document/8392399
  // Update cell variance, but clamp to minimum value to prevent vanishing to zero (for dynamic update)
  cellVariance_ = std::max(minCellVariance_, (ptVariance * getCellVariance()) / (ptVariance + getCellVariance()));
}

// default constructor
SiteMap::SiteMap(){}

// nominal constructor
SiteMap::SiteMap(size_t cHeight, size_t cWidth, float cResolution){
  height_ = cHeight;
  width_ = cWidth;
  resolution_ = cResolution;
  heightMap_.resize(height_*width_, 0.0f);
  filterMap_.resize(height_*width_, CellHistory());
  varianceMap_.resize(height_*width_, CellBayes());

}

int SiteMap::binLen(float pos){
  // for a continous point along an axis, return its int location which should align with map
  return (int) floor(pos/getResolution());
}

float CellHistory::getMean(){
  if (!filterFull_){
    if (fingerIndex_ != 0){ // if filter is not full BUT we have data
      // TODO: make this NOT iterate over entire window
      return (float) (std::accumulate(window_.begin(), window_.end(), 0.0f) / fingerIndex_);
    }
  }
  else{ // if filter is full and we are only constant time updating
    // TODO CONVERT THIS TO CONSTANT TIME
    return (float) (std::accumulate(window_.begin(), window_.end(), 0.0f) / windowSize_);
    }
  // if filter is NOT FULL AND THERE ARE NO POINTS IN THE WINDOW, THEN RETURN ZERO 
  return 0.0f;
}

void SiteMap::binPts(std::vector<mapPoint> rawPts){
  std::vector<cg::mapping::indexPoint> descretePoints(rawPts.size());

  for (size_t i=0 ; i < rawPts.size(); i++){
    descretePoints[i].x = binLen(rawPts[i].x - xTransform_);
    descretePoints[i].y = binLen(rawPts[i].y - yTransform_);
    descretePoints[i].z = rawPts[i].z;
  }
  
  // use postProcess method 
  std::vector<cg::mapping::indexPoint> processedPts = postProcess(descretePoints);

  // update view 2, filtermap
  for (size_t i=0 ; i < processedPts.size(); i++){

    size_t index = processedPts[i].y + (processedPts[i].x * getWidth());    
    filterMap_[index].addPoint(processedPts[i]);
    filterMap_[index].filterUpdate(); 

  }
}

std::vector<cg::mapping::indexPoint> SiteMap::postProcess(std::vector<cg::mapping::indexPoint> ptsCheck){
  // out of bounds check 

  size_t badCount = 0; 
  std::vector<bool> goodBad(ptsCheck.size(), true);

  for (size_t i=0 ; i < ptsCheck.size(); i++){
    if (!cg::mapping::indexInMap(ptsCheck[i].x, ptsCheck[i].y, getWidth(), getHeight())){
      if (!cg::mapping::heightInRange(ptsCheck[i].z, filterMinTerrain_, filterMaxTerrain_)){
        badCount++; 
        goodBad[i] = false;
      }
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

void SiteMap::updateCellsMean(){
  // use view 2 filter map to update values in view 1 height map
  for (size_t i=0; i<getNcells(); i++){
    heightMap_[i] = filterMap_[i].getMean();
  }
}

void SiteMap::updateCellsBayes(){
  // for each cell in map
  for (size_t i=0; i<getNcells(); i++){
    // if the filter is empty, set the height of the cell to the default value
    if (filterMap_[i].filterIsEmpty() == true){
      heightMap_[i] = unseenGridHeight_;
    }
    // if filter is not empty (we have seen this cell before)
    else{
      // if the filter has been updated since the last updateCellsBayes
      if (filterMap_[i].filterIsUpdated() == true){
        // get the new element in the filter map 
        float elev = filterMap_[i].getFirstElement();
        // set the variance of the incoming point 
        // TODO, turn into a function for variance expansion
        float variance = incomingPointVariance_;
        // set flag that filter map has been updated 
        filterMap_[i].filterUpdated();
        // call bayes recursive update
        varianceMap_[i].updateElvationStatic(elev, variance);
        varianceMap_[i].updateVarianceStatic(variance);
      }
      // get the cell elevation for the map
      heightMap_[i] = varianceMap_[i].getCellElevation();
    }

  }
}

cg_msgs::msg::SiteMap SiteMap::toMsg() const {
  cg_msgs::msg::SiteMap map_msg;
  map_msg.height_map = heightMap_;
  return map_msg;
}

void SiteMap::setHeightMapFromMsg(const cg_msgs::msg::SiteMap& msg) {
  heightMap_ = msg.height_map;
}

} // mapping namespace
} // cg namespace
