#include "mapping/site_map.hpp"

namespace cg {
namespace mapping {

// default constructor
SiteMap::SiteMap(){}

// full param constructor
SiteMap::SiteMap(size_t cHeight, 
                  size_t cWidth, 
                  float cResolution, 
                  float xTransform, 
                  float yTransform, 
                  float unseenGridHeight, 
                  float incomingPointVariance, 
                  float cellStartingVariance, 
                  float minCellVariance){

  height_ = cHeight;
  width_ = cWidth;
  resolution_ = cResolution;
  xTransform_ = xTransform;
  yTransform_ = yTransform;
  unseenGridHeight_ = unseenGridHeight;
  incomingPointVariance_ = incomingPointVariance;
  cellStartingVariance_ = cellStartingVariance;
  minCellVariance_ = minCellVariance;
  heightMap_.resize(height_*width_, unseenGridHeight_);
  bufferMap_.resize(height_*width_, CellBuffer(unseenGridHeight_));
  varianceMap_.resize(height_*width_, CellBayes(cellStartingVariance_, minCellVariance_));
  seenPointsMap_.resize(height_*width_, 0);
  }

void SiteMap::binPts(std::vector<mapPoint> rawPts){

  // statically declare a vector of index points based on the raw point size
  std::vector<cg::mapping::indexPoint> discretePoints(rawPts.size());

  // pack every raw point value into an Index point
  for (size_t i=0 ; i < rawPts.size(); ++i){
    float x_pos_site_map_frame = cg::mapping::convertMaptoSiteMapFrame(rawPts[i].x, xTransform_);
    float y_pos_site_map_frame = cg::mapping::convertMaptoSiteMapFrame(rawPts[i].y, yTransform_);
    discretePoints[i].x = cg::mapping::pointToDiscreteCoord(x_pos_site_map_frame, getResolution());
    discretePoints[i].y = cg::mapping::pointToDiscreteCoord(y_pos_site_map_frame, getResolution());
    discretePoints[i].z = rawPts[i].z - plane_offset_;
  }
  
  // use postProcess method 
  std::vector<cg::mapping::indexPoint> processedPts = postProcess(discretePoints);

  // update view 2, filtermap
  for (size_t i=0 ; i < processedPts.size(); i++){
    size_t index = cg::mapping::discreteCoordsToCellIndex(processedPts[i].x, processedPts[i].y, getWidth());
    bufferMap_[index].addPoint(processedPts[i]);
    seenPointsMap_[index] = 1;
  }
}

std::vector<cg::mapping::indexPoint> SiteMap::postProcess(std::vector<cg::mapping::indexPoint> ptsCheck){
  // out of bounds check 

  size_t badCount = 0; 
  std::vector<bool> isPointValid(ptsCheck.size(), true);

  for (size_t i=0 ; i < ptsCheck.size(); i++){
    if (!cg::mapping::indexInMap(ptsCheck[i].x, ptsCheck[i].y, getWidth(), getHeight())){
      badCount++; 
      isPointValid[i] = false;
    }
  }

  std::vector<cg::mapping::indexPoint> goodPts;
  goodPts.resize(ptsCheck.size()-badCount);
  size_t goodCounter = 0; 
  for (size_t i=0 ; i < ptsCheck.size(); i++){
    if (isPointValid[i]){
      goodPts[goodCounter] = ptsCheck[i];
      goodCounter++;
    }
  }
  return goodPts;
}

void SiteMap::updateMapCoverage(){
  size_t sum_of_elems = static_cast<size_t>(std::reduce(seenPointsMap_.begin(), seenPointsMap_.end()));
  if (getNcells() == sum_of_elems){
    siteMapFull_ = true;
  }
  else {
    siteMapFull_ = false;
  }
  siteMapCoverageRatio_ = static_cast<float>(sum_of_elems) / static_cast<float>(getNcells());
}

void SiteMap::updateCellsBayes(){
  // for each cell in map
  for (size_t i=0; i<getNcells(); i++){

    // if the filter has been updated since the last updateCellsBayes
    if (bufferMap_[i].doesBufferHaveNewData() == true){
      // get the new element in the filter map 
      float elev = bufferMap_[i].getHeight();
      // set the variance of the incoming point 
      // TODO, turn into a function for variance expansion
      float variance = incomingPointVariance_;
      // set flag that filter map has been updated 
      bufferMap_[i].bufferHasBeenUpdated();
      // call bayes recursive update
      varianceMap_[i].updateElvationStatic(elev, variance);
      varianceMap_[i].updateVarianceStatic(variance);
    }
    // get the cell elevation for the map
    heightMap_[i] = varianceMap_[i].getCellElevation();
  }
}

void SiteMap::normalizeHeightMap(){
  float sum = 0.0f;
  // for every cell, sum to get mean
  // TODO convert this to 
  for (size_t i=0; i<getNcells(); i++){
    sum += heightMap_[i];
  }
  plane_offset_ = sum / getNcells();
  std::cout << "  offset " << plane_offset_ << std::endl;
  // for every cell, subtract mean
  for (size_t i=0; i<getNcells(); i++){
    // std::cout << "    before" << heightMap_[i] << std::endl;
    heightMap_[i] -= plane_offset_;
    bufferMap_[i].offset_height(plane_offset_);
    varianceMap_[i].offset_height(plane_offset_);
    // std::cout << "      after" << heightMap_[i] << std::endl;
  }
}

std::vector<float> SiteMap::getVarianceMapFloats() {

  std::vector<float> map_of_variances;

  for (size_t i=0; i<getNcells(); i++){
    map_of_variances.push_back(varianceMap_[i].getCellVariance());
  }
  return map_of_variances;
}

cg_msgs::msg::SiteMap SiteMap::toMsg() const {
  cg_msgs::msg::SiteMap map_msg;
  map_msg.height_map = heightMap_;
  return map_msg;
}

void SiteMap::setHeightMapFromMsg(const cg_msgs::msg::SiteMap& msg) {
  heightMap_ = msg.height_map;
}

bool SiteMap::setHeightMap(std::vector<float> input_data) {
  // Don't update data if the dimensions are wrong
  if (input_data.size() != (height_ * width_)) {
    return false;
  }
  // Otherwise, ok to update
  heightMap_ = input_data;
  return true;
}

} // mapping namespace
} // cg namespace
