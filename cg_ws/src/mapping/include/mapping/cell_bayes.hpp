#ifndef MAPPING__CELL_BAYES_HPP
#define MAPPING__CELL_BAYES_HPP

#include <vector>
#include <math.h>
// #include <assert.h>
// #include <numeric>
// #include <cg_msgs/msg/site_map.hpp>
// #include "mapping/map_util.hpp"

namespace cg {
namespace mapping {

class CellBayes {
  public:
  // constructor
  CellBayes(float cellStartingVariance, float minCellVariance);

  // methods
  void updateElvationStatic(float ptHeight, float ptVariance);
  void updateVarianceStatic(float ptVariance);

  //getters
  float getCellElevation(){return cellElevation_;}
  float getCellVariance(){return cellVariance_;}

  private:
  // attributes  
  float cellElevation_; 
  float cellVariance_;
  float minCellVariance_;

};

} // mapping namespace
} // cg namespace

#endif // MAPPING__CELL_BAYES_HPP
