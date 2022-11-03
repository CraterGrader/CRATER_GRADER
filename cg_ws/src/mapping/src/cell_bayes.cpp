#include "mapping/cell_bayes.hpp"

namespace cg {
namespace mapping {

CellBayes::CellBayes(float cellElevation, float cellStartingVariance, float minCellVariance){
  cellElevation_ = cellElevation;
  cellVariance_ = cellStartingVariance;
  minCellVariance_ = minCellVariance;
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

} // mapping namespace
} // cg namespace
