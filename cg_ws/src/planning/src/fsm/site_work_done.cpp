#include <iostream> // DEBUG
#include <planning/fsm/site_work_done.hpp>

namespace cg {
namespace planning {

void SiteWorkDone::runState(const cg::mapping::Map<float> &current_height_map,
                            const cg::mapping::Map<float> &design_height_map,
                            float topology_equality_threshold) {
  std::cout << "SITE_WORK_DONE" << std::endl;

  bool within_thresh = cg::mapping::mapSimilarityWithinThreshold(
      current_height_map.getCellData(), design_height_map.getCellData(),
      topology_equality_threshold);

  // Update shared current state and the precursing signal
  if (within_thresh) {
    pre_signal_ = Signal::YES;
    curr_state_l0_ = StateL0::END_MISSION;
  } else {
    pre_signal_ = Signal::NO;
    curr_state_l0_ = StateL0::MAP_EXPLORED;
  }
}

} // namespace planning
} // namespace cg
