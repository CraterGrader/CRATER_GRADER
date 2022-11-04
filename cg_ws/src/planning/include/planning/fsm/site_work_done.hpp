#ifndef PLANNING__SITE_WORK_DONE_HPP
#define PLANNING__SITE_WORK_DONE_HPP

#include <mapping/map.hpp>
#include <mapping/map_util.hpp>
#include <planning/fsm/fsm.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class SiteWorkDone : public FSM {

public:
  void runState(
      const cg::mapping::Map<float> &current_height_map,
      const cg::mapping::Map<float> &design_height_map,
      float topology_equality_threshold); // Main function to run current state;
                                          // optionally modifies signal and
                                          // state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__SITE_WORK_DONE_HPP
