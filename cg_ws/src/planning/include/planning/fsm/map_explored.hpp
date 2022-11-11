#ifndef PLANNING__MAP_EXPLORED_HPP
#define PLANNING__MAP_EXPLORED_HPP

#include <planning/fsm/fsm.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class MapExplored : public FSM {

public:
  void runState(float current_map_coverage_ratio_, float map_coverage_threshold); // Main function to run current state; optionally modifies signal and state for transition

private:

bool explored_once_{false};

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__MAP_EXPLORED_HPP
