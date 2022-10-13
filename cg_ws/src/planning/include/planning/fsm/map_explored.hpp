#ifndef PLANNING__MAP_EXPLORED_HPP
#define PLANNING__MAP_EXPLORED_HPP

#include <planning/fsm/fsm.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class MapExplored : public FSM {

public:
  void runState(); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__MAP_EXPLORED_HPP
