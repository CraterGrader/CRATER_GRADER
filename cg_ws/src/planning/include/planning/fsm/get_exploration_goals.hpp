#ifndef PLANNING__GET_EXPLORATION_GOALS_HPP
#define PLANNING__GET_EXPLORATION_GOALS_HPP

#include <planning/fsm/fsm.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class GetExplorationGoals : public FSM {

public:
  void runState(); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__GET_EXPLORATION_GOALS_HPP
