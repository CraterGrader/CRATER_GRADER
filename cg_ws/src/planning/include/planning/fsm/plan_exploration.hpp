#ifndef PLANNING__PLAN_EXPLORATION_HPP
#define PLANNING__PLAN_EXPLORATION_HPP

#include <planning/fsm/fsm.hpp>
#include <planning/exploration_planner.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class PlanExploration : public FSM {

public:
  void runState(cg::planning::ExplorationPlanner &exploration_planner, const cg::mapping::Map<float> &map); // Main function to run current state; optionally modifies signal and state for transition

private:
  bool exploration_planned_ = false;

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__PLAN_EXPLORATION_HPP
