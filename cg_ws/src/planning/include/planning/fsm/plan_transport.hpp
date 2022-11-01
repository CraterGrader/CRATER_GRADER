#ifndef PLANNING__PLAN_TRANSPORT_HPP
#define PLANNING__PLAN_TRANSPORT_HPP

#include <planning/fsm/fsm.hpp>
#include <planning/transport_planner.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class PlanTransport : public FSM {

public:
  void runState(cg::planning::TransportPlanner &transport_planner, const cg::mapping::Map<float> &current_height_map, const cg::mapping::Map<float> &design_height_map, const std::vector<int> &seen_map, const float threshold_z, const float thresh_max_assignment_distance); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__PLAN_TRANSPORT_HPP
