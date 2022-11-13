#include <planning/fsm/plan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

  void PlanTransport::runState(cg::planning::TransportPlanner &transport_planner, const cg::mapping::Map<float> &current_height_map, const cg::mapping::Map<float> &design_height_map, const std::vector<int> &seen_map, const float thresh_max_assignment_distance)
  {
    std::cout << "PLAN_TRANSPORT" << std::endl;

    float objective_value = transport_planner.planTransport(current_height_map, design_height_map, seen_map, thresh_max_assignment_distance);
    std::cout << "    obj value: " << objective_value << std::endl;

    // Update shared current state and the precursing signal
    pre_signal_ = Signal::TRANSPORT_PLANNED;
    curr_state_l0_ = StateL0::GET_TRANSPORT_GOALS;
  }

} // planning namespace
} // cg namespace
