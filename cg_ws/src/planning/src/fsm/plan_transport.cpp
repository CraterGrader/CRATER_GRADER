#include <planning/fsm/plan_transport.hpp>
#include <iostream> // DEBUG

namespace cg {
namespace planning {

  void PlanTransport::runState(cg::planning::TransportPlanner &transport_planner, const cg::mapping::Map<float> &current_height_map, const cg::mapping::Map<float> &design_height_map, const float threshold_z, const float thresh_max_assignment_distance)
  {
    std::cout << "PLAN_TRANSPORT" << std::endl;

    float objective_value = transport_planner.planTransport(current_height_map, design_height_map, threshold_z, thresh_max_assignment_distance);
    std::cout << "    obj value: " << objective_value << std::endl;

    // Update shared current state and the precursing signal
    pre_signal_ = Signal::TRANSPORT_PLANNED;
    curr_state_ = State::GET_TRANSPORT_GOALS;
  }

} // planning namespace
} // cg namespace
