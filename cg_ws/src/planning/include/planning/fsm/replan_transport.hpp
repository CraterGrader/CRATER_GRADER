#ifndef PLANNING__REPLAN_TRANSPORT_HPP
#define PLANNING__REPLAN_TRANSPORT_HPP

#include <planning/fsm/fsm.hpp>
#include <mapping/map_util.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class ReplanTransport : public FSM {

public:
  void runState(int max_calls_before_replan); // Main function to run current state; optionally modifies signal and state for transition

private:
  int transport_counter_ = 0;

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__REPLAN_TRANSPORT_HPP
