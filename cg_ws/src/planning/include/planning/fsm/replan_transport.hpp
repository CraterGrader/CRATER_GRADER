#ifndef PLANNING__REPLAN_TRANSPORT_HPP
#define PLANNING__REPLAN_TRANSPORT_HPP

#include <planning/fsm/fsm.hpp>
#include <mapping/map_util.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class ReplanTransport : public FSM {

public:
  void runState(); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__REPLAN_TRANSPORT_HPP
