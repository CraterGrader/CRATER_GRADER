#ifndef PLANNING__FOLLOWING_TRAJECTORY_HPP
#define PLANNING__FOLLOWING_TRAJECTORY_HPP

#include <planning/fsm/fsm.hpp>

namespace cg {
namespace planning {

// Inherit from FSM to have access to current state/signal
class FollowingTrajectory : public FSM {

public:
  void runState(); // Main function to run current state; optionally modifies signal and state for transition

}; // class State

} // namespace planning
} // namespace cg

#endif // PLANNING__FOLLOWING_TRAJECTORY_HPP
