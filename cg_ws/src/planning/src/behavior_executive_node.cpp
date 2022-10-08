#include "planning/behavior_executive_node.hpp"

namespace cg {
namespace planning {

  BehaviorExecutive::BehaviorExecutive() : Node("behavior_executive")
  {
    /* Initialize publishers and subscribers */

    // Timer callback
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&BehaviorExecutive::timerCallback, this));
  }

void BehaviorExecutive::timerCallback()
{

}


} // namespace planning
} // namespace cg
