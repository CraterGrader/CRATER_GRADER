#include "motion_control/lateral_controller.hpp"

namespace cg {
namespace motion_control {

LateralController::LateralController(double k) : k_(k) {}


double LateralController::computeSteer(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state) {
  // TODO
  return 0.0;
}

}  // namespace motion_control
}  // namespace cg
