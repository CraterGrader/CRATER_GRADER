#include "motion_control/longitudinal_controller.hpp"

namespace cg {
namespace motion_control {

LongitudinalController::LongitudinalController(const PIDParams &params) {
  velocity_controller_ = std::make_unique<PIDController>(PIDController(params));
}

void LongitudinalController::setGains(const double kp, const double ki, const double kd) {
  // TODO
}

double LongitudinalController::computeDrive(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state) {
  // TODO
  return 0.0;
}


}  // namespace motion_control
}  // namespace cg
