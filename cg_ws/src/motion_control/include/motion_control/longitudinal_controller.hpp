#pragma once

#include "motion_control/pid_controller.hpp"

#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cg {
namespace motion_control {

class LongitudinalController {
public:
  LongitudinalController() {}
  LongitudinalController(const PIDParams &params);
  double computeVelocity(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state);
private:
  PIDController velocity_controller_;
};

} // namespace motion_control
} // namespace cg
