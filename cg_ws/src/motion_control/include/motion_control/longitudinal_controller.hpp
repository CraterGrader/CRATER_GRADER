#pragma once

#include <memory>

#include "motion_control/pid_controller.hpp"

#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cg {
namespace motion_control {

class LongitudinalController {
public:
  LongitudinalController(const PIDParams &params);
  void setGains(const double kp, const double ki, const double kd);
  double computeDrive(
    // TODO should this be refactored to take a single TrajectoryPoint?
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state);
private:
  std::unique_ptr<PIDController> velocity_controller_;
};

} // namespace motion_control
} // namespace cg
