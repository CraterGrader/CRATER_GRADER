#pragma once

#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "motion_control/pid_controller.hpp"
#include <planning/common.hpp>
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cg {
namespace motion_control {

class LongitudinalController {
public:
  LongitudinalController(const PIDParams &params);
  void setGains(const double kp, const double ki, const double kd);
  double computeDrive(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state);
  void resetPrevTrajIdx();
private:
  std::unique_ptr<PIDController> velocity_controller_;
  size_t prev_traj_idx_ = 0;
  double scaleToDriveActuators(double desired_drive);
  int LongitudinalController::getClosestPointIndex(
      const cg_msgs::msg::Trajectory &target_trajectory,
      const nav_msgs::msg::Odometry &current_state);
};

} // namespace motion_control
} // namespace cg
