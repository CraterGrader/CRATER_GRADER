#pragma once

#include <memory>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "motion_control/pid_controller.hpp"
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cg {
namespace motion_control {

class LongitudinalController {
public:
  LongitudinalController(const PIDParams &params, float min_drive_speed_scalar, float max_steer_speed);
  void setGains(const double kp, const double ki, const double kd);
  double computeDrive(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state,
    const size_t traj_idx, const float steer_error);
private:
  std::unique_ptr<PIDController> velocity_controller_;
  double scaleToDriveActuators(double desired_drive);
  int getClosestPointIndex(
      const cg_msgs::msg::Trajectory &target_trajectory,
      const nav_msgs::msg::Odometry &current_state);
  float max_steer_error_ = 1000;
  float min_drive_speed_scalar_ = 0.2;
};

} // namespace motion_control
} // namespace cg
