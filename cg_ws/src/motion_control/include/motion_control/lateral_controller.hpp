#pragma once

#include <cmath> // arctangent
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <planning/common.hpp>
#include <tf2/LinearMath/Quaternion.h> // For visualizing the current goal poses
#include <tf2/utils.h> // Yaw getter

struct LateralControllerDebug {
  double heading_err;
  double cross_track_err;
};

namespace cg {
namespace motion_control {

class LateralController {
public:
  LateralController() {}
  LateralController(double k, double stanley_softening_constant);
  double computeSteer(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state);
  LateralControllerDebug getDebug() const {return debug_;}
  void resetPrevTrajIdx();

private:
  double stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const;

  double scaleToSteerActuators(double desired_steer);
  double k_;  // Stanley controller gain
  double stanley_softening_constant_;
  LateralControllerDebug debug_;
  size_t prev_traj_idx_ = 0;
};

} // namespace motion_control
} // namespace cg
