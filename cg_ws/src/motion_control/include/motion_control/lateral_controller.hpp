#pragma once

#include <cmath> // arctangent
#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <planning/common.hpp>
#include <tf2/LinearMath/Quaternion.h> // For visualizing the current goal poses
#include <tf2/utils.h> // Yaw getter

namespace cg {
namespace motion_control {

class LateralController {
public:
  LateralController() {}
  LateralController(double k, double stanley_softening_constant_);
  double computeSteer(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state);

private:
  double stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const;
  double k_;  // Stanley controller gain
  double stanley_softening_constant_;
};

} // namespace motion_control
} // namespace cg
