#pragma once

#include <cg_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cg {
namespace motion_control {

class LateralController {
public:
  LateralController() {}
  LateralController(double k);
  double computeSteer(
    const cg_msgs::msg::Trajectory &target_trajectory,
    const nav_msgs::msg::Odometry &current_state);

private:
  double stanleyControlLaw(
    const double heading_err,
    const double cross_track_err,
    const double velocity) const;
  double k_;  // Stanley controller gain
};

} // namespace motion_control
} // namespace cg

