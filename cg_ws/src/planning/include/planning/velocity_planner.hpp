#ifndef PLANNING__VELOCITY_PLANNER_HPP
#define PLANNING__VELOCITY_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/map.hpp>
#include <cg_msgs/msg/trajectory.hpp>

namespace cg {
namespace planning {

class VelocityPlanner {

public:
  // Updates the trajectory.velocity_targets field in-place
  VelocityPlanner() {}
  VelocityPlanner(double velocity_constant) :
    constant_velocity_(velocity_constant) {}

  void generateVelocityTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map);

private:

  double constant_velocity_ = 100; // TODO: 1) make param DEBUG
  // double constant_velocity_ = 15*0.003795275591; // m/s TODO: 1) make param 

};

} // namespace planning
} // namespace cg

#endif // PLANNING__VELOCITY_PLANNER_HPP
