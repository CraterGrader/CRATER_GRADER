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
  VelocityPlanner(double velocity_constant) :
    constant_velocity_(velocity_constant) {}

  void generateVelocityTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map);

  void setVelocityTarget(double desired_velocity);

  double getVelocityTarget() const {return constant_velocity_;}

private:
  double constant_velocity_;

};

} // namespace planning
} // namespace cg

#endif // PLANNING__VELOCITY_PLANNER_HPP
