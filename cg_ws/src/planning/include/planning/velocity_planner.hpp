#ifndef PLANNING__VELOCITY_PLANNER_HPP
#define PLANNING__VELOCITY_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/site_map.hpp>
#include <cg_msgs/msg/trajectory.hpp>

namespace cg {
namespace planning {

class VelocityPlanner {

public:
  // Updates the trajectory.velocity_targets field in-place
  void generateVelocityTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::SiteMap &map);
};

} // namespace planning
} // namespace cg

#endif // PLANNING__VELOCITY_PLANNER_HPP
