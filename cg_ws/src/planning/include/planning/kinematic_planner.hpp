#ifndef PLANNING__KINEMATIC_PLANNER_HPP
#define PLANNING__KINEMATIC_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/site_map.hpp>
#include <cg_msgs/msg/trajectory.hpp>

namespace cg {
namespace planning {

class KinematicPlanner {

public:
  // Updates the trajectory.path field in-place
  void generatePath(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::SiteMap &map);
};

} // namespace planning
} // namespace cg

#endif // PLANNING__KINEMATIC_PLANNER_HPP
