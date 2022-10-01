#ifndef PLANNING__KINEMATIC_PLANNER_HPP
#define PLANNING__KINEMATIC_PLANNER_HPP

#include <vector>

#include <planning/common.hpp>
#include <mapping/site_map.hpp>
#include <cg_msgs/msg/pose2_d.hpp>

namespace cg {
namespace planning {

class KinematicPlanner {

public:
  // Updates the path field in-place
  void generatePath(
    std::vector<cg_msgs::msg::Pose2D> &path,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::SiteMap &map);
};

} // namespace planning
} // namespace cg

#endif // PLANNING__KINEMATIC_PLANNER_HPP
