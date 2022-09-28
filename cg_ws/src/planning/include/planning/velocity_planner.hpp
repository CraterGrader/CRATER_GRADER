#ifndef PLANNING__VELOCITY_PLANNER_HPP
#define PLANNING__VELOCITY_PLANNER_HPP

#include <vector>

#include <planning/common.hpp>
#include <mapping/site_map.hpp>

namespace cg {
namespace planning {

class VelocityPlanner {

public:
  // Updates the velocity_targets field in-place
  void generateVelocityTargets(
    std::vector<float> &velocity_targets,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::SiteMap &map);
};

} // namespace planning
} // namespace cg

#endif // PLANNING__VELOCITY_PLANNER_HPP
