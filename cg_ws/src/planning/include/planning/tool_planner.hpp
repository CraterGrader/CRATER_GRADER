#ifndef PLANNING__TOOL_PLANNER_HPP
#define PLANNING__TOOL_PLANNER_HPP

#include <planning/common.hpp>
#include <mapping/map.hpp>
#include <cg_msgs/msg/trajectory.hpp>

namespace cg {
namespace planning {

class ToolPlanner {

public:

  ToolPlanner(double design_blade_height, double raised_blade_height)
   : design_blade_height_(design_blade_height), raised_blade_height_(raised_blade_height), autograder_enabled_(false) {}

  // Updates the trajectory.tool_positions field in-place
  void generateToolTargets(
    cg_msgs::msg::Trajectory &trajectory,
    const cg_msgs::msg::Pose2D &agent_pose,
    const cg::mapping::Map<float> &map);

  void setEnable(bool enable) {autograder_enabled_ = enable;}

private:
  double design_blade_height_;
  double raised_blade_height_;
  bool autograder_enabled_;

};

} // namespace planning
} // namespace cg

#endif // PLANNING__TOOL_PLANNER_HPP
