#include <gtest/gtest.h>
#include <planning/tool_planner.hpp>

namespace cg {
namespace planning {


TEST(ToolPlannerTest, constructorTest)
{

  cg_msgs::msg::Pose2D start_pose = create_pose2d(0,0,deg2rad(0));

  std::vector<cg_msgs::msg::Pose2D> path {
    cg::planning::create_pose2d(0.5, 0.5, deg2rad(0)),
    cg::planning::create_pose2d(0.0, 0.0, deg2rad(0)),
    cg::planning::create_pose2d(0.5, 1.5, deg2rad(0)),
    cg::planning::create_pose2d(1.5, 1.5, deg2rad(0))};

  double design_blade_height = 0;
  double raised_blade_height = 1;
  ToolPlanner tool_planner = ToolPlanner(design_blade_height, raised_blade_height);


  cg_msgs::msg::Trajectory trajectory;
  trajectory.set__path(path);

  cg::mapping::Map<float> map;

  tool_planner.generateToolTargets(
    trajectory,
    start_pose,
    map);

  std::cout << "Tool positions:\n";
  for (float tool_pos: trajectory.tool_positions) {
    std::cout << tool_pos << std::endl;
  }

  EXPECT_EQ(trajectory.tool_positions[0], 0);

}

}
}