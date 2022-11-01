#include <gtest/gtest.h>
#include <planning/velocity_planner.hpp>

namespace cg {
namespace planning {


TEST(VelocityPlannerTest, constructorTest)
{

  cg_msgs::msg::Pose2D start_pose = create_pose2d(0,0,deg2rad(0));

  std::vector<cg_msgs::msg::Pose2D> path {
    cg::planning::create_pose2d(0.5, 0.5, deg2rad(0)),
    cg::planning::create_pose2d(0.0, 0.0, deg2rad(0)),
    cg::planning::create_pose2d(0.5, 1.5, deg2rad(0)),
    cg::planning::create_pose2d(1.5, 1.5, deg2rad(0))};

  double constant_vel = 1.5;
  VelocityPlanner velocity_planner = VelocityPlanner(constant_vel);

  cg_msgs::msg::Trajectory trajectory;
  trajectory.set__path(path);

  cg::mapping::Map<float> map;

  velocity_planner.generateVelocityTargets(
    trajectory,
    start_pose,
    map);

  EXPECT_NEAR(trajectory.velocity_targets[0], constant_vel, 0.001);
  EXPECT_NEAR(trajectory.velocity_targets[1], -constant_vel, 0.001);
  EXPECT_NEAR(trajectory.velocity_targets[2], constant_vel, 0.001);
  EXPECT_NEAR(trajectory.velocity_targets[3], constant_vel, 0.001);

}

}
}
