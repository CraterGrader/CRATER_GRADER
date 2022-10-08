#include <gtest/gtest.h>
#include <mapping/map.hpp>
#include <planning/exploration_planner.hpp>

TEST(ExplorationPlannerTest, planExplorationTest) {
  cg::planning::ExplorationPlanner ep;
  auto agent_pose = cg::planning::create_pose2d(0, 0, 0);
  size_t map_height = 10, map_width = 10;
  float map_res = 1.0;
  cg::mapping::Map<float> map(map_height, map_width, map_res);
  auto exploration_waypoints = ep.planExploration(agent_pose, map);
  EXPECT_GT(exploration_waypoints.size(), 0ul);
  double center_x = map_width * map_res / 2.0;
  double center_y = map_height * map_res / 2.0;
  double expected_squared_radius =
      (exploration_waypoints[0].pt.x-center_x) * (exploration_waypoints[0].pt.x-center_x) +
      (exploration_waypoints[0].pt.y-center_y) * (exploration_waypoints[0].pt.y-center_y);
  for (unsigned long i = 0; i < exploration_waypoints.size(); ++i) {
    // Verify point is inside the map
    EXPECT_TRUE(map.validPoint(exploration_waypoints[i].pt));
    if (i < exploration_waypoints.size()-2) {
      // Verify waypoints are on a circle
      EXPECT_NEAR(
        (exploration_waypoints[i].pt.x-center_x) * (exploration_waypoints[i].pt.x-center_x) +
        (exploration_waypoints[i].pt.y-center_y) * (exploration_waypoints[i].pt.y-center_y),
        expected_squared_radius, 1e-5);
    } else {
      // Verify final waypoints make a roughly 45 degree line
      EXPECT_NEAR(std::fmod(exploration_waypoints[i].yaw, M_PI/4), 0.0, 1e-5);
      if (i < exploration_waypoints.size()-1) {
        double angle = std::atan2(
          exploration_waypoints[i+1].pt.y-exploration_waypoints[i].pt.y,
          exploration_waypoints[i+1].pt.x-exploration_waypoints[i].pt.x
        );
        EXPECT_NEAR(angle, exploration_waypoints[i].yaw, 1e-5);
      }
    }
  }
}

TEST(ExplorationPlannerTest, getGoalPoseTest) {
  cg::planning::ExplorationPlanner ep;
  auto agent_pose = cg::planning::create_pose2d(0, 0, 0);
  size_t map_height = 10, map_width = 10;
  float map_res = 1.0;
  cg::mapping::Map<float> map(map_height, map_width, map_res);

  auto exploration_waypoints = ep.planExploration(agent_pose, map);

  auto pose = ep.getGoalPose(agent_pose, map);
  EXPECT_NEAR(pose.pt.x, exploration_waypoints[0].pt.x, 1e-5);
  EXPECT_NEAR(pose.pt.y, exploration_waypoints[0].pt.y, 1e-5);
  EXPECT_NEAR(pose.yaw, exploration_waypoints[0].yaw, 1e-5);

  pose = ep.getGoalPose(exploration_waypoints[0], map);
  EXPECT_NEAR(pose.pt.x, exploration_waypoints[1].pt.x, 1e-5);
  EXPECT_NEAR(pose.pt.y, exploration_waypoints[1].pt.y, 1e-5);
  EXPECT_NEAR(pose.yaw, exploration_waypoints[1].yaw, 1e-5);
}
