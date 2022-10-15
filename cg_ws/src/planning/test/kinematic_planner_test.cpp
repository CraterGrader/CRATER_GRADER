#include <gtest/gtest.h>
#include <planning/kinematic_planner.hpp>
#include <planning/common.hpp>


// Test generatePath()
TEST(KinematicPlannerTest, Test_generatePathForward) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.01);

  std::vector<cg_msgs::msg::Pose2D> path;
  cg_msgs::msg::Pose2D agent_pose{cg::planning::create_pose2d(5, 5, 0)};
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(6, 5, 0)};

  std::vector<float> cells(10000, 1);
  cg::mapping::Map<float> map{static_cast<size_t>(100), 
                      static_cast<size_t>(100), 
                      static_cast<float>(0.1),
                      cells};
  
  kinematic_planner.generatePath(path, agent_pose, goal_pose, map);

  std::cout << "Path: \n";
  for (auto pose: path) {
    std::cout << pose.pt.x << " " << pose.pt.y << " " << pose.yaw << std::endl;
  }

  std::cout << "Path: size" << path.size() << std::endl;
  EXPECT_EQ(static_cast<int>(path.size()), static_cast<int>(1.0f/kinematic_planner.getTrajectoryResolution()));
  EXPECT_NEAR(path.back().pt.x, goal_pose.pt.x, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().pt.y, goal_pose.pt.y, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().yaw, goal_pose.yaw, kinematic_planner.getPoseYawEqualityThreshold());

}


// Test generatePath()
TEST(KinematicPlannerTest, Test_generatePathForwardMultiple) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.01);
  kinematic_planner.setTopographyWeight(1.0);

  std::vector<cg_msgs::msg::Pose2D> path;
  cg_msgs::msg::Pose2D agent_pose{cg::planning::create_pose2d(5, 5, 0)};
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(7, 5, 0)};

  std::vector<float> cells(10000, 1);
  cg::mapping::Map<float> map{static_cast<size_t>(100), 
                      static_cast<size_t>(100), 
                      static_cast<float>(0.1),
                      cells};
  
  kinematic_planner.generatePath(path, agent_pose, goal_pose, map);

  std::cout << "Path: \n";
  for (auto pose: path) {
    std::cout << pose.pt.x << " " << pose.pt.y << " " << pose.yaw << std::endl;
  }

  std::cout << "Path: size" << path.size() << std::endl;
  EXPECT_EQ(static_cast<int>(path.size()), static_cast<int>(2.0f/kinematic_planner.getTrajectoryResolution()));
  EXPECT_NEAR(path.back().pt.x, goal_pose.pt.x, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().pt.y, goal_pose.pt.y, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().yaw, goal_pose.yaw, kinematic_planner.getPoseYawEqualityThreshold());

}


// Test generatePath()
TEST(KinematicPlannerTest, Test_generatePathBackwards) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.01);

  std::vector<cg_msgs::msg::Pose2D> path;
  cg_msgs::msg::Pose2D agent_pose{cg::planning::create_pose2d(5, 5, 0)};
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(4, 5, 0)};

  std::vector<float> cells(10000, 1);
  cg::mapping::Map<float> map{static_cast<size_t>(100), 
                      static_cast<size_t>(100), 
                      static_cast<float>(0.1),
                      cells};
  
  kinematic_planner.generatePath(path, agent_pose, goal_pose, map);

  std::cout << "Path: \n";
  for (auto pose: path) {
    std::cout << pose.pt.x << " " << pose.pt.y << " " << pose.yaw << std::endl;
  }

  std::cout << "Path: size" << path.size() << std::endl;
  EXPECT_EQ(static_cast<int>(path.size()), static_cast<int>(1.0f/kinematic_planner.getTrajectoryResolution()));
  EXPECT_NEAR(path.back().pt.x, goal_pose.pt.x, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().pt.y, goal_pose.pt.y, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().yaw, goal_pose.yaw, kinematic_planner.getPoseYawEqualityThreshold());

}


// Test generatePath()
TEST(KinematicPlannerTest, Test_generatePathCurved) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.05);
  kinematic_planner.setPoseYawEqualityThreshold(cg::planning::deg2rad(5));

  std::vector<cg_msgs::msg::Pose2D> path;
  cg_msgs::msg::Pose2D agent_pose{cg::planning::create_pose2d(5, 5, 0)};
  std::cout << "Desired rad: " << cg::planning::deg2rad(30) << std::endl;
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(6, 6, cg::planning::deg2rad(30))};

  std::vector<float> cells(10000, 1);
  cg::mapping::Map<float> map{static_cast<size_t>(100), 
                      static_cast<size_t>(100), 
                      static_cast<float>(0.1),
                      cells};
  
  kinematic_planner.generatePath(path, agent_pose, goal_pose, map);

  std::cout << "Path: \n";
  for (auto pose: path) {
    std::cout << pose.pt.x << " " << pose.pt.y << " " << pose.yaw << std::endl;
  }

  std::cout << "Path size: " << path.size() << std::endl;
  EXPECT_NEAR(path.back().pt.x, goal_pose.pt.x, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().pt.y, goal_pose.pt.y, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().yaw, goal_pose.yaw, kinematic_planner.getPoseYawEqualityThreshold());

}


// Test generatePath()
TEST(KinematicPlannerTest, Test_generatePathPreciseCurved) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.05);
  kinematic_planner.setPoseYawEqualityThreshold(cg::planning::deg2rad(5));
  kinematic_planner.setTurnRadiiResolutuion(0.2f); // Finer lattice resolution

  std::vector<cg_msgs::msg::Pose2D> path;
  cg_msgs::msg::Pose2D agent_pose{cg::planning::create_pose2d(5, 5, 0)};
  std::cout << "Desired rad: " << cg::planning::deg2rad(30) << std::endl;
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(2, 2, cg::planning::deg2rad(90))};

  std::vector<float> cells(10000, 1);
  cg::mapping::Map<float> map{static_cast<size_t>(100), 
                      static_cast<size_t>(100), 
                      static_cast<float>(0.1),
                      cells};
  
  kinematic_planner.generatePath(path, agent_pose, goal_pose, map);

  std::cout << "Precise Path: \n";
  std::cout << "Path: size: " << path.size() << std::endl;
  for (auto pose: path) {
    std::cout << pose.pt.x << " " << pose.pt.y << " " << pose.yaw << std::endl;
  }

  EXPECT_NEAR(path.back().pt.x, goal_pose.pt.x, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().pt.y, goal_pose.pt.y, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().yaw, goal_pose.yaw, kinematic_planner.getPoseYawEqualityThreshold());

}

// Test generatePath()
TEST(KinematicPlannerTest, Test_generatePathPreciseObstacle) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.05);

  std::vector<cg_msgs::msg::Pose2D> path;
  cg_msgs::msg::Pose2D agent_pose{cg::planning::create_pose2d(0.5, 0.5, 0)};
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(3.0, 0.5, 0)};

  std::vector<float> cells{
    1, 10, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    };
  cg::mapping::Map<float> map{static_cast<size_t>(4), 
                      static_cast<size_t>(4), 
                      static_cast<float>(1),
                      cells};
  
  kinematic_planner.generatePath(path, agent_pose, goal_pose, map);

  std::cout << "Obstacle Path: \n";
  std::cout << "Path: size" << path.size() << std::endl;
  for (auto pose: path) {
    std::cout << pose.pt.x << " " << pose.pt.y << " " << pose.yaw << std::endl;
  }

  EXPECT_NEAR(path.back().pt.x, goal_pose.pt.x, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().pt.y, goal_pose.pt.y, kinematic_planner.getPosePositionEqualityThreshold());
  EXPECT_NEAR(path.back().yaw, goal_pose.yaw, kinematic_planner.getPoseYawEqualityThreshold());

}


// Test getClosestTrajectoryPoseToGoal()
TEST(KinematicPlannerTest, Test_getClosestTrajectoryPoseToGoal)
{

  cg::planning::KinematicPlanner kinematic_planner;

  cg_msgs::msg::Pose2D expected_closest{cg::planning::create_pose2d(0, 1, 0)};

  std::vector<cg_msgs::msg::Pose2D> trajectory{
    cg::planning::create_pose2d(0, 0, 0),
    expected_closest,
    cg::planning::create_pose2d(0, 4, 0)};

  cg_msgs::msg::Pose2D goalPose{cg::planning::create_pose2d(0, 2, 0)};

  std::pair<cg_msgs::msg::Pose2D, int> closest_traj;
  closest_traj = kinematic_planner.getClosestTrajectoryPoseToGoal(
    trajectory,
    goalPose
  );

  float absolute_range = 1e-6f;
  EXPECT_NEAR(expected_closest.pt.x, closest_traj.first.pt.x, absolute_range);
  EXPECT_NEAR(expected_closest.pt.y, closest_traj.first.pt.y, absolute_range);
  EXPECT_NEAR(1, closest_traj.second, absolute_range);
}


// Test samePoseWithinThresh
TEST(KinematicPlannerTest, Test_samePoseWithinThresh)
{

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setPosePositionEqualityThreshold(0.01f);
  kinematic_planner.setPoseYawEqualityThreshold(0.05f);

  cg_msgs::msg::Pose2D pose1{cg::planning::create_pose2d(0, 1, 0)};
  cg_msgs::msg::Pose2D pose2{cg::planning::create_pose2d(0, 1.02, 0)};
  cg_msgs::msg::Pose2D pose3{cg::planning::create_pose2d(0, 1.005, 0)};
  cg_msgs::msg::Pose2D pose4{cg::planning::create_pose2d(0, 1, 0.02)};
  cg_msgs::msg::Pose2D pose5{cg::planning::create_pose2d(0, 1, 0.06)};

  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose2), false);
  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose3), true);
  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose4), true);
  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose2), false);

}


// Test generateLatticeArm
TEST(KinematicPlannerTest, Test_generateLatticeArm)
{

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setMaxTrajectoryLength(1.0f);
  kinematic_planner.setTrajectoryResolution(0.1f);
  float absolute_range = 1e-6f;

  // Straight forward lattice arm
  std::vector<cg_msgs::msg::Pose2D> lattice_arm_staight_forward = kinematic_planner.generateLatticeArm(0, true, true);

  EXPECT_EQ(static_cast<int>(lattice_arm_staight_forward.size()), 10);
  EXPECT_NEAR(lattice_arm_staight_forward[0].pt.x, 0.1, absolute_range);
  EXPECT_NEAR(lattice_arm_staight_forward[9].pt.x, 1.0, absolute_range);
  EXPECT_NEAR(lattice_arm_staight_forward[0].pt.y, 0.0, absolute_range);
  EXPECT_NEAR(lattice_arm_staight_forward[9].pt.y, 0.0, absolute_range);

  // Straight backward lattice arm
  std::vector<cg_msgs::msg::Pose2D> lattice_arm_straight_backward = kinematic_planner.generateLatticeArm(0, false, true);

  EXPECT_NEAR(lattice_arm_straight_backward[0].pt.x, -0.1, absolute_range);
  EXPECT_NEAR(lattice_arm_straight_backward[9].pt.x, -1.0, absolute_range);
  EXPECT_NEAR(lattice_arm_straight_backward[0].pt.y, 0.0, absolute_range);
  EXPECT_NEAR(lattice_arm_straight_backward[9].pt.y, 0.0, absolute_range);

  // Forwards right lattice arm
  std::vector<cg_msgs::msg::Pose2D> lattice_arm_right_forward = kinematic_planner.generateLatticeArm(1, true, true);

  EXPECT_EQ(static_cast<int>(lattice_arm_right_forward.size()), 10);
  EXPECT_NEAR(lattice_arm_right_forward[0].pt.x, cos(M_PI/2 - 0.1), absolute_range);
  EXPECT_NEAR(lattice_arm_right_forward[0].pt.y, sin(M_PI/2 - 0.1) - 1, absolute_range);

  EXPECT_NEAR(lattice_arm_right_forward[9].pt.x, cos(M_PI/2 - 1), absolute_range);
  EXPECT_NEAR(lattice_arm_right_forward[9].pt.y, sin(M_PI/2 - 1) - 1, absolute_range);

  // Back left lattice arm
  std::vector<cg_msgs::msg::Pose2D> lattice_arm_left_backwards = kinematic_planner.generateLatticeArm(1, false, false);

  EXPECT_EQ(static_cast<int>(lattice_arm_left_backwards.size()), 10);
  EXPECT_NEAR(lattice_arm_left_backwards[0].pt.x, cos(-M_PI/2 - 0.1), absolute_range);
  EXPECT_NEAR(lattice_arm_left_backwards[0].pt.y, sin(-M_PI/2 - 0.1) + 1, absolute_range);

  EXPECT_NEAR(lattice_arm_left_backwards[9].pt.x, cos(-M_PI/2 - 1), absolute_range);
  EXPECT_NEAR(lattice_arm_left_backwards[9].pt.y, sin(-M_PI/2 - 1) + 1, absolute_range);

}


// Test generateBaseLattice
TEST(KinematicPlannerTest, Test_generateBaseLattice)
{

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setTurnRadiiMax(2.0f);
  kinematic_planner.setTurnRadiiMin(1.0f);
  kinematic_planner.setTurnRadiiResolutuion(0.5f);
  kinematic_planner.setMaxTrajectoryLength(1.0f);
  kinematic_planner.setTrajectoryResolution(0.1f);
  float absolute_range = 1e-6f;

  std::vector<std::vector<cg_msgs::msg::Pose2D>> base_lattice = kinematic_planner.generateBaseLattice();

  for (auto traj: base_lattice) {
      EXPECT_EQ(static_cast<int>(traj.size()), 10);
  }

  std::vector<cg_msgs::msg::Pose2D> lattice_arm_right_forward = base_lattice[0];
  EXPECT_EQ(static_cast<int>(lattice_arm_right_forward.size()), 10);
  EXPECT_NEAR(lattice_arm_right_forward[0].pt.x, cos(M_PI/2 - 0.1), absolute_range);
  EXPECT_NEAR(lattice_arm_right_forward[0].pt.y, sin(M_PI/2 - 0.1) - 1, absolute_range);

  EXPECT_NEAR(lattice_arm_right_forward[9].pt.x, cos(M_PI/2 - 1), absolute_range);
  EXPECT_NEAR(lattice_arm_right_forward[9].pt.y, sin(M_PI/2 - 1) - 1, absolute_range);

}


// Test transformLatticeToPose
TEST(KinematicPlannerTest, Test_transformLatticeToPose) {

  cg::planning::KinematicPlanner kinematic_planner;
  cg_msgs::msg::Pose2D current_pose{cg::planning::create_pose2d(3, 3, M_PI/4)};
  float absolute_range = 1e-6f;
  
  std::vector<cg_msgs::msg::Pose2D> lattice_arm{
    cg::planning::create_pose2d(0, 0, 0),
    cg::planning::create_pose2d(0, 1, 0),
    cg::planning::create_pose2d(0, 2, M_PI/4)};
  std::vector<std::vector<cg_msgs::msg::Pose2D>> lattice{lattice_arm};

  auto transformed_lattice = kinematic_planner.transformLatticeToPose(lattice, current_pose);

  EXPECT_EQ(static_cast<int>(transformed_lattice.size()), 1);
  EXPECT_EQ(static_cast<int>(transformed_lattice[0].size()), 3);

  // First point
  EXPECT_NEAR(transformed_lattice[0][0].pt.x, 3, absolute_range);
  EXPECT_NEAR(transformed_lattice[0][0].pt.y, 3, absolute_range);
  EXPECT_NEAR(transformed_lattice[0][0].yaw, M_PI/4, absolute_range);

  // Last point
  EXPECT_NEAR(transformed_lattice[0][2].pt.x, 3 - 2*sin(M_PI/4), absolute_range);
  EXPECT_NEAR(transformed_lattice[0][2].pt.y, 3 + 2*cos(M_PI/4), absolute_range);
  EXPECT_NEAR(transformed_lattice[0][2].yaw, M_PI/2, absolute_range);
}


// isValidTrajectory() need map input
TEST(KinematicPlannerTest, Test_isValidTrajectory) {

  cg::planning::KinematicPlanner kinematic_planner;
  cg::mapping::Map<float> map{static_cast<size_t>(2), 
                      static_cast<size_t>(2), 
                      static_cast<float>(1)};
  std::vector<cg_msgs::msg::Pose2D> trajectory;

  trajectory.push_back(cg::planning::create_pose2d(1, 1, 0));
  EXPECT_TRUE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.push_back(cg::planning::create_pose2d(1, 1, M_PI/2));
  EXPECT_TRUE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.push_back(cg::planning::create_pose2d(0, 0, 0));
  EXPECT_TRUE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.push_back(cg::planning::create_pose2d(0, 0, M_PI/2));
  EXPECT_TRUE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.push_back(cg::planning::create_pose2d(2, 2, 0));
  EXPECT_FALSE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.pop_back();
  EXPECT_TRUE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.push_back(cg::planning::create_pose2d(0, -0.01, 0));
  EXPECT_FALSE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.pop_back();
  trajectory.push_back(cg::planning::create_pose2d(-0.01, 0, 0));
  EXPECT_FALSE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.pop_back();
  trajectory.push_back(cg::planning::create_pose2d(1, 2, 0));
  EXPECT_FALSE(kinematic_planner.isValidTrajectory(trajectory, map));

  trajectory.pop_back();
  trajectory.push_back(cg::planning::create_pose2d(2, 1, 0));
  EXPECT_FALSE(kinematic_planner.isValidTrajectory(trajectory, map));
}


// Test calculateTopographyCost
TEST(KinematicPlannerTest, Test_calculateTopographyCost) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setTrajectoryResolution(1.0f);
  float absolute_range = 1e-6f;
  std::vector<float> cells{1, 2, 3, 4};
  cg::mapping::Map<float> map{static_cast<size_t>(2), 
                      static_cast<size_t>(2), 
                      static_cast<float>(1),
                      cells};
  std::vector<cg_msgs::msg::Pose2D> trajectory_1{
    cg::planning::create_pose2d(0, 0, 0)};
  
  auto cost_1 = kinematic_planner.calculateTopographyCost(trajectory_1, map);
  EXPECT_NEAR(cost_1, kinematic_planner.getTopographyWeight() * 1, absolute_range);

  std::vector<cg_msgs::msg::Pose2D> trajectory_2{
    cg::planning::create_pose2d(0.5, 0.5, 0),
    cg::planning::create_pose2d(1.5, 0.5, 0),
    cg::planning::create_pose2d(0.5, 1.5, 0),
    cg::planning::create_pose2d(1.5, 1.5, 0)};

  auto cost_2 = kinematic_planner.calculateTopographyCost(trajectory_2, map);
  EXPECT_NEAR(cost_2, kinematic_planner.getTopographyWeight() * 10, absolute_range);

}


// Test calculateTopographyCost
TEST(KinematicPlannerTest, Test_calculateTopographyCostObstacle) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setTrajectoryResolution(1.0f);
  float absolute_range = 1e-6f;

  std::vector<float> cells{
    1, 10, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    1, 1, 1, 1,
    };
  cg::mapping::Map<float> map{static_cast<size_t>(4), 
                      static_cast<size_t>(4), 
                      static_cast<float>(1),
                      cells};

  std::vector<cg_msgs::msg::Pose2D> trajectory_1{
    cg::planning::create_pose2d(0, 0, 0)};
  
  auto cost_1 = kinematic_planner.calculateTopographyCost(trajectory_1, map);
  EXPECT_NEAR(cost_1, kinematic_planner.getTopographyWeight() * 1, absolute_range);

  std::vector<cg_msgs::msg::Pose2D> trajectory_2{
    cg::planning::create_pose2d(0.5, 0.5, 0),
    cg::planning::create_pose2d(1.5, 0.5, 0),
    cg::planning::create_pose2d(2.5, 0.5, 0),
    cg::planning::create_pose2d(3.5, 0.5, 0)};

  auto cost_2 = kinematic_planner.calculateTopographyCost(trajectory_2, map);
  EXPECT_NEAR(cost_2, kinematic_planner.getTopographyWeight() * 13, absolute_range);

}


// Test calculateTopographyCost
TEST(KinematicPlannerTest, Test_calculateTopographyCostSmallResolution) {

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.setTrajectoryResolution(0.05f);
  float absolute_range = 1e-6f;
  std::vector<float> cells{1, 2, 3, 4};
  cg::mapping::Map<float> map{static_cast<size_t>(2), 
                      static_cast<size_t>(2), 
                      static_cast<float>(1),
                      cells};
  std::vector<cg_msgs::msg::Pose2D> trajectory_1{
    cg::planning::create_pose2d(0, 0, 0)};
  
  auto cost_1 = kinematic_planner.calculateTopographyCost(trajectory_1, map);
  EXPECT_NEAR(cost_1, kinematic_planner.getTopographyWeight() * 0.05, absolute_range);

  std::vector<cg_msgs::msg::Pose2D> trajectory_2{
    cg::planning::create_pose2d(0.55, 0.5, 0),
    cg::planning::create_pose2d(0.6, 0.5, 0),
    cg::planning::create_pose2d(0.65, 0.5, 0),
    cg::planning::create_pose2d(0.7, 0.5, 0),
    cg::planning::create_pose2d(0.75, 0.5, 0),
    cg::planning::create_pose2d(0.8, 0.5, 0),
    cg::planning::create_pose2d(0.85, 0.5, 0),
    cg::planning::create_pose2d(0.9, 0.5, 0),
    cg::planning::create_pose2d(0.95, 0.5, 0),
    cg::planning::create_pose2d(1.0, 0.5, 0)};

  float cost_2 = kinematic_planner.calculateTopographyCost(trajectory_2, map);
  EXPECT_NEAR(cost_2, kinematic_planner.getTopographyWeight() * 0.55f, absolute_range);

}


// Test trajectoriesHeuristic
TEST(KinematicPlannerTest, Test_trajectoriesHeuristic) {

  cg::planning::KinematicPlanner kinematic_planner;
  float absolute_range = 1e-6f;
  cg_msgs::msg::Pose2D goal_pose{cg::planning::create_pose2d(1, 1, 0)};

  std::vector<cg_msgs::msg::Pose2D> trajectory_1{
    cg::planning::create_pose2d(0, 0, 0)};
  std::vector<cg_msgs::msg::Pose2D> trajectory_2{
    cg::planning::create_pose2d(-1, -1, 0),
    cg::planning::create_pose2d(0, 0, 0)};
  std::vector<cg_msgs::msg::Pose2D> trajectory_3{
    cg::planning::create_pose2d(0, 0, 0),
    cg::planning::create_pose2d(1, 0, 0),
    cg::planning::create_pose2d(2, 0, 0),
    cg::planning::create_pose2d(3, 0, 0)};

  std::vector<std::vector<cg_msgs::msg::Pose2D>> trajectories{
    trajectory_1, trajectory_2, trajectory_3};

  auto trajectory_heuristic = kinematic_planner.trajectoriesHeuristic(trajectories, goal_pose);
  EXPECT_NEAR(trajectory_heuristic[0], trajectory_heuristic[1], absolute_range);
  EXPECT_NEAR(trajectory_heuristic[1], sqrt(2), absolute_range);
  EXPECT_NEAR(trajectory_heuristic[2], sqrt(5), absolute_range);

}
