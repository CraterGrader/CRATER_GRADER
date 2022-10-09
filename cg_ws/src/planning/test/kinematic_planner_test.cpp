#include <gtest/gtest.h>
#include <planning/kinematic_planner.hpp>
#include <planning/common.hpp>

TEST(KinematicPlannerTest, helloworld)
{
  float expected = 0.0f;
  float actual = 0.0f;
  float absolute_range = 0.0f;
  EXPECT_NEAR(expected, actual, absolute_range);
}


// Test getClosestTrajectoryPoseToGoal()
TEST(KinematicPlannerTest, Test_getClosestTrajectoryPoseToGoal)
{

  cg::planning::KinematicPlanner kinematic_planner;

  cg_msgs::msg::Pose2D expected_closest{cg::planning::create_pose2d(0,1,0)};

  std::vector<cg_msgs::msg::Pose2D> trajectory{
    cg::planning::create_pose2d(0,0,0),
    expected_closest,
    cg::planning::create_pose2d(0,4,0)};

  cg_msgs::msg::Pose2D goalPose{cg::planning::create_pose2d(0,2,0)};

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
  kinematic_planner.pose_position_equality_threshold = 0.01;
  kinematic_planner.pose_yaw_equality_threshold = 0.05;

  cg_msgs::msg::Pose2D pose1{cg::planning::create_pose2d(0,1,0)};
  cg_msgs::msg::Pose2D pose2{cg::planning::create_pose2d(0,1.02,0)};
  cg_msgs::msg::Pose2D pose3{cg::planning::create_pose2d(0,1.005,0)};
  cg_msgs::msg::Pose2D pose4{cg::planning::create_pose2d(0,1,0.02)};
  cg_msgs::msg::Pose2D pose5{cg::planning::create_pose2d(0,1,0.06)};

  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose2), false);
  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose3), true);
  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose4), true);
  EXPECT_EQ(kinematic_planner.samePoseWithinThresh(pose1, pose2), false);

}

// Test generateLatticeArm
TEST(KinematicPlannerTest, Test_generateLatticeArm)
{

  cg::planning::KinematicPlanner kinematic_planner;
  kinematic_planner.max_trajectory_length = 1.0f;
  kinematic_planner.trajectory_resolution = 0.1f;
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
  kinematic_planner.turn_radii_max = 2;
  kinematic_planner.turn_radii_min = 1;
  kinematic_planner.turn_radii_resolution = 0.5;
  kinematic_planner.max_trajectory_length = 1.0f;
  kinematic_planner.trajectory_resolution = 0.1f;
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

// isValidTrajectory() need map input

// Test calculateTopographyCost

// Test trajectories_heuristic

// Test transformLatticeToPose
