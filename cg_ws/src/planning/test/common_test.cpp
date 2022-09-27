#include <gtest/gtest.h>
#include <planning/common.hpp>

TEST(CommonTest, createPoint2DTest) {
  double x = 1.0, y = 2.0;
  cg_msgs::msg::Point2D pt = cg::planning::create_point2d(x, y);
  float absolute_range = 1e-5;
  EXPECT_NEAR(x, pt.x, absolute_range);
  EXPECT_NEAR(y, pt.y, absolute_range);
}

TEST(CommonTest, createPose2DTest) {
  double x = 1.0, y = 2.0, yaw = 3.0;
  cg_msgs::msg::Pose2D pose = cg::planning::create_pose2d(x, y, yaw);
  float absolute_range = 1e-5;
  EXPECT_NEAR(x, pose.pt.x, absolute_range);
  EXPECT_NEAR(y, pose.pt.y, absolute_range);
  EXPECT_NEAR(yaw, pose.yaw, absolute_range);
}

TEST(CommonTest, euclideanTest) {
  float expected = 1.41;
  cg_msgs::msg::Point2D pt1 = cg::planning::create_point2d(0.0, 0.0);
  cg_msgs::msg::Point2D pt2 = cg::planning::create_point2d(1.0, 1.0);
  float actual = cg::planning::euclidean_distance(pt1, pt2);
  float absolute_range = 0.01;
  EXPECT_NEAR(expected, actual, absolute_range);
}
