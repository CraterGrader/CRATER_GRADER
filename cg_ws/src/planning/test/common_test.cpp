#include <gtest/gtest.h>
#include <planning/common.hpp>

TEST(CommonTest, euclideanTest) {
  float expected = 1.41;
  cg::planning::Point2D pt1 = {.x = 0.0, .y = 0.0};
  cg::planning::Point2D pt2 = {.x = 1.0, .y = 1.0};
  float actual = cg::planning::euclidean_distance(pt1, pt2);
  float absolute_range = 0.01;
  EXPECT_NEAR(expected, actual, absolute_range);
}
