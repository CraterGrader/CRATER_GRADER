#include <gtest/gtest.h>
#include <mapping/site_map.hpp>

TEST(HelloWorldTest, helloWorldTest) {
    EXPECT_TRUE(true);
}

TEST(MapTestConstructorTest, funcitonalityCheck) {

  size_t cHeight = 2;
  size_t cWidth = 2; 
  float cResolution = 1.0;
  
  cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);

  // resolution test 
  float getRes = siteMapTest.getResolution();
  float resSet = 1.0;
  EXPECT_EQ(getRes,resSet);

}

TEST(MapUtilTest, UtilfuncitonalityCheck) {

  float x = 0.2f;
  float y = 1.5f;
  size_t width = 50;
  size_t height = 50;
  float resolution = 0.1f;

  bool inMap = cg::mapping::pointInMap(x, y, width, height, resolution);

  ASSERT_TRUE(inMap);
}
