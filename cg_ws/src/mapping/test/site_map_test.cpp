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

TEST(MyTestFixture, toMsgTest) {
  size_t cHeight = 2;
  size_t cWidth = 2;
  float cResolution = 1.0;

  cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);
  cg_msgs::msg::SiteMap map_msg = siteMapTest.toMsg();
  EXPECT_EQ(map_msg.height_map, siteMapTest.getHeightMap());
}

TEST(MyTestFixture, setHeightMapFromMsgTest) {
  size_t cHeight = 2;
  size_t cWidth = 2;
  float cResolution = 1.0;

  cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);
  cg_msgs::msg::SiteMap map_msg;
  map_msg.height_map = {0.0, 1.0, 2.0, 3.0};
  siteMapTest.setHeightMapFromMsg(map_msg);
  EXPECT_EQ(map_msg.height_map, siteMapTest.getHeightMap());
}