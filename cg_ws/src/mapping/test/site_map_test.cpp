#include <gtest/gtest.h>
#include <mapping/site_map.hpp>

TEST(HelloWorldTest, helloWorldTest) {
    EXPECT_TRUE(true);
}

// replace with test fixture if you dare! 

TEST(MyTestFixture, funcitonalityCheck) {

  size_t cHeight = 2;
  size_t cWidth = 2; 
  float cResolution = 1.0;
  
  cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);

  // resolution test 
  float getRes = siteMapTest.getResolution();
  float resSet = 1.0;
  EXPECT_EQ(getRes,resSet);

}

TEST(MyTestFixture, filterCheck) {

  size_t cHeight = 2;
  size_t cWidth = 2; 
  float cResolution = 1.0;
  
  cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);

  cg::mapping::mapPoint pt1; 
  pt1.x = 0.0;
  pt1.y = 0.0;
  pt1.z = 1.0;

  cg::mapping::mapPoint pt2; 
  pt2.x = 0.0;
  pt2.y = 0.0;
  pt2.z = 2.0;

  cg::mapping::mapPoint pt3; 
  pt3.x = 0.0;
  pt3.y = 0.0;
  pt3.z = 3.0;

  std::vector<cg::mapping::mapPoint> newPts{pt1,pt2,pt3};

  siteMapTest.binPts(newPts);

  siteMapTest.updateCellsMean();

  std::vector<float> heightMapGet = siteMapTest.getHeightMap();

  EXPECT_NEAR(heightMapGet[0], 2.0, 0.1);
}

TEST(MyTestFixture, indexCheck) {

  size_t cHeight = 2;
  size_t cWidth = 2; 
  float cResolution = 1.0;
  
  cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);

  cg::mapping::mapPoint pt1; 
  pt1.x = 0.5;
  pt1.y = 0.5;
  pt1.z = 1.0;

  cg::mapping::mapPoint pt2; 
  pt2.x = 1.5;
  pt2.y = 0.5;
  pt2.z = 2.0;

  cg::mapping::mapPoint pt3; 
  pt3.x = 99.5;
  pt3.y = -1.5;
  pt3.z = 3.0;

  cg::mapping::mapPoint pt4; 
  pt4.x = 1.5;
  pt4.y = 1.5;
  pt4.z = 4.0;

  std::vector<cg::mapping::mapPoint> newPts{pt1,pt2,pt3,pt4};

  siteMapTest.binPts(newPts);

  siteMapTest.updateCellsMean();

  std::vector<float> heightMapGet = siteMapTest.getHeightMap();

  EXPECT_NEAR(heightMapGet[0], 1.0, 0.1);
  EXPECT_NEAR(heightMapGet[1], 2.0, 0.1);
  EXPECT_NEAR(heightMapGet[2], 0.0, 0.1);
  EXPECT_NEAR(heightMapGet[3], 4.0, 0.1);
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
