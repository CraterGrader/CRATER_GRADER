// #include <gtest/gtest.h>
// #include <mapping/site_map.hpp>

// TEST(HelloWorldTest, helloWorldTest) {
//     EXPECT_TRUE(true);
// }

// TEST(MapTestConstructorTest, funcitonalityCheck) {

//   size_t cHeight = 2;
//   size_t cWidth = 2; 
//   float cResolution = 1.0;
  
//   cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);

//   // resolution test 
//   float getRes = siteMapTest.getResolution();
//   float resSet = 1.0;
//   EXPECT_EQ(getRes,resSet);

// }

// TEST(MapUtilTest, UtilfuncitonalityCheck) {

//   float x = 0.2f;
//   float y = 1.5f;
//   size_t width = 50;
//   size_t height = 50;
//   float resolution = 0.1f;

//   bool inMap = cg::mapping::pointInMap(x, y, width, height, resolution);

//   ASSERT_TRUE(inMap);
// }

// TEST(MyTestFixture, toMsgTest) {
//   size_t cHeight = 2;
//   size_t cWidth = 2;
//   float cResolution = 1.0;

//   cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);
//   cg_msgs::msg::SiteMap map_msg = siteMapTest.toMsg();
//   EXPECT_EQ(map_msg.height_map, siteMapTest.getHeightMap());
// }

// TEST(MyTestFixture, setHeightMapFromMsgTest) {
//   size_t cHeight = 2;
//   size_t cWidth = 2;
//   float cResolution = 1.0;

//   cg::mapping::SiteMap siteMapTest(cHeight, cWidth, cResolution);
//   cg_msgs::msg::SiteMap map_msg;
//   map_msg.height_map = {0.0, 1.0, 2.0, 3.0};
//   siteMapTest.setHeightMapFromMsg(map_msg);
//   EXPECT_EQ(map_msg.height_map, siteMapTest.getHeightMap());
// }

// TEST(MyTestFixture, siteVsSiteTest0) {

//   float thresh = 0.1;

//   std::vector<float> map1{0.1, 0.0, 0.2, 0.4, 0.6, 0.1, 100.0, 2.0, 1.0};
//   std::vector<float> map2{0.15, 0.05, 0.25, 0.43, 0.62, 0.11, 100.02, 2.03, 1.04};

//   bool mapIsclose = cg::mapping::mapSimilarityWithinThreshold(map1, map2, thresh);

//   EXPECT_EQ(mapIsclose, true);
// }

// TEST(MyTestFixture, siteVsSiteTest1) {

//   float thresh2 = 0.000001f;

//   std::vector<float> map11{0.1, 0.0, 0.2, 0.4, 0.6, 0.1, 100.0, 2.0, 1.0};
//   std::vector<float> map21{0.15, 0.05, 0.25, 0.43, 0.62, 0.11, 100.02, 2.03, 1.04};

//   bool mapIsclose = cg::mapping::mapSimilarityWithinThreshold(map11, map21, thresh2);

//   EXPECT_EQ(mapIsclose, false);
// }
