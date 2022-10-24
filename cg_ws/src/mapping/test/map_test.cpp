// #include <gtest/gtest.h>
// #include <mapping/map.hpp>

// TEST(MapWithCSV, write_to_file) {

//   size_t height = 2;
//   size_t width = 2; 
//   float resolution = 1.0;
//   std::vector<float> map_data{0.1, -0.2, 0.0, 0.4};
  
//   cg::mapping::Map<float> map(height, width, resolution, map_data);

//   // Write test
//   std::string filepath = "map_test.csv";
//   map.write_map_to_file(filepath);
//   EXPECT_EQ(1.0,1.0);
// }

// TEST(MapWithCSV, read_from_file) {
  
//   cg::mapping::Map<float> map;

//   // Write test
//   std::string filepath = "/root/CRATER_GRADER/cg_ws/src/mapping/test/50x50_zeros_height_map.csv";
//   map.load_map_from_file(filepath);
//   EXPECT_EQ(2500, map.getCellData().size());
//   EXPECT_EQ(50, map.getHeight());
//   EXPECT_EQ(50, map.getWidth());
//   EXPECT_NEAR(0.1, map.getResolution(), 1e-6);
// }

// TEST(MapTestConstructorTest, emptyInit) {

//   size_t height = 2;
//   size_t width = 2; 
//   float resolution = 1.0;
  
//   cg::mapping::Map<float> empty_map(height, width, resolution);

//   // resolution test 
//   float getRes = empty_map.getResolution();
//   EXPECT_EQ(getRes,resolution);
// }

// TEST(MapTestConstructorTest, default_init)
// {

//   size_t height = 2;
//   size_t width = 2;
//   float resolution = 1.0;

//   cg::mapping::Map<float> default_map;
//   default_map.updateDimensions(height, width, resolution);

//   // Check values
//   size_t get_height = default_map.getHeight();
//   size_t get_width = default_map.getWidth();
//   float get_resolution = default_map.getResolution();
//   EXPECT_EQ(get_height, height);
//   EXPECT_EQ(get_width, width);
//   EXPECT_EQ(get_resolution, resolution);
// }

// TEST(UtilitiesTest, TransformPointIndentity) {

//   cg_msgs::msg::Point2D pt;
//   pt.x = 1;
//   pt.y = 2;

//   cg_msgs::msg::Pose2D source_rel_to_dest_frame;
//   source_rel_to_dest_frame.yaw = 0;
//   source_rel_to_dest_frame.pt.x = 0;
//   source_rel_to_dest_frame.pt.y = 0;

//   cg_msgs::msg::Point2D transformed_pt = cg::mapping::transformPoint(pt, source_rel_to_dest_frame);

//   EXPECT_NEAR(transformed_pt.x, pt.x, 1e-6);
//   EXPECT_NEAR(transformed_pt.y, pt.y, 1e-6);

// }

// TEST(UtilitiesTest, TransformPointOffset) {

//   cg_msgs::msg::Point2D pt;
//   pt.x = 1;
//   pt.y = 2;

//   cg_msgs::msg::Pose2D source_rel_to_dest_frame;
//   source_rel_to_dest_frame.yaw = 0;
//   source_rel_to_dest_frame.pt.x = 1;
//   source_rel_to_dest_frame.pt.y = 1;

//   cg_msgs::msg::Point2D transformed_pt = cg::mapping::transformPoint(pt, source_rel_to_dest_frame);

//   EXPECT_NEAR(transformed_pt.x, 2.0, 1e-6);
//   EXPECT_NEAR(transformed_pt.y, 3.0, 1e-6);

// }

// TEST(UtilitiesTest, TransformPointRotate) {

//   cg_msgs::msg::Point2D pt;
//   pt.x = 1;
//   pt.y = 2;

//   cg_msgs::msg::Pose2D source_rel_to_dest_frame;
//   source_rel_to_dest_frame.yaw = cg::mapping::deg2rad(90);
//   source_rel_to_dest_frame.pt.x = 0;
//   source_rel_to_dest_frame.pt.y = 0;

//   cg_msgs::msg::Point2D transformed_pt = cg::mapping::transformPoint(pt, source_rel_to_dest_frame);

//   EXPECT_NEAR(transformed_pt.x, -2.0, 1e-6);
//   EXPECT_NEAR(transformed_pt.y, 1.0, 1e-6);

// }

// TEST(UtilitiesTest, TransformPointRotateOffset) {

//   cg_msgs::msg::Point2D pt;
//   pt.x = 1;
//   pt.y = 2;

//   cg_msgs::msg::Pose2D source_rel_to_dest_frame;
//   source_rel_to_dest_frame.yaw = cg::mapping::deg2rad(180);
//   source_rel_to_dest_frame.pt.x = 1;
//   source_rel_to_dest_frame.pt.y = 1;

//   cg_msgs::msg::Point2D transformed_pt = cg::mapping::transformPoint(pt, source_rel_to_dest_frame);

//   EXPECT_NEAR(transformed_pt.x, 0.0, 1e-6);
//   EXPECT_NEAR(transformed_pt.y, -1.0, 1e-6);

// }

