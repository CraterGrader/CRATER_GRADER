#include <gtest/gtest.h>
#include <mapping/map.hpp>

TEST(MapTestConstructorTest, emptyInit) {

  size_t height = 2;
  size_t width = 2; 
  float resolution = 1.0;
  
  cg::mapping::Map empty_map(height, width, resolution);

  // resolution test 
  float getRes = empty_map.getResolution();
  EXPECT_EQ(getRes,resolution);
}
