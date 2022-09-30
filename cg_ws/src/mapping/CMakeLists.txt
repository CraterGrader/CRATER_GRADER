cmake_minimum_required(VERSION 3.8)
project(mapping)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

include_directories(include)

# build nodes
# add_executable(point_cloud_registration src/point_cloud_registration_main.cpp
#   src/point_cloud_registration_node.cpp
# )
# ament_target_dependencies(point_cloud_registration
#   rclcpp
#   rclcpp_components
#   pcl_ros
#   sensor_msgs
#   tf2
#   tf2_ros
#   tf2_eigen
# )

# # build nodes
# add_executable(point_cloud_registration_but_fancier src/point_cloud_registration_main_but_fancier.cpp
#   src/point_cloud_registration_node_but_fancier.cpp
# )
# ament_target_dependencies(point_cloud_registration_but_fancier
#   rclcpp
#   rclcpp_components
#   pcl_ros
#   sensor_msgs
#   tf2
#   tf2_ros
#   tf2_eigen
# )

add_executable(terrain_filtering src/terrain_filtering_main.cpp
  src/terrain_filtering_node.cpp
)
ament_target_dependencies(terrain_filtering
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  tf2
  geometry_msgs
  pcl_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
)

add_executable(site_map src/site_map_main.cpp
  src/site_map_node.cpp
  include/mapping/site_map.hpp
  src/site_map.cpp
)

ament_target_dependencies(site_map
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2_ros
  tf2
  geometry_msgs
  pcl_ros
  tf2_eigen
  tf2_geometry_msgs
  tf2_sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(site_map_test
        test/site_map_test.cpp
        include/mapping/site_map.hpp
        src/site_map.cpp
        )
endif()


# Install targets
# install(TARGETS point_cloud_registration
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(TARGETS point_cloud_registration_but_fancier
#   DESTINATION lib/${PROJECT_NAME}
# )

install(TARGETS terrain_filtering
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS site_map
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
