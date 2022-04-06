cmake_minimum_required(VERSION 3.8)
project(imu)

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
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

# Add third-party VectorNav Library
add_subdirectory(vectornav/cpp)
include_directories(include vectornav/cpp/include)

# build nodes
add_executable(imu_node src/imu_main.cpp
  src/vn_imu_node.cpp
)
ament_target_dependencies(imu_node
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(
  imu_node
  libvncxx
)
add_executable(imu_base_link_conversion_node src/imu_base_link_conversion_main.cpp
  src/imu_base_link_conversion_node.cpp
)
ament_target_dependencies(imu_base_link_conversion_node
  rclcpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS imu_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS imu_base_link_conversion_node
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
