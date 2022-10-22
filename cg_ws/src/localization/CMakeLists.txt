cmake_minimum_required(VERSION 3.8)
project(localization)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cg_msgs)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3)

# Add Eigen3 Library
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Find package head files
include_directories(include)

# Build nodes
add_executable(slip_estimate_node src/slip_estimate_main.cpp
  src/slip_estimate_node.cpp
)
ament_target_dependencies(slip_estimate_node
rclcpp
cg_msgs
nav_msgs
geometry_msgs
)

# Add a custom library, and link the library with the executable
add_library(kalman_filter_linear src/kalman_filter_linear.cpp)
target_link_libraries(slip_estimate_node kalman_filter_linear)

# build nodes
# add_executable(beacon_transformer src/beacon_transformer_main.cpp
#   src/beacon_transformer.cpp
# )
# ament_target_dependencies(beacon_transformer
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   Eigen3
# )
# target_link_libraries(
#   beacon_transformer
# )

# build nodes
add_executable(ts_prism_transformer src/ts_prism_transformer_main.cpp
  src/ts_prism_transformer.cpp
)
ament_target_dependencies(ts_prism_transformer
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
)
target_link_libraries(
  ts_prism_transformer
)

# Code Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS slip_estimate_node
  DESTINATION lib/${PROJECT_NAME})

# Install targets
# install(TARGETS beacon_transformer
#   DESTINATION lib/${PROJECT_NAME}
# )
install(TARGETS ts_prism_transformer
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

# Create the package
ament_package()
