cmake_minimum_required(VERSION 3.8)
project(cg_visualization)

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
find_package(cg_msgs)
find_package(visualization_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Find package header files
include_directories(include)

# Build nodes
add_executable(marker_viz_node src/marker_viz_main.cpp
  src/marker_viz_node.cpp
)
ament_target_dependencies(marker_viz_node
rclcpp
cg_msgs
visualization_msgs
nav_msgs
geometry_msgs
)

# Install targets
install(TARGETS marker_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
