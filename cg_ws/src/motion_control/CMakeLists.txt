cmake_minimum_required(VERSION 3.8)
project(motion_control)

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
find_package(std_msgs REQUIRED)
find_package(cg_msgs REQUIRED)

# Find package header files
include_directories(include)

# Build nodes
add_executable(cmd_mux src/cmd_mux_main.cpp
  src/cmd_mux_node.cpp
)
ament_target_dependencies(cmd_mux
  rclcpp
  std_msgs
  cg_msgs
)

# Code testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS cmd_mux
  DESTINATION lib/${PROJECT_NAME}
)

# Install Directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Create the package
ament_package()
