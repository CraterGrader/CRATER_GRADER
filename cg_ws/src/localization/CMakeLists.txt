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
)

# Code Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install targets
install(TARGETS slip_estimate_node
  DESTINATION lib/${PROJECT_NAME})

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
