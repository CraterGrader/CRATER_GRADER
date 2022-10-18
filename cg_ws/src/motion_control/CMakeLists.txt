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
find_package(nav_msgs REQUIRED)
find_package(cg_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(planning REQUIRED)
find_package(Eigen3 REQUIRED)

# Add Eigen3 Library
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

# Find package header files
include_directories(include)

# Build nodes
add_executable(cmd_mux src/cmd_mux_main.cpp
  src/cmd_mux_node.cpp
)
ament_target_dependencies(cmd_mux
rclcpp
nav_msgs
cg_msgs
diagnostic_updater
)
add_executable(odom_node src/odom_main.cpp
  src/odom_node.cpp
)
ament_target_dependencies(odom_node
  rclcpp
  nav_msgs
  cg_msgs
  tf2_ros
)
add_executable(worksystem_control_node src/worksystem_control_main.cpp
  src/worksystem_control_node.cpp
  src/longitudinal_controller.cpp
  src/lateral_controller.cpp
  src/pid_controller.cpp
  ../planning/src/common.cpp
)
ament_target_dependencies(worksystem_control_node
  rclcpp
  cg_msgs
  nav_msgs
  planning
)

# Code testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(lateral_controller_test
        test/lateral_controller_test.cpp
        ../planning/src/common.cpp
        src/lateral_controller.cpp
        )
  ament_target_dependencies(lateral_controller_test
    rclcpp
    cg_msgs
    nav_msgs
    planning
  )
endif()

# Install targets
install(TARGETS cmd_mux
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS odom_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS worksystem_control_node
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
