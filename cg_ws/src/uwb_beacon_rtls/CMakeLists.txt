cmake_minimum_required(VERSION 3.8)
project(uwb_beacon_rtls)

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
find_package(cg_msgs REQUIRED)

include_directories(include)

# build nodes
add_executable(uwb_beacon_rtls_node src/uwb_beacon_rtls_main.cpp
  src/uwb_beacon_rtls_node.cpp
)
ament_target_dependencies(uwb_beacon_rtls_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cg_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Install targets
install(TARGETS uwb_beacon_rtls_node
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
