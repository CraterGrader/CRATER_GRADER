cmake_minimum_required(VERSION 3.8)
project(rectify)

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(OpenCV_DIR /usr/include/boost/compute/interop/opencv/)

# find dependencies
#find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

include_directories(include)

# build nodes
add_executable(rectify_node src/rectify_main.cpp
  src/rectify_node.cpp
)
ament_target_dependencies(rectify_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


# Install targets
install(TARGETS rectify_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
#install(DIRECTORY launch
#  DESTINATION share/${PROJECT_NAME}
#)
#install(DIRECTORY config
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
