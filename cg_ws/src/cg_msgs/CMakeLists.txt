cmake_minimum_required(VERSION 3.8)
project(cg_msgs)

find_package(rosidl_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  # Add custom interfaces here
  "msg/ActuatorCommand.idl"
  "msg/BeaconTag.idl"
  "msg/BeaconMultiTag.idl"
  "msg/EncoderTelemetry.idl"
  "msg/Point2D.idl"
  "msg/Pose2D.idl"
  "msg/Slip.idl"
  "msg/MuxMode.idl"
  "msg/ActuatorState.idl"
  "msg/SiteMap.idl"
  "msg/Trajectory.idl"
  "srv/SiteMap.srv"
  "srv/UpdateTrajectory.srv"
  "srv/EnableWorksystem.srv"
  "srv/SaveMap.srv"
  # Implementation-specific messages
  DEPENDENCIES builtin_interfaces std_msgs
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
