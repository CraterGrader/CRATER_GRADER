cmake_minimum_required(VERSION 3.8)
project(planning)

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
find_package(cg_msgs REQUIRED)
find_package(mapping REQUIRED)
find_package(ortools CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED) # viz
find_package(geometry_msgs REQUIRED) # viz
find_package(tf2_ros REQUIRED) # viz

# Add Eigen3 Library
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})

include_directories(include)

# build nodes
add_executable(autograder_node src/autograder_main.cpp
  src/autograder_node.cpp
)
ament_target_dependencies(autograder_node
  rclcpp
  cg_msgs
)
add_executable(behavior_executive_node src/behavior_executive_main.cpp
  src/behavior_executive_node.cpp
  # State machine
  src/fsm/fsm.cpp
  src/fsm/ready.cpp
  src/fsm/update_map.cpp
  src/fsm/site_work_done.cpp
  src/fsm/map_explored.cpp
  src/fsm/replan_transport.cpp
  src/fsm/plan_transport.cpp
  src/fsm/get_transport_goals.cpp
  src/fsm/plan_exploration.cpp
  src/fsm/get_exploration_goals.cpp
  src/fsm/goals_remaining.cpp
  src/fsm/get_worksystem_trajectory.cpp
  src/fsm/following_trajectory.cpp
  src/fsm/stopped.cpp
  # Planner modules
  src/transport_planner.cpp
  src/exploration_planner.cpp
  src/common.cpp
  src/kinematic_planner.cpp
  # External Linkings
  ../mapping/src/map_util.cpp
)
ament_target_dependencies(behavior_executive_node
  rclcpp
  cg_msgs
  mapping
  nav_msgs # viz
  geometry_msgs # viz
  tf2_ros # viz
)
target_link_libraries(behavior_executive_node ortools::ortools)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(common_test
    test/common_test.cpp
    include/planning/common.hpp
    src/common.cpp
  )
  ament_target_dependencies(common_test
    cg_msgs
  )

  ament_add_gtest(exploration_planner_test
    test/exploration_planner_test.cpp
    ../mapping/src/map_util.cpp
    include/planning/exploration_planner.hpp
    src/exploration_planner.cpp
    src/common.cpp
  )
  ament_target_dependencies(exploration_planner_test
    cg_msgs
    mapping
  )

  ament_add_gtest(transport_planner_test
    test/transport_planner_test.cpp
    include/planning/transport_planner.hpp
    src/transport_planner.cpp
    src/common.cpp
  )
  ament_target_dependencies(transport_planner_test
    cg_msgs
    mapping
  )
  target_link_libraries(transport_planner_test ortools::ortools)

  ament_add_gtest(kinematic_planner_test
    test/kinematic_planner_test.cpp
    src/common.cpp
    src/kinematic_planner.cpp
    ../mapping/src/map_util.cpp
  )
  ament_target_dependencies(kinematic_planner_test
    cg_msgs
    mapping
  )

  ament_add_gtest(velocity_planner_test
    test/velocity_planner_test.cpp
    include/planning/velocity_planner.hpp
    src/velocity_planner.cpp
  )
  ament_target_dependencies(velocity_planner_test
    cg_msgs
    mapping
  )

  ament_add_gtest(fsm_test
    test/fsm_test.cpp
    include/planning/fsm/fsm.hpp
    src/fsm/fsm.cpp
  )
endif()

# Install targets
install(TARGETS autograder_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS behavior_executive_node
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

# Export library files so other packages can link
ament_export_include_directories(include)

ament_package()
