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

### TODO: had to include this to get mapping package to work?
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
####

include_directories(include)

# build nodes
add_executable(autograder_node src/autograder_main.cpp
  src/autograder_node.cpp
)
ament_target_dependencies(autograder_node
  rclcpp
  cg_msgs
)

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
    include/planning/exploration_planner.hpp
    src/exploration_planner.cpp
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
    include/planning/kinematic_planner.hpp
    src/kinematic_planner.cpp
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
endif()

# Install targets
install(TARGETS autograder_node
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
