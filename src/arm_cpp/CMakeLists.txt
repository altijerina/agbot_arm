cmake_minimum_required(VERSION 3.8)
project(arm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sdformat REQUIRED)

include_directories(include)

add_executable(move_group_to_cartesian src/move_group_to_cartesian.cpp)
ament_target_dependencies(move_group_to_cartesian rclcpp moveit_ros_planning_interface sdformat)

install(
  TARGETS move_group_to_cartesian 
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
