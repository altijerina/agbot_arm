cmake_minimum_required(VERSION 3.8)
project(arm_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(agbot_full_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
#find_package(ignition-sdformat9 REQUIRED)

find_package(geometry_msgs)

include_directories(
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

#include_directories(${ignition-sdformat_INCLUDE_DIRS})


add_executable(simple_lifecycle_node src/simple_lifecycle_node.cpp)
ament_target_dependencies(simple_lifecycle_node rclcpp rclcpp_lifecycle std_msgs)

add_executable(move_group_to_cartesian src/move_group_to_cartesian.cpp)
ament_target_dependencies(move_group_to_cartesian rclcpp moveit_ros_planning_interface moveit_ros_move_group)

add_executable(arm_moveit_interface src/arm_moveit_interface.cpp)
ament_target_dependencies(arm_moveit_interface geometry_msgs rclcpp moveit_ros_planning_interface moveit_ros_move_group)

target_link_libraries(arm_moveit_interface
  ${moveit_ros_planning_interface_LIBRARIES}
)

#target_link_libraries(arm_moveit_interface
#  ${ignition-sdformat_LIBRARIES}
#)

install(TARGETS
  arm_moveit_interface
  simple_lifecycle_node
  move_group_to_cartesian
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()