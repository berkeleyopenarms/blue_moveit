cmake_minimum_required(VERSION 2.8.3)
project(blue_moveit_demos)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
set(GCC_COVERAGE_COMPILE_FLAGS "-fpermissive")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS} ${GAZEBO_CXX_FLAGS}" )

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  gazebo_ros
  joint_state_controller
  joint_state_publisher
  joint_trajectory_controller
  robot_state_publisher
  roscpp
  std_msgs
  tf
  xacro
  moveit_core
  moveit_visual_tools
  moveit_ros_planning
  moveit_ros_planning_interface
  actionlib
  actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(PkgConfig REQUIRED)

pkg_check_modules(JSONCPP jsoncpp)
message(${JSONCPP_LIBRARIES})

catkin_package(
  CATKIN_DEPENDS 
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pick_and_place src/pick_and_place.cpp)
add_library(moveit_arm_utils include/moveit_arm_utils.cpp)
target_link_libraries(pick_and_place moveit_arm_utils ${catkin_LIBRARIES})

add_dependencies(pick_and_place moveit_arm_utils ${blue_moveit_gazebo_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
