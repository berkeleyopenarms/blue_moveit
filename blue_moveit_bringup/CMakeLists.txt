cmake_minimum_required(VERSION 2.8.3)
project(blue_moveit_bringup)

find_package(catkin REQUIRED COMPONENTS)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
