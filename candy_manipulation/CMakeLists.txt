cmake_minimum_required(VERSION 2.8.3)
project(candy_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  control_msgs
  geometry_msgs
  message_generation
  moveit_msgs
  moveit_ros_planning_interface
  roscpp
  task_executor
  tf
  tf_conversions
  tf2_geometry_msgs
  tf2_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_action_files(FILES
        Drop.action
        Grasp.action
        PresetJointsMove.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        actionlib_msgs
        task_executor
        moveit_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(candy_manipulator src/candy_manipulator.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(candy_manipulator ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
add_dependencies(candy_manipulator ${PROJECT_NAME}_generate_messages_cpp task_executor_generate_messages_cpp)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS candy_manipulator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Copy header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)
