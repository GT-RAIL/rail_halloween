cmake_minimum_required(VERSION 2.8.3)
project(hotword_detector)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

add_action_files(
  FILES
  DetectHotWord.action
)

add_service_files(
  DIRECTORY srv
  FILES
  DetectHotWord.srv
)

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS nodes/hotword_detector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
