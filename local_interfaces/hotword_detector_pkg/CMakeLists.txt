cmake_minimum_required(VERSION 2.8.3)
project(hotword_detector_pkg)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  roslaunch
)

catkin_python_setup()

#add_message_files(
#    DIRECTORY msg
#    FILES
#)

add_service_files(
  DIRECTORY srv
  FILES
  DetectHotWord.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS bin/hotword_detector_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
