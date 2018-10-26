# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "candy_manipulation: 14 messages, 0 services")

set(MSG_I_FLAGS "-Icandy_manipulation:/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(candy_manipulation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" "geometry_msgs/PoseStamped:actionlib_msgs/GoalID:std_msgs/Header:geometry_msgs/Quaternion:candy_manipulation/DropResult:geometry_msgs/Point:candy_manipulation/DropActionResult:candy_manipulation/DropActionGoal:candy_manipulation/DropFeedback:actionlib_msgs/GoalStatus:geometry_msgs/Pose:candy_manipulation/DropActionFeedback:candy_manipulation/DropGoal"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" "candy_manipulation/DropFeedback:actionlib_msgs/GoalID:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:candy_manipulation/EmptyGoal"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" "actionlib_msgs/GoalID:std_msgs/Header:candy_manipulation/DropResult:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" "actionlib_msgs/GoalID:std_msgs/Header:geometry_msgs/Quaternion:geometry_msgs/Point:geometry_msgs/PoseStamped:geometry_msgs/Pose:candy_manipulation/DropGoal"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" "actionlib_msgs/GoalID:candy_manipulation/EmptyResult:std_msgs/Header:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" ""
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" "actionlib_msgs/GoalID:actionlib_msgs/GoalStatus:candy_manipulation/EmptyActionResult:candy_manipulation/EmptyResult:candy_manipulation/EmptyActionFeedback:candy_manipulation/EmptyFeedback:candy_manipulation/EmptyGoal:candy_manipulation/EmptyActionGoal:std_msgs/Header"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" ""
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" ""
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" "actionlib_msgs/GoalID:std_msgs/Header:candy_manipulation/EmptyFeedback:actionlib_msgs/GoalStatus"
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" ""
)

get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" NAME_WE)
add_custom_target(_candy_manipulation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "candy_manipulation" "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_cpp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
)

### Generating Services

### Generating Module File
_generate_module_cpp(candy_manipulation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(candy_manipulation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(candy_manipulation_generate_messages candy_manipulation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_cpp _candy_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(candy_manipulation_gencpp)
add_dependencies(candy_manipulation_gencpp candy_manipulation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS candy_manipulation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)
_generate_msg_eus(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
)

### Generating Services

### Generating Module File
_generate_module_eus(candy_manipulation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(candy_manipulation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(candy_manipulation_generate_messages candy_manipulation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_eus _candy_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(candy_manipulation_geneus)
add_dependencies(candy_manipulation_geneus candy_manipulation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS candy_manipulation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)
_generate_msg_lisp(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
)

### Generating Services

### Generating Module File
_generate_module_lisp(candy_manipulation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(candy_manipulation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(candy_manipulation_generate_messages candy_manipulation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_lisp _candy_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(candy_manipulation_genlisp)
add_dependencies(candy_manipulation_genlisp candy_manipulation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS candy_manipulation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)
_generate_msg_nodejs(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(candy_manipulation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(candy_manipulation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(candy_manipulation_generate_messages candy_manipulation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_nodejs _candy_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(candy_manipulation_gennodejs)
add_dependencies(candy_manipulation_gennodejs candy_manipulation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS candy_manipulation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg/GoalStatus.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)
_generate_msg_py(candy_manipulation
  "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
)

### Generating Services

### Generating Module File
_generate_module_py(candy_manipulation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(candy_manipulation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(candy_manipulation_generate_messages candy_manipulation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropActionGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyAction.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropGoal.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/DropFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyActionFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyFeedback.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/dekent/ROS/halloween/src/candy_manipulation/cmake-build-debug/devel/share/candy_manipulation/msg/EmptyResult.msg" NAME_WE)
add_dependencies(candy_manipulation_generate_messages_py _candy_manipulation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(candy_manipulation_genpy)
add_dependencies(candy_manipulation_genpy candy_manipulation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS candy_manipulation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/candy_manipulation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_cpp)
  add_dependencies(candy_manipulation_generate_messages_cpp actionlib_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(candy_manipulation_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/candy_manipulation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_eus)
  add_dependencies(candy_manipulation_generate_messages_eus actionlib_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(candy_manipulation_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/candy_manipulation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_lisp)
  add_dependencies(candy_manipulation_generate_messages_lisp actionlib_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(candy_manipulation_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/candy_manipulation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_nodejs)
  add_dependencies(candy_manipulation_generate_messages_nodejs actionlib_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(candy_manipulation_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/candy_manipulation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET actionlib_msgs_generate_messages_py)
  add_dependencies(candy_manipulation_generate_messages_py actionlib_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(candy_manipulation_generate_messages_py geometry_msgs_generate_messages_py)
endif()
