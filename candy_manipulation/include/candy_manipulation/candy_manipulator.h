#ifndef CANDY_MANIPULATION_CANDY_MANIPULATOR_H
#define CANDY_MANIPULATION_CANDY_MANIPULATOR_H

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/GripperCommandAction.h>
#include <candy_manipulation/DropAction.h>
#include <candy_manipulation/GraspAction.h>
#include <candy_manipulation/StirAction.h>
#include <moveit_msgs/GetCartesianPath.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/conversions.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class CandyManipulator
{

public:
    CandyManipulator();

private:

  void executeGrasp(const candy_manipulation::GraspGoalConstPtr &goal);

  void executeStir(const candy_manipulation::StirGoalConstPtr &goal);

  void executeDrop(const candy_manipulation::DropGoalConstPtr &goal);

  bool enableCollision();

  ros::NodeHandle n, pn;

  ros::ServiceClient compute_cartesian_path_client;
  ros::ServiceClient planning_scene_client;

  // publishers
  ros::Publisher planning_scene_publisher;

  // actionlib
  actionlib::SimpleActionServer<candy_manipulation::GraspAction> grasp_server;
  actionlib::SimpleActionServer<candy_manipulation::StirAction> stir_server;
  actionlib::SimpleActionServer<candy_manipulation::DropAction> drop_server;
  actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_client;

  // MoveIt interfaces
  move_group_interface::MoveGroup *arm_group;
  move_group_interface::PlanningSceneInterface *planning_scene_interface;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;

  tf::TransformListener tf1_listener;

  trajectory_msgs::JointTrajectory stir_trajectory;

  std::vector<std::string> gripper_names;
  std::vector<std::string> collision_objects;

  double drop_wait_time;
};

#endif  // CANDY_MANIPULATION_CANDY_MANIPULATOR_H
