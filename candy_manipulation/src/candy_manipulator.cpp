#include <candy_manipulation/candy_manipulator.h>

using std::string;
using std::stringstream;
using std::vector;

CandyManipulator::CandyManipulator() :
    pn("~"),
    tf_listener(tf_buffer),
    gripper_client("gripper_controller/gripper_action"),
    grasp_server(pn, "grasp", boost::bind(&CandyManipulator::executeGrasp, this, _1), false),
    stir_server(pn, "stir", boost::bind(&CandyManipulator::executeStir, this, _1), false),
    drop_server(pn, "drop", boost::bind(&CandyManipulator::executeDrop, this, _1), false)
{
  pn.param("drop_wait_time", drop_wait_time, 1.0);

  gripper_names.push_back("gripper_link");
  gripper_names.push_back("l_gripper_finger_link");
  gripper_names.push_back("r_gripper_finger_link");

//  grasp_pose.name.push_back("shoulder_pan_joint");
//  grasp_pose.name.push_back("shoulder_lift_joint");
//  grasp_pose.name.push_back("upperarm_roll_joint");
//  grasp_pose.name.push_back("elbow_flex_joint");
//  grasp_pose.name.push_back("forearm_roll_joint");
//  grasp_pose.name.push_back("wrist_flex_joint");
//  grasp_pose.name.push_back("wrist_roll_joint");
//  grasp_pose.position.push_back(1.4264682869091796);
//  grasp_pose.position.push_back(0.7546394518989991);
//  grasp_pose.position.push_back(1.8010552049179078);
//  grasp_pose.position.push_back(-1.4541920443832397);
//  grasp_pose.position.push_back(2.400358734161377);
//  grasp_pose.position.push_back(-1.2402720307693482);
//  grasp_pose.position.push_back(0.25749375096061705);

  // TODO: read stir trajectory from file, store in stir_trajectory

  arm_group = new move_group_interface::MoveGroup("arm");
  arm_group->startStateMonitor();

  planning_scene_interface = new move_group_interface::PlanningSceneInterface();

  planning_scene_publisher = n.advertise<moveit_msgs::PlanningScene>("/planning_scene", 1);

  compute_cartesian_path_client = n.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path");
  planning_scene_client = n.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  grasp_server.start();
  stir_server.start();
  drop_server.start();
}

void CandyManipulator::executeGrasp(const candy_manipulation::GraspGoalConstPtr &goal)
{
  candy_manipulation::GraspResult result;
  collision_objects.clear();

  // disable gripper-environment collision
  if (grasp_server.isPreemptRequested())
  {
    grasp_server.setPreempted(result, "Preempted at disabling environment collision.");
  }
  moveit_msgs::GetPlanningScene planning_scene_srv;
  planning_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

  if (!planning_scene_client.call(planning_scene_srv))
  {
    ROS_INFO("Could not get the current planning scene!");
    grasp_server.setAborted(result, "Could not get the current planning scene!");
    return;
  }
  else
  {
    collision_detection::AllowedCollisionMatrix acm(planning_scene_srv.response.scene.allowed_collision_matrix);

    // disable collisions between gripper links and octomap
    acm.setEntry("<octomap>", gripper_names, true);
    moveit_msgs::PlanningScene planning_scene_update;
    acm.getMessage(planning_scene_update.allowed_collision_matrix);
    planning_scene_update.is_diff = true;
    planning_scene_publisher.publish(planning_scene_update);

    ros::Duration(0.5).sleep(); //delay for publish to go through
  }

  // move to grasp pose
  if (grasp_server.isPreemptRequested())
  {
    enableCollision();
    grasp_server.setPreempted(result, "Preempted at move to grasp pose.");
    return;
  }

  sensor_msgs::JointState grasp_pose;
  grasp_pose.name.push_back("shoulder_pan_joint");
  grasp_pose.name.push_back("shoulder_lift_joint");
  grasp_pose.name.push_back("upperarm_roll_joint");
  grasp_pose.name.push_back("elbow_flex_joint");
  grasp_pose.name.push_back("forearm_roll_joint");
  grasp_pose.name.push_back("wrist_flex_joint");
  grasp_pose.name.push_back("wrist_roll_joint");
  grasp_pose.position.clear();
  for (unsigned int i = 0; i < goal->pick_pose.size(); i ++)
  {
    grasp_pose.position.push_back(goal->pick_pose[i]);
  }

  arm_group->setStartStateToCurrentState();
  arm_group->setJointValueTarget(grasp_pose);

  // TODO (stretch): break this into plan and move so we can do safety checks
  int error_code = arm_group->move();

  // assume success, because this is a very short move into a potentially dense bowl of candy

  // close gripper
  if (grasp_server.isPreemptRequested())
  {
    enableCollision();
    grasp_server.setPreempted(result, "Preempted at gripper close.");
    return;
  }

  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0;
  gripper_goal.command.max_effort = 50;  // TODO: set effort for candy
  gripper_client.sendGoal(gripper_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  // move to post grasp pose
  if (grasp_server.isPreemptRequested())
  {
    enableCollision();
    grasp_server.setPreempted(result, "Preempted at move to post grasp pose.");
    return;
  }

  sensor_msgs::JointState post_grasp_pose;
  post_grasp_pose.name.push_back("shoulder_pan_joint");
  post_grasp_pose.name.push_back("shoulder_lift_joint");
  post_grasp_pose.name.push_back("upperarm_roll_joint");
  post_grasp_pose.name.push_back("elbow_flex_joint");
  post_grasp_pose.name.push_back("forearm_roll_joint");
  post_grasp_pose.name.push_back("wrist_flex_joint");
  post_grasp_pose.name.push_back("wrist_roll_joint");
  post_grasp_pose.position.clear();
  for (unsigned int i = 0; i < goal->post_pick_pose.size(); i ++)
  {
    post_grasp_pose.position.push_back(goal->post_pick_pose[i]);
  }

  arm_group->setStartStateToCurrentState();
  arm_group->setJointValueTarget(post_grasp_pose);

  // TODO (stretch): break this into plan and move so we can do safety checks
  error_code = arm_group->move();

  // assume success, because this is a very short move out of the candy

  // add collision sphere only after we're clear of the bowl
  // create virtual object at gripper
  if (grasp_server.isPreemptRequested())
  {
    enableCollision();
    grasp_server.setPreempted(result, "Preempted at virtual collision object creation.");
    return;
  }
  vector<moveit_msgs::CollisionObject> virtual_objects;
  virtual_objects.resize(1);
  string group_reference_frame = arm_group->getPoseReferenceFrame();
  group_reference_frame = group_reference_frame.substr(1);
  virtual_objects[0].header.frame_id = group_reference_frame;
  virtual_objects[0].id = "virtual_object";

  // set object shape
  shape_msgs::SolidPrimitive bounding_volume;
  bounding_volume.type = shape_msgs::SolidPrimitive::SPHERE;
  bounding_volume.dimensions.resize(1);
  bounding_volume.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.1;
  virtual_objects[0].primitives.push_back(bounding_volume);

  // set object pose
  geometry_msgs::Pose virtual_object_pose;
  geometry_msgs::TransformStamped eef_transform =
      tf_buffer.lookupTransform(group_reference_frame, "gripper_link", ros::Time(0), ros::Duration(3.0));
  virtual_object_pose.position.x = eef_transform.transform.translation.x;
  virtual_object_pose.position.y = eef_transform.transform.translation.y;
  virtual_object_pose.position.z = eef_transform.transform.translation.z;
  virtual_object_pose.orientation = eef_transform.transform.rotation;
  virtual_objects[0].primitive_poses.push_back(virtual_object_pose);

  virtual_objects[0].operation = moveit_msgs::CollisionObject::ADD;
  planning_scene_interface->addCollisionObjects(virtual_objects);
  collision_objects.push_back("virtual_object");
  ros::Duration(0.5).sleep();  // wait for scene to be updated...

  // attach virtual object to gripper
  if (grasp_server.isPreemptRequested())
  {
    enableCollision();
    planning_scene_interface->removeCollisionObjects(collision_objects);
    collision_objects.clear();
    grasp_server.setPreempted(result, "Preempted at virtual object attachment.");
    return;
  }
  arm_group->attachObject("virtual_object", arm_group->getEndEffectorLink(), gripper_names);
  ros::Duration(0.2).sleep();  // wait for change to go through (seems to have a race condition otherwise)

  // enable gripper collision with octomap
  if (!enableCollision())
  {
    grasp_server.setAborted(result, "Could not get the current planning scene!");
    return;
  }

  grasp_server.setSucceeded(result);
}

void CandyManipulator::executeStir(const candy_manipulation::StirGoalConstPtr &goal)
{
  // use MoveIt to plan a trajectory?

  //TODO: execute stir trajectory
}

void CandyManipulator::executeDrop(const candy_manipulation::DropGoalConstPtr &goal)
{
  candy_manipulation::DropResult result;

  // detach collision sphere
  arm_group->detachObject("virtual_object");
  planning_scene_interface->removeCollisionObjects(collision_objects);
  collision_objects.clear();

  ros::Duration(drop_wait_time).sleep();

  if (drop_server.isPreemptRequested())
  {
    drop_server.setPreempted(result, "Preempted at open gripper.");
    return;
  }

  // open gripper
  control_msgs::GripperCommandGoal gripper_goal;
  gripper_goal.command.position = 0.1;
  gripper_goal.command.max_effort = 100;
  gripper_client.sendGoal(gripper_goal);
  gripper_client.waitForResult(ros::Duration(5.0));

  drop_server.setSucceeded(result);
}

bool CandyManipulator::enableCollision()
{
  moveit_msgs::GetPlanningScene planning_scene_srv;
  planning_scene_srv.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  if (!planning_scene_client.call(planning_scene_srv))
  {
    ROS_INFO("Could not get the current planning scene!");
    return false;
  }
  else
  {
    collision_detection::AllowedCollisionMatrix acm(planning_scene_srv.response.scene.allowed_collision_matrix);
    acm.setEntry("<octomap>", gripper_names, false);

    moveit_msgs::PlanningScene planning_scene_update;
    acm.getMessage(planning_scene_update.allowed_collision_matrix);
    planning_scene_update.is_diff = true;
    planning_scene_publisher.publish(planning_scene_update);
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "candy_manipulator");

  CandyManipulator cm;

  ros::spin();

  return EXIT_SUCCESS;
}
