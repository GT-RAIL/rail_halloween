#!/usr/bin/env python
# The pick action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from candy_manipulation.msg import GraspAction, GraspGoal
from task_executor.msg import ArmJointPose
from task_executor.srv import GetArmJointPose


# The class definition

class PickCandyAction(AbstractStep):

    PICK_ACTION_SERVER = "/candy_manipulator/grasp"
    ARM_JOINT_POSES_SERVICE_NAME = "/database/arm_joint_pose"
    PICK_POSE_NAME = "pick"

    def init(self, name):
        self.name = name

        # Initialize the service clients, action clients, etc.
        self._get_arm_joint_pose_srv = rospy.ServiceProxy(
            PickCandyAction.ARM_JOINT_POSES_SERVICE_NAME,
            GetArmJointPose
        )
        self._pick_client = actionlib.SimpleActionClient(
            PickCandyAction.PICK_ACTION_SERVER,
            GraspAction
        )

        rospy.loginfo("Connecting to pick executor...")
        self._pick_client.wait_for_server()
        rospy.loginfo("...pick executor connected")

        rospy.loginfo("Connecting to database services...")
        self._get_arm_joint_pose_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self):
        rospy.loginfo("Action {}: Picking up candy".format(self.name))
        pick_pose = self._get_arm_joint_pose_srv(PickCandyAction.PICK_POSE_NAME).pose
        self.notify_service_called(PickCandyAction.ARM_JOINT_POSES_SERVICE_NAME)

        # Create the goal and send it to the server
        goal = GraspGoal(joint_angles=pick_pose.angles)
        self._pick_client.send_goal(goal)
        self.notify_action_send_goal(PickCandyAction.PICK_ACTION_SERVER, goal)

        # Wait for a response
        while self._pick_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Check the status and return appropriately
        status = self._pick_client.get_state()
        self._pick_client.wait_for_result()
        result = self._pick_client.get_result()
        self.notify_action_recv_result(PickCandyAction.PICK_ACTION_SERVER, status, result)

        # Yield based on how we exited
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                result=result
            )

    def stop(self):
        self._pick_client.cancel_goal()
        self.notify_action_cancel(PickCandyAction.PICK_ACTION_SERVER)
