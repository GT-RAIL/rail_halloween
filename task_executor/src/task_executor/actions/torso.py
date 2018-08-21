#!/usr/bin/env
# The torso action in a task plan

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus


class TorsoAction(AbstractAction):

    def __init__(self):
        self._torso_client = actionlib.SimpleActionClient(
            "torso_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self._joint_names = ["torso_lift_joint"]
        self._duration = 5.0

    def init(self, locations, objects):
        rospy.loginfo("Connecting to torso_controller...")
        self._torso_client.wait_for_server()
        rospy.loginfo("...torso_controller connected")

    def run(self, height):
        rospy.loginfo("Torso to height: {}".format(height))

        # Create and send the goal height
        trajectory = JointTrajectory()
        trajectory.joint_names = self._joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = [height]
        trajectory.points[0].velocities = [0.0]
        trajectory.points[0].accelerations = [0.0]
        trajectory.points[0].time_from_start = rospy.Duration(self._duration)

        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self._torso_client.send_goal(follow_goal)

        # Yield an empty dict while we're executing
        while self._torso_client.get_state() in AbstractAction.RUNNING_GOAL_STATES:
            yield self.running()

        # Yield a aborted or succeeded based on how we exited
        if self._torso_client.get_state() == GoalStatus.SUCCEEDED:
            yield self.succeeded()
        elif self._torso_client.get_state() == GoalStatus.PREEMPTED:
            yield self.preempted()
        else:
            yield self.aborted()

    def stop(self):
        self._torso_client.cancel_all_goals()
