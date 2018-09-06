#!/usr/bin/env
# The torso action in a task plan

import numpy as np

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from actionlib_msgs.msg import GoalStatus


class TorsoAction(AbstractStep):

    def init(self, name):
        self.name = name
        self._torso_client = actionlib.SimpleActionClient(
            "torso_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self._joint_names = ["torso_lift_joint"]

        self._duration = 5.0
        self._tolerance = 3e-2  # A 3 cm tolerance in torso joint is OK

        rospy.loginfo("Connecting to torso_controller...")
        self._torso_client.wait_for_server()
        rospy.loginfo("...torso_controller connected")

        # Create a subscriber to check if we even have to move to the desired
        # torso height
        self._current_torso_height = 0.0
        self._joints_sub = rospy.Subscriber("/joint_states", JointState, self._on_joints)

    def run(self, height):
        rospy.loginfo("Action {}: Torso to height {}".format(self.name, height))

        # Check to see if we are close enough to the desired torso height
        if np.isclose(self._current_torso_height, height, atol=self._tolerance):
            yield self.set_succeeded()
            raise StopIteration()

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
        while self._torso_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._torso_client.get_state()
        self._torso_client.wait_for_result()
        result = self._torso_client.get_result()

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=height,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=height,
                result=result
            )

    def stop(self):
        self._torso_client.cancel_goal()

    def _on_joints(self, msg):
        try:
            idx = msg.name.index("torso_lift_joint")
            self._current_torso_height = msg.position[idx]
        except ValueError as e:
            pass
