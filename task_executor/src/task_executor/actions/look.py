#!/usr/bin/env
# The torso action in a task plan

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib_msgs.msg import GoalStatus


class LookAction(AbstractAction):

    def __init__(self):
        self._look_client = actionlib.SimpleActionClient(
            "head_controller/point_head",
            PointHeadAction
        )
        self._duration = 1.0

    def init(self, locations, objects):
        rospy.loginfo("Connecting to head_controller...")
        self._look_client.wait_for_server()
        rospy.loginfo("...head_controller connected")

    def run(self, pose):
        rospy.loginfo("Looking at point: {}".format(pose))

        # Create and send the goal pose
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = pose['frame']
        goal.target.point.x = pose['x']
        goal.target.point.y = pose['y']
        goal.target.point.z = pose['z']
        goal.min_duration = rospy.Duration(self._duration)
        self._look_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._look_client.get_state() in AbstractAction.RUNNING_GOAL_STATES:
            yield self.running()

        # Yield based on how we exited
        if self._look_client.get_state() == GoalStatus.SUCCEEDED:
            yield self.succeeded()
        elif self._look_client.get_state() == GoalStatus.PREEMPTED:
            yield self.preempted()
        else:
            yield self.aborted()

    def stop(self):
        self._look_client.cancel_all_goals()
