#!/usr/bin/env python
# The tuck action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import PresetMoveAction, PresetMoveGoal


class DropPoseAction(AbstractAction):

    def init(self, locations, objects):
        self._drop_pose_client = actionlib.SimpleActionClient(
            "/grasp_executor/drop_position",
            PresetMoveAction
        )

        rospy.loginfo("Connecting to drop_pose_executor...")
        self._drop_pose_client.wait_for_server()
        rospy.loginfo("...drop_pose_executor connected")

    def run(self):
        rospy.loginfo("Readying arm")

        # Create and send the goal
        goal = PresetMoveGoal()
        self._drop_pose_client.send_goal(goal)

        # Yield running while the client is executing
        while self._drop_pose_client.get_state() in AbstractAction.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Yield based on the server's status
        status = self._drop_pose_client.get_state()
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted()
        else:
            yield self.set_aborted()

    def stop(self):
        self._drop_pose_client.cancel_goal()
