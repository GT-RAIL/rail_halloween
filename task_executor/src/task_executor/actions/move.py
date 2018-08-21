#!/usr/bin/env
# The move action in a task plan

from __future__ import print_function, division

from math import sin, cos

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


class MoveAction(AbstractAction):
    """Move to a location"""

    def __init__(self):
        self._move_base_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self._locations = None

    def init(self, locations, objects):
        self._locations = locations
        rospy.loginfo("Connecting to move_base...")
        self._move_base_client.wait_for_server()
        rospy.loginfo("...move_base connected")

    def run(self, location):
        # Parse out the location. This will error if the location format is
        # incorrect
        location = location.split('.', 1)[1]
        rospy.loginfo("Moving to location: {}".format(location))
        coords = self._locations[location]

        status = GoalStatus.LOST
        for coord in coords:
            # Create and send the goal
            goal = MoveBaseGoal()
            goal.target_pose.pose.position.x = coord['x']
            goal.target_pose.pose.position.y = coord['y']
            goal.target_pose.pose.orientation.z = sin(coord['theta']/2.0)
            goal.target_pose.pose.orientation.w = cos(coord['theta']/2.0)
            goal.target_pose.header.frame_id = coord['frame']
            goal.target_pose.header.stamp = rospy.Time.now()
            self._move_base_client.send_goal(goal)

            # Yield running while the move_client is executing
            while self._move_base_client.get_state() in AbstractAction.RUNNING_GOAL_STATES:
                yield self.running()

            # Check the status and stop executing if we didn't complete our goal
            status = self._move_base_client.get_state()
            if status != GoalStatus.SUCCEEDED:
                break

        # Finally, yield based on the final status
        if status == GoalStatus.SUCCEEDED:
            yield self.succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.preempted()
        else:
            yield self.aborted()

    def stop(self):
        self._move_base_client.cancel_all_goals()
