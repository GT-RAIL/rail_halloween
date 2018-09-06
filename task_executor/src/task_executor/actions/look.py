#!/usr/bin/env
# The look action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from control_msgs.msg import PointHeadAction, PointHeadGoal
from actionlib_msgs.msg import GoalStatus


class LookAction(AbstractStep):

    def init(self, name):
        self.name = name
        self._look_client = actionlib.SimpleActionClient(
            "head_controller/point_head",
            PointHeadAction
        )
        self._duration = 0.5

        rospy.loginfo("Connecting to head_controller...")
        self._look_client.wait_for_server()
        rospy.loginfo("...head_controller connected")

    def run(self, pose):
        rospy.loginfo("Action {}: Looking at point: {}".format(self.name, pose))

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
        while self._look_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._look_client.get_state()
        self._look_client.wait_for_result()
        result = self._look_client.get_result()

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=pose,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=pose,
                result=result
            )

    def stop(self):
        self._look_client.cancel_goal()
