#!/usr/bin/env
# The gripper action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib_msgs.msg import GoalStatus


class GripperAction(AbstractStep):

    def init(self, **kwargs):
        self._gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action",
            GripperCommandAction
        )

        rospy.loginfo("Connecting to gripper_controller...")
        self._gripper_client.wait_for_server()
        rospy.loginfo("...gripper_controller connected")

    def run(self, command):
        if command not in ['close', 'open']:
            error_msg = "Unknown gripper command: {}".format(command)
            rospy.logerr(error_msg)
            self.set_aborted(exception=Exception(error_msg))
            raise StopIteration()

        rospy.loginfo("Gripper command: {}".format(command))

        # Create and send the goal pose
        goal = GripperCommandGoal()
        if command == 'close':
            goal.command.position = 0.0
            goal.command.max_effort = 200
        elif command == 'open':
            goal.command.position = 0.15

        self._gripper_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._gripper_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Yield based on how we exited
        if self._gripper_client.get_state() == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif self._gripper_client.get_state() == GoalStatus.PREEMPTED:
            yield self.set_preempted()
        else:
            yield self.set_aborted()

    def stop(self):
        self._gripper_client.cancel_goal()
