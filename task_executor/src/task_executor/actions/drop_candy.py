#!/usr/bin/env python
# The pick action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from candy_manipulation.msg import DropAction, DropGoal


# The class definition

class DropCandyAction(AbstractStep):

    DROP_ACTION_SERVER = "/candy_manipulator/drop"

    def init(self, name):
        self.name = name
        self._drop_client = actionlib.SimpleActionClient(
            DropCandyAction.DROP_ACTION_SERVER,
            DropAction
        )

        rospy.loginfo("Connecting to drop executor...")
        self._drop_client.wait_for_server()
        rospy.loginfo("...drop executor connected")

    def run(self):
        rospy.loginfo("Action {}: Dropping the candy")

        # Create the goal
        goal = DropGoal()
        self._drop_client.send_goal(goal)
        self.notify_action_send_goal(DropCandyAction.DROP_ACTION_SERVER, goal)
        while self._drop_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Check the status
        status = self._drop_client.get_state()
        self._drop_client.wait_for_result()
        result = self._drop_client.get_result()
        self.notify_action_recv_result(DropCandyAction.DROP_ACTION_SERVER, status, result)

        # Exit accordingly
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
        self._drop_client.cancel_goal()
        self.notify_action_cancel(DropCandyAction.DROP_ACTION_SERVER)
