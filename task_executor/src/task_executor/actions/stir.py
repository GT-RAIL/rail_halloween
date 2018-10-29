#!/usr/bin/env python
# The stir action in the task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from data_recorder.msg import PlaybackAction, PlaybackGoal


# The class definition

class StirAction(AbstractStep):

    PLAYBACK_ACTION_SERVER = "/playback_primitive"
    PLAYBACK_PRIMITIVE_NAME = "stir"

    def init(self, name):
        self.name = name
        self._stir_client = actionlib.SimpleActionClient(
            StirAction.PLAYBACK_ACTION_SERVER,
            PlaybackAction
        )

        rospy.loginfo("Connecting to stir executor...")
        self._stir_client.wait_for_server()
        rospy.loginfo("...stir executor connected")

    def run(self):
        rospy.loginfo("Action {}: Stirring the pot".format(self.name))

        # Create the goal, send it to the server and wait
        goal = PlaybackGoal(primitive_name=StirAction.PLAYBACK_PRIMITIVE_NAME)
        self._stir_client.send_goal(goal)
        self.notify_action_send_goal(StirAction.PLAYBACK_ACTION_SERVER, goal)
        while self._stir_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Check the status and return appropriately
        status = self._stir_client.get_state()
        self._stir_client.wait_for_result()
        result = self._stir_client.get_result()
        self.notify_action_recv_result(StirAction.PLAYBACK_ACTION_SERVER, status, result)

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
        self._stir_client.cancel_goal()
        self.notify_action_cancel(StirAction.PLAYBACK_ACTION_SERVER)
