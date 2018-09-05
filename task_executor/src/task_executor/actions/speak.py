#!/usr/bin/env
# The speech action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep
from sound_interface import SoundClient

from actionlib_msgs.msg import GoalStatus


class SpeakAction(AbstractStep):

    def init(self, **kwargs):
        self._speak_client = SoundClient()
        self._stopped = False

    def run(self, text, affect=""):
        # Check to see if we know about this beep type
        if type(text) != str \
                or (affect and affect.upper() not in self._speak_client.get_affect_names()):
            error_msg = "Invalid speech args: {} (affect: {}).".format(text, affect)
            rospy.logerr(error_msg)
            self.set_aborted(exception=Exception(error_msg))
            raise StopIteration()

        self._stopped = False
        affect = affect.upper()
        rospy.loginfo("Saying: {} (affect: {})".format(text, affect))

        # Send the command to play the beep and wait
        self._speak_client.say(text, affect, blocking=False)
        while self._speak_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            if self._stopped:
                break

            yield self.set_running()

        # Yield based on how we exited
        if self._speak_client.get_state() == GoalStatus.PREEMPTED or self._stopped:
            yield self.set_preempted()
        elif self._speak_client.get_state() == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        else:
            yield self.set_aborted()

    def stop(self):
        self._stopped = True
        self._speak_client.stop()
