#!/usr/bin/env python
# The speech action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep
from sound_interface import SoundClient

from actionlib_msgs.msg import GoalStatus


class SpeakAction(AbstractStep):

    def init(self, name):
        self.name = name
        self._speak_client = SoundClient()
        self._stopped = False

    def run(self, text, affect=""):
        # Check to see if we know about this affect type
        if type(text) != str \
                or (affect and affect.upper() not in self._speak_client.get_affect_names()):
            rospy.logerr("Action {}: FAIL. Invalid Args: {} (affect: {})."
                         .format(self.name, text, affect))
            self.set_aborted(
                action=self.name,
                cause="Invalid Args",
                context=(text, affect,)
            )
            raise StopIteration()

        self._stopped = False
        affect = affect.upper()
        rospy.loginfo("Action {}: {} (affect: {})".format(self.name, text, affect))

        # Send the command to play the speech and wait
        self._speak_client.say(text, affect, blocking=False)
        while self._speak_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            if self._stopped:
                break

            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._speak_client.get_state()
        result = self._speak_client.get_result(blocking=True)
        if status == GoalStatus.PREEMPTED or self._stopped:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=text,
                affect=affect,
                result=result
            )
        elif status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=text,
                affect=affect,
                result=result
            )

    def stop(self):
        self._stopped = True
        self._speak_client.stop()
