#!/usr/bin/env
# The beep action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep
from sound_interface import SoundClient

from actionlib_msgs.msg import GoalStatus


class BeepAction(AbstractStep):

    def init(self, **kwargs):
        self._beep_client = SoundClient()
        self._stopped = False

    def run(self, beep, async=False):
        # Check to see if we know about this beep type
        if type(beep) != str \
                or beep.upper() not in self._beep_client.get_beep_names():
            error_msg = "Unrecognized beep {}.".format(beep)
            rospy.logerr(error_msg)
            self.set_aborted(exception=Exception(error_msg))
            raise StopIteration()

        self._stopped = False
        beep = beep.upper()
        rospy.loginfo("Playing beep: {}".format(beep))

        # Send the command to play the beep and wait if not async. If async,
        # set as succeeded and exit
        self._beep_client.beep(beep, blocking=False)
        if async:
            yield self.set_succeeded()
            raise StopIteration()

        # If we are to block until the client is done running...
        while self._beep_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            if self._stopped:
                break

            yield self.set_running()

        # Yield based on how we exited
        if self._beep_client.get_state() == GoalStatus.PREEMPTED or self._stopped:
            yield self.set_preempted()
        elif self._beep_client.get_state() == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        else:
            yield self.set_aborted()

    def stop(self):
        self._stopped = True
        self._beep_client.stop()
