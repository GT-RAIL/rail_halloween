#!/usr/bin/env python
# The main action server that provides local recovery behaviour

from __future__ import print_function, division

import pickle

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import RequestAssistanceAction, RequestAssistanceResult
from power_msgs.srv import BreakerCommand

from task_executor.actions import default_actions
from sound_interface import SoundClient


# The server performs local behaviours to resume execution after contact with
# local humans

class LocalRecoveryServer(object):
    """
    Given a request for assistance, this class interfaces with the robot's
    look, speech, and point modules to request assistance
    """

    def __init__(self):
        # Instantiate the action server to perform the recovery
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

        # Service to communicate with the breakers
        self._arm_breaker_srv = rospy.ServiceProxy("/arm_breaker", BreakerCommand)
        self._base_breaker_srv = rospy.ServiceProxy("/base_breaker", BreakerCommand)
        self._gripper_breaker_srv = rospy.ServiceProxy("/gripper_breaker", BreakerCommand)

        # The actions that we are interested in using
        self.actions = default_actions

    def start(self):
        # Initialize the actions
        self.actions.init()

        # Initialize our connections to the robot driver
        rospy.loginfo("Connecting to robot driver...")
        self._arm_breaker_srv.wait_for_service()
        self._base_breaker_srv.wait_for_service()
        self._gripper_breaker_srv.wait_for_service()
        rospy.loginfo("...robot driver connected")

        # Finally, start our action server to indicate that we're ready
        self._server.start()
        rospy.loginfo("Local strategy node ready...")

    def execute(self, goal):
        """Execute the request for assistance"""
        result = self._server.get_default_result()
        result.stats.request_received = rospy.Time.now()
        rospy.loginfo("Serving Assistance Request for: {} ({})"
                      .format(goal.component, goal.component_status))

        # The actual error recovery mechanism
        # Sad beep first
        self.actions.beep(beep=SoundClient.BEEP_SAD, async=True)

        # First we look for a person.
        for variables in self.actions.find_closest_person.run(max_duration=0.0):
            if self._server.is_preempt_requested() or not self._server.is_active():
                self.actions.find_closest_person.stop()

        # If we exited without success, report the failure. Otherwise, save the
        # person that we found
        if self.actions.find_closest_person.is_preempted():
            result.context = pickle.dumps(variables)
            self._server.set_preempted(result)
            return

        if self.actions.find_closest_person.is_aborted():
            result.context = pickle.dumps(variables)
            self._server.set_aborted(result)
            return

        person = variables['person']

        # Show exceitement and solicit help from them
        self.actions.beep(beep=SoundClient.BEEP_EXCITED)
        result.stats.request_acked = rospy.Time.now()
        self._enter_compliant_mode()

        # Return when the request for help is completed
        self._exit_compliant_mode()
        result.resume_hint = RequestAssistanceResult.RESUME_CONTINUE
        result.stats.request_complete = rospy.Time.now()
        self._server.set_succeeded(result)

    def stop(self):
        pass

    def _enter_compliant_mode(self):
        # Send a disable message to the base and gripper
        self._base_breaker_srv(enable=False)
        self._gripper_breaker_srv(enable=False)

        # Disable and reenable the arm so that we have gravity comp
        self._arm_breaker_srv(enable=False)
        self._arm_breaker_srv(enable=True)

        rospy.loginfo("Robot is now in compliant mode")

    def _exit_compliant_mode(self):
        # Reenable the base and the gripper
        self._base_breaker_srv(enable=True)
        self._gripper_breaker_srv(enable=True)

        rospy.loginfo("Robot has exited compliant mode")

    def _get_speech_tokens_from_goal(self, goal):
        # Given a goal, generate the tokens relevant to asking for help.
        # The target speech is: "<component> at step <step_num> failed."
        # "the most likely cause is <cause>"
        pass
