#!/usr/bin/env python
# The main action server that provides local recovery behaviour

from __future__ import print_function, division

import pickle

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from assistance_msgs.msg import RequestAssistanceAction, RequestAssistanceResult

from task_executor.actions import get_default_actions
from sound_interface import SoundClient

# from .dialogue import DialogueManager


# The server performs local behaviours to resume execution after contact with
# local humans

class LocalRecoveryServer(object):
    """
    Given a request for assistance, this class interfaces with the robot's
    look, speech, and point modules to request assistance
    """

    JOY_TOPIC = "/joy"
    DISABLE_ARM_BREAKER_BUTTON = 7
    ENABLE_ARM_BREAKER_BUTTON = 5
    BUTTON_PRESS_TIMEOUT = rospy.Duration(0.5)

    def __init__(self):
        # Instantiate the action server to perform the recovery
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

        # The actions that we are interested in using
        self.actions = get_default_actions()

        # Also subscribe to the joystick topic in order to enable and disable
        # the breakers
        self._assisting = False                  # Flag for we-are-assisting
        self._last_pressed = None                # Which button was pressed?
        self._last_pressed_time = rospy.Time(0)  # Don't spam the breaker srv
        self._joy_sub = rospy.Subscriber(LocalRecoveryServer.JOY_TOPIC, Joy, self._on_joy)

    def start(self):
        # Initialize the actions
        self.actions.init()

        # Finally, start our action server to indicate that we're ready
        self._server.start()
        rospy.loginfo("Local strategy node ready...")

    def execute(self, goal):
        """Execute the request for assistance"""
        result = self._server.get_default_result()
        result.stats.request_received = rospy.Time.now()

        rospy.loginfo("Serving Assistance Request for: {} (status - {})"
                      .format(goal.component, goal.component_status))
        if goal.context != '':
            goal.context = pickle.loads(goal.context)

        # We ack the request now
        result.stats.request_acked = rospy.Time.now()
        self._assisting = True

        # The actual error recovery mechanism
        # Unsure beep first
        self.actions.beep(beep=SoundClient.BEEP_UNSURE, async=True)

        # Then wait for the joystick trigger
        for variables in self.actions.joystick_trigger.run(beep=True):
            if self._server.is_preempt_requested() or not self._server.is_active():
                self.actions.joystick_trigger.stop()

        # Determine exit status if we are preempted
        if self.actions.joystick_trigger.is_preempted():
            result.context = pickle.dumps(variables)
            self._server.set_preempted(result)
            return

        # Then determine the eventual response based on the choice
        choice = variables['choice']
        if choice:
            result.resume_hint = RequestAssistanceResult.RESUME_CONTINUE
        else:
            result.resume_hint = RequestAssistanceResult.RESUME_NONE

        # Finally, send the response back
        self._assisting = False
        result.stats.request_complete = rospy.Time.now()
        self._server.set_succeeded(result)

    def stop(self):
        pass

    def _on_joy(self, joy_msg):
        # Check if we need to debounce the button press
        if self._last_pressed_time + LocalRecoveryServer.BUTTON_PRESS_TIMEOUT >= rospy.Time.now():
            # If the button is still pressed, update the time of press
            if joy_msg.buttons[self._last_pressed] > 0:
                self._last_pressed_time = rospy.Time.now()
            return
        elif self._last_pressed_time > rospy.Time(0) and \
                self._last_pressed_time + LocalRecoveryServer.BUTTON_PRESS_TIMEOUT < rospy.Time.now():
            # Reset the debounce
            self._last_pressed_time = rospy.Time(0)
            self._last_pressed = None
            return

        # Don't continue if we're not assisting
        if not self._assisting:
            return

        # Finally, update the breakers accordingly
        if joy_msg.buttons[LocalRecoveryServer.ENABLE_ARM_BREAKER_BUTTON] > 0:
            self._last_pressed = LocalRecoveryServer.ENABLE_ARM_BREAKER_BUTTON
            self._last_pressed_time = rospy.Time.now()
            self.actions.toggle_breakers(enable_arm=True)
        elif joy_msg.buttons[LocalRecoveryServer.DISABLE_ARM_BREAKER_BUTTON] > 0:
            self._last_pressed = LocalRecoveryServer.DISABLE_ARM_BREAKER_BUTTON
            self._last_pressed_time = rospy.Time.now()
            self.actions.toggle_breakers(enable_arm=False)
