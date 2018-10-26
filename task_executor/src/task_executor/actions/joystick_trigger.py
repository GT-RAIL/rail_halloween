#!/usr/bin/env python
# The trigger based on the joystick buttons

import rospy

from task_executor.abstract_step import AbstractStep
from sound_interface import SoundClient

from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy

from .beep import BeepAction


class JoystickTriggerAction(AbstractStep):
    """
    A trigger returns a true or false depending on the logic in the trigger
    """

    JOY_TOPIC = "/joy"
    ACCEPT_BUTTON_IDX = 13
    REJECT_BUTTON_IDX = 15

    def init(self, name):
        self.name = name

        # Internal variables to keep track of trigger state
        self._choice = None          # The choice made by the person
        self._make_a_choice = False  # Indicates if we need to ask the human
        self._joy_sub = rospy.Subscriber(
            JoystickTriggerAction.JOY_TOPIC,
            Joy,
            self._on_joy
        )

        # Child actions
        self._beep_action = BeepAction()

        # Set a stop flag
        self._stopped = False

        # Initialize the actions
        self._beep_action.init('beep_joystick_trigger')

    def run(self, beep=True):
        rospy.loginfo("Action {}: Waiting for a trigger response on Joystick".format(self.name))

        # Set the flags for the wait
        self._stopped = False
        self._choice = None
        self._make_a_choice = True

        # Then wait for the trigger
        while self._make_a_choice:
            if self._stopped:
                yield self.set_preempted(action=self.name)
                raise StopIteration()

            yield self.set_running()

        # Make a sound based on the response
        if beep:
            self._beep_action(beep=(SoundClient.BEEP_CHEERFUL if self._choice else SoundClient.BEEP_SAD))

        # Yield a success
        yield self.set_succeeded(choice=self._choice)

    def stop(self):
        self._beep_action.stop()
        self._stopped = True

    def _on_joy(self, joy_msg):
        # If there is no choice to be made, then don't make a choice
        if not self._make_a_choice:
            return

        # We need to make a choice
        if joy_msg.buttons[JoystickTriggerAction.ACCEPT_BUTTON_IDX] > 0:
            self._choice = True
        elif joy_msg.buttons[JoystickTriggerAction.REJECT_BUTTON_IDX] > 0:
            self._choice = False

        # Book-keeping
        if self._choice is not None:
            self._make_a_choice = False
            self.notify_topic_message(JoystickTriggerAction.JOY_TOPIC, joy_msg)
