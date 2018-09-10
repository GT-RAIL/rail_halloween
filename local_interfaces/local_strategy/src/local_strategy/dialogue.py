#!/usr/bin/env python
# The dialogue manager in the local recovery strategy

from __future__ import print_function, division

import pickle
import numpy as np

import rospy
import actionlib

from sound_interface import SoundClient
from task_executor.actions import default_actions

from rail_people_detection_msgs.msg import Person


# The main manager class

class DialogueManager(object):
    """
    This class manages dialogue with the person that with we seek to get help
    from. Basically, given the help request, the manager generates target
    speech similar to: "Excuse me. Could you please help me? I am failing to
    complete <component>, which is step <step_num>. I think the cause is
    <cause>."

    Other tasks of the manager:
    - List the other components that might have failed and the likely causes for
        those failures
    - If the person wants the robot to perform autonomous actions, then request
        safety checks before acting.
    - Look out for the keyword "STOP". If found, stop all actuation.

    When the person is done, the manager should also handle the dialog to resume
    execution according to the policies outlined in RequestAssistanceResult
    """

    # Topic name constants
    CLOSEST_PERSON_TOPIC = "rail_people_detector/closest_person"

    # Template texts
    REQUEST_HELP = """
Excuse me, I have encountered an error and need help in my task. Could you
assist me?
    """
    THANK = "Thank you!"
    INITIAL_PRESENT_CAUSE = """
I am failing to complete {component}, which is step {step_num} in my task plan.
I think the cause is {cause}.
    """
    INSTRUCTIONS = """
You can move me however you wish to when helping me. All my joints are pliable
right now. When you're done, say "I'm done!"
    """
    HOW_TO_PROCEED_PROMPT = "How should I proceed?"
    PROCEED_VALID_OPTIONS = """
I understand the phrases: "Retry failed action", "Continue to next
action", "Restart Task", and "Stop Executing"
    """
    BYEBYE = "Bye!"

    def __init__(self):
        # Load the actions that are available to us
        self.actions = default_actions

        # Variable to keep track of the person to interact with
        self.person = None

        # The listener to the nearest person so that we can always look at them
        self._closest_person_sub = rospy.Subscriber(
            DialogueManager.CLOSEST_PERSON_TOPIC,
            Person,
            self._on_closest_person
        )
        self._should_look_at_person = False

    def start(self):
        # Initialize the actions
        self.actions.init()

    def request_help(self, request, person):
        # Set the person and start looking at them
        self._should_look_at_person = True
        self.person = person

        # Wait a bit, then send out an auditory request for assistance
        pass

    def _on_closest_person(self, msg):
        # We have nothing to do if we're not tracking a person
        if self.person is None:
            return

        # We are tracking a person, so make sure that the person we're tracking
        # matches the ID of the current closest person
        if self.person.id != msg.id:
            return

        # Update the person
        old_person = self.person
        self.person = msg

        # If we should look at the person, run the look command
        if self._should_look_at_person \
                and np.sqrt(
                    (old_person.pose.position.x - self.person.pose.position.x) ** 2
                    + (old_person.pose.position.y - self.person.pose.position.y) ** 2
                    + (old_person.pose.position.z - self.person.pose.position.z) ** 2
                ) >= 0.03:
            self.actions.look({
                'x': self.person.pose.position.x,
                'y': self.person.pose.position.y,
                'z': self.person.pose.position.z,
                'frame': self.person.header.frame_id,
            })
