#!/usr/bin/env python
# The dialogue manager in the local recovery strategy

from __future__ import print_function, division

import pickle

import rospy
import actionlib

from sound_client import SoundClient
from task_executor.actions import default_actions


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
        self._person = None

    def start(self):
        # Initialize the actions
        self.actions.init()

    def request_help(self, request, person):
        pass
