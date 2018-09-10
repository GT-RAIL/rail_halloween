#!/usr/bin/env python
# The dialogue manager in the local recovery strategy

from __future__ import print_function, division

import pickle
import numpy as np

from threading import Thread, Lock

import rospy
import actionlib

from sound_interface import SoundClient
from task_executor.actions import default_actions

from actionlib_msgs.msg import GoalStatus
from rail_people_detection_msgs.msg import Person


# The main dialogue manager class

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

    # Behavioural constants
    POSITION_CHANGE_HEAD_FOLLOW_THRESHOLD = 0.03
    POST_LOOK_SPEAK_WAIT = 1.5

    # Speech commands
    SPEECH_GREETING = 'GREETING'
    SPEECH_HEAR_CHECK = 'HEAR_CHECK'
    SPEECH_SMALL_TALK = 'SMALL_TALK'
    SPEECH_ASSIST_AGREE = 'ASSIST_AGREE'
    SPEECH_ASSIST_DISAGREE = 'ASSIST_DISAGREE'
    SPEECH_RESUME_CURRENT = 'RESUME_CURRENT'
    SPEECH_RESUME_NEXT = 'RESUME_NEXT'
    SPEECH_RESUME_RETRY = 'RESUME_RETRY'
    SPEECH_RESUME_NONE = 'RESUME_NONE'

    # Template texts
    SAY_HELLO = "Hello!"
    SAY_I_CAN_HEAR_YOU = "I can hear you"
    SAY_SMALL_TALK = "I am a rolling stone that gathers no moss"
    SAY_REQUEST_HELP = """
Excuse me, I have encountered an error and need help in my task. Could you
assist me?
    """
    SAY_THANKS = "Thank you!"
    SAY_INITIAL_REQUEST = """
I am failing to complete {component}, which is step {step_num} in my task plan.
I think the cause is {cause}.
    """
    SAY_INSTRUCTIONS = """
You can move me however you wish to when helping me. All my joints are pliable
right now. When you're done, say "I'm done!"
    """
    SAY_HOW_TO_PROCEED = "How should I proceed?"
    SAY_PROCEED_VALID_OPTIONS = """
I understand the phrases: "Retry failed action", "Continue to next
action", "Restart Task", and "Stop Executing"
    """
    SAY_BYEBYE = "Bye!"

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

        # Idle behaviours. These are normally always available, however, they
        # can be locked out during interactions
        self._idle_behaviour_lock = Lock()
        self._idle_behaviour_thread = Thread(target=self.run_idle_behaviours)

    def start(self):
        # Initialize the actions
        self.actions.init()

        # Start the idle behaviours
        self._idle_behaviour_thread.start()

    def run_idle_behaviours(self):
        speech_generator = self.actions.listen.run()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if not self._idle_behaviour_lock.acquire(False):
                continue

            # Query the speech recognition interface for speech. If the speech
            # is one of GREETING, HEAR_CHECK, or SMALL_TALK, respond
            # appropriately
            try:
                cmd = speech_generator.next()
            except StopIteration as e:
                cmd = cmd['cmd']
                if cmd == DialogueManager.SPEECH_GREETING:
                    self.actions.speak(text=DialogueManager.SAY_HELLO, affect=SoundClient.AFFECT_HAPPY)
                elif cmd == DialogueManager.SPEECH_HEAR_CHECK:
                    self.actions.speak(text=DialogueManager.SAY_I_CAN_HEAR_YOU, affect=SoundClient.AFFECT_CALM)
                elif cmd == DialogueManager.SPEECH_SMALL_TALK:
                    self.actions.speak(text=DialogueManager.SAY_SMALL_TALK, affect=SoundClient.AFFECT_CALM)
                speech_generator = self.actions.listen.run()

            # Remember to release the lock
            self._idle_behaviour_lock.release()

    def request_help(self, request, person):
        with self._idle_behaviour_lock:
            # Set the person and start looking at them
            self._should_look_at_person = True
            self.person = person

            # Wait a bit, then send out an auditory request for assistance
            start_time = rospy.Time.now()
            while rospy.Time.now() - start_time <= rospy.Duration(DialogueManager.POST_LOOK_SPEAK_WAIT):
                yield {}

            for variables in self.actions.speak.run(text=DialogueManager.SAY_REQUEST_HELP):
                yield variables

            # Wait for a response

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
                ) >= DialogueManager.POSITION_CHANGE_HEAD_FOLLOW_THRESHOLD:
            self.actions.look({
                'x': self.person.pose.position.x,
                'y': self.person.pose.position.y,
                'z': self.person.pose.position.z,
                'frame': self.person.header.frame_id,
            })
