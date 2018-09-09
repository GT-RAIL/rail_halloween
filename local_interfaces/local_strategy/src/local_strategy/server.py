#!/usr/bin/env python
# The main action server that provides local recovery behaviour

from __future__ import print_function, division

import pickle
import numpy as np

from threading import Lock

import rospy
import actionlib
import tf

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import RequestAssistanceAction, RequestAssistanceResult
from rail_people_detection_msgs.msg import Person, DetectionContext
from task_executor.msg import Bounds
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

        # tf
        self._tf_listener = tf.TransformListener()

        # Person detections
        self.selected_person = None             # This is person that we select
        self._last_closest_person = None        # These are the closest people
        self._select_closest_person = False     # Flag to trigger the save of a person
        self._closest_person_sub = rospy.Subscriber(
            "rail_people_detector/closest_person",
            Person,
            self._on_closest_person
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

        # Abuse python type conversion to change the data-type of the context
        # member
        goal.context = pickle.loads(goal.context)
        rospy.loginfo("Serving Assistance Request for: {} ({}) - {}"
                      .format(goal.component, goal.component_status, goal.context.keys()))

        # The actual error recovery mechanism
        # Sad beep first
        self.actions.beep(beep=SoundClient.BEEP_SAD, async=True)

        # First we look for a person
        for status in self.look_for_person():
            if self._server.is_preempt_requested() or not self._server.is_active():
                self._server.set_preempted(result)
                return

        self.actions.beep(beep=SoundClient.BEEP_EXCITED)

        # Then we solicit help from them
        result.stats.request_acked = rospy.Time.now()
        self._enter_compliant_mode()

        # Return when the request for help is completed
        self._exit_compliant_mode()
        result.resume_hint = RequestAssistanceResult.RESUME_CONTINUE
        result.stats.request_complete = rospy.Time.now()
        self._server.set_succeeded(result)

    def stop(self):
        pass

    def look_for_person(self):
        # Define the region of interest where we expect to find a person
        roi = Bounds(xmin=0.4, xmax=3.6, ymin=-2.4, ymax=2.4, zmin=1.4, zmax=2.0)
        step_sizes = { 'x': 0.8, 'y': 1.2, 'z': 0.3 }

        # Set the find person flag and iterate through the volume that we've
        # defined. Make sure to exit the moment the person is found
        self._select_closest_person = True
        while self._select_closest_person:

            z = roi.zmin
            while z <= roi.zmax and self._select_closest_person:

                x = roi.xmin
                while x <= roi.xmax and self._select_closest_person:

                    y = roi.ymin
                    while y <= roi.ymax and self._select_closest_person:
                        location = {'x': x, 'y': y, 'z': z, 'frame': 'base_link'}
                        self.actions.look(pose=location)

                        # Wait for a detection, but yield control
                        start_time = rospy.Time.now()
                        while rospy.Time.now() <= start_time + rospy.Duration(1.5):
                            yield GoalStatus.ACTIVE

                        y += step_sizes['y']

                    x += step_sizes['x']
                    if not (z + step_sizes['z'] > roi.zmax and x > roi.xmax) \
                            and self._select_closest_person:
                        self.actions.beep(beep=SoundClient.BEEP_UNSURE, async=True)

                z += step_sizes['z']

            if self._select_closest_person:
                self.actions.beep(beep=SoundClient.BEEP_CONCERNED, async=True)

    def _on_closest_person(self, msg):
        self._last_closest_person = msg

        # If we don't need to save the latest person as our selected target,
        # exit this callback
        if not self._select_closest_person:
            return

        # Choose this person only if they have a face
        if msg.detection_context.pose_source == DetectionContext.POSE_FROM_FACE:
            self.selected_person = self._last_closest_person
            self._select_closest_person = False

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
