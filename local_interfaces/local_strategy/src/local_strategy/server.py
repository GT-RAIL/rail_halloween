#!/usr/bin/env python
# The main action server that provides local recovery behaviour

from __future__ import print_function, division

from threading import Lock

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import RequestAssistanceAction
from rail_people_detection_msgs.msg import Person, DetectionContext

from task_executor.actions import default_actions


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

        # Person detections
        self.selected_person = None             # This is person that we select
        self._last_closest_person = None        # These are the closest people
        self._select_closest_person = False     # Flag to trigger the save of a person
        self._closest_person_sub = rospy.Subscriber(
            "rail_people_detector/closest_person",
            Person,
            self._on_closest_person
        )

        # Initialize the actions that we are interested in using
        self.actions = default_actions
        self.actions.init()

    def start(self):
        self._server.start()
        rospy.loginfo("Local strategy node ready...")

    def execute(self, goal):
        """Execute the request for assistance"""
        result = self._server.get_default_result()
        result.stats.request_received = rospy.Time.now()

        # TODO: The actual error recovery mechanism
        # First we look for a person

        # Then we solicit help from them
        result.stats.request_acked = rospy.Time.now()

        result.stats.request_complete = rospy.Time.now()
        self._server.set_succeeded(result)

    def stop(self):
        pass

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
