#!/usr/bin/env python
# The trigger based on the joystick buttons

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy


class TriggerAction(AbstractStep):

    JOY_TOPIC = "/joy"
    ACCEPT_BUTTON_IDX = 13
    REJECT_BUTTON_IDX = 15

    def init(self, name):
        self.name = name
        self.

        rospy.loginfo("Connecting to head_controller...")
        self._look_client.wait_for_server()
        rospy.loginfo("...head_controller connected")
