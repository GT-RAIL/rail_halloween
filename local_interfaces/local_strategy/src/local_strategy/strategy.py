#!/usr/bin/env python
# The main action server that provides local recovery behaviour


from __future__ import print_function, division

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import RequestAssistanceAction, RequestAssistanceResult


class LocalRecovery(object):
    """
    Given a request for assistance, this class interfaces with the robot's
    look, speech, and point modules to request assistance
    """

    def __init__(self):
        pass
