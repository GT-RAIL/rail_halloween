#!/usr/bin/env python
# The code in this file monitors the status of a task execution. It does so by
# listening in on the execution trace, monitoring outputs from fault detectors,
# and monitoring critical node outputs in the rosgraph. The output is combined
# monitoring data that can be used by some other diagnosis node.

from __future__ import print_function, division

import pickle
import Queue

import rospy

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import ExecutionEvent

from .monitors import (BaseStallMonitor, CostmapMonitor, LocalizationMonitor,
                       RobotStateMonitor, ROSGraphMonitor, TraceMonitor)


# The class definition

class ExecutionMonitor(object):
    """
    This class combines the output from multiple detectors and monitors into a
    coherent stream that can be used by a diagnosis element
    """

    def __init__(self):
        self.base_stall_monitor = BaseStallMonitor()
        self.costmap_monitor = CostmapMonitor()
        self.localization_monitor = LocalizationMonitor()
        self.robot_state_monitor = RobotStateMonitor()
        self.rosgraph_monitor = ROSGraphMonitor()
        self.trace_monitor = TraceMonitor()

    def start(self):
        # Nothing needs to be done on a start
        pass
