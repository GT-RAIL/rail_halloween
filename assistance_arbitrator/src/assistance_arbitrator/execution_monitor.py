#!/usr/bin/env python
# The code in this file monitors the status of a task execution. It does so by
# listening in on the execution trace, monitoring outputs from fault detectors,
# and monitoring critical node outputs in the rosgraph

from __future__ import print_function, division

import pickle
import Queue

import rospy

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import ExecutionEvent


# The class definition

class ExecutionMonitor(object):
    """
    This class contains the logic to parse out the execution events during a
    task execution

    TODO: This is a stub class for debugging at the moment. Should add more to
    the class in the future.
    """

    EXECUTION_TRACE_TOPIC = '/execution_trace'

    def __init__(self):
        # Book-keeping variables to keep track of the state
        self.step_monitor = {}
        self.trace_monitor = Queue.Queue()
        self.status_monitor = {
            GoalStatus.ACTIVE: set(),
            GoalStatus.SUCCEEDED: set(),
            GoalStatus.ABORTED: set(),
            GoalStatus.PREEMPTED: set(),
        }

        # Setup the subscriber to the trace
        self._trace_sub = rospy.Subscriber(
            ExecutionMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            self._on_trace_event
        )

    def _on_trace_event(self, event):
        # TODO: This is a bit of a stub, like most of this class
        # First remove the event's step from the status monitor so that we can
        # appropriately refresh its status
        if event.uuid in self.step_monitor.keys() \
                and self.step_monitor[event.uuid].status in self.status_monitor.keys():
            self.status_monitor[self.step_monitor[event.uuid].status].discard(event.name)

        self.step_monitor[event.uuid] = event
        self.trace_monitor.put(event)
        if event.status in self.status_monitor.keys():
            self.status_monitor[event.status].add(event.name)
        else:
            rospy.logwarn("Unexpected status in event: {}".format(event))


# TODO: This is a temporary debug measure
if __name__ == '__main__':
    rospy.init_node('execution_trace_monitor')
    monitor = ExecutionMonitor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Running: {}".format(monitor.status_monitor[GoalStatus.ACTIVE]))
        print("Succeeded: {}".format(monitor.status_monitor[GoalStatus.SUCCEEDED]))
        print("Preempted: {}".format(monitor.status_monitor[GoalStatus.PREEMPTED]))
        print("Aborted: {}".format(monitor.status_monitor[GoalStatus.ABORTED]))
        print("Trace Size: {}".format(len(monitor.trace_monitor.queue)))

        rate.sleep()
