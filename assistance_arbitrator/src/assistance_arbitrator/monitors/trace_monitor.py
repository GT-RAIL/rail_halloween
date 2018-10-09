#!/usr/bin/env python
# Monitor the trace of events that are sent out as traces

from __future__ import print_function, division

import pickle
import Queue

import rospy

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import ExecutionEvent


# The class definition

class TraceMonitor(object):
    """
    This class monitors the execution trace messages and compiles the data into
    a events trace stream
    """

    EXECUTION_TRACE_TOPIC = 'execution_monitor/trace'

    def __init__(self):
        # Book-keeping variables to keep track of the trace state
        self.task_steps = {}  # TODO: Debug only?
        self.trace = Queue.Queue()
        self.task_status = {  # TODO: Debug only?
            GoalStatus.ACTIVE: set(),
            GoalStatus.SUCCEEDED: set(),
            GoalStatus.ABORTED: set(),
            GoalStatus.PREEMPTED: set(),
        }

        # Setup the subscriber to track the trace
        self._trace_sub = rospy.Subscriber(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            self._on_trace_event
        )

    def _on_trace_event(self, event):
        # TODO: This is a bit of a stub, like most of this class
        # First remove the event's step from the status monitor so that we can
        # appropriately refresh its status
        if event.task_step_metadata.uuid in self.task_steps.keys() \
                and self.task_steps[event.task_step_metadata.uuid].task_step_metadata.status in self.task_status.keys():
            self.task_status[self.task_steps[event.task_step_metadata.uuid].task_step_metadata.status].discard(event.name)

        self.task_steps[event.task_step_metadata.uuid] = event
        self.trace.put(event)
        if event.task_step_metadata.status in self.task_status.keys():
            self.task_status[event.task_step_metadata.status].add(event.name)
        else:
            rospy.logwarn("Unexpected status in event: {}".format(event))


# TODO: This is a temporary debug measure
if __name__ == '__main__':
    rospy.init_node('execution_trace_monitor')
    monitor = TraceMonitor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Running: {}".format(monitor.task_status[GoalStatus.ACTIVE]))
        print("Succeeded: {}".format(monitor.task_status[GoalStatus.SUCCEEDED]))
        print("Preempted: {}".format(monitor.task_status[GoalStatus.PREEMPTED]))
        print("Aborted: {}".format(monitor.task_status[GoalStatus.ABORTED]))
        print("Trace Size: {}".format(len(monitor.trace.queue)))

        rate.sleep()
