#!/usr/bin/env python
# This node listens to updates on the execution trace and monitors the status
# of task execution based on the updates there

from __future__ import print_function, division

import pickle

import rospy

from actionlib_msgs.msg import GoalStatus
from task_executor.msg import ExecutionEvent


# The class definition

class ExecutionTraceMonitor(object):
    """
    This class contains the logic to parse out the execution events during a
    task execution

    TODO: This is a stub class for debugging at the moment. Should add more to
    the class in the future.
    """

    EXECUTION_TRACE_TOPIC = '/execution_trace'

    def __init__(self):
        self.running_events = set()
        self.succeeded_events = []
        self.aborted_events = []
        self.preempted_events = []

        # Setup the subscriber to the trace
        self._trace_sub = rospy.Subscriber(
            ExecutionTraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            self._on_trace_event
        )

    def _on_trace_event(self, event):
        # TODO: This is a bit of a stub, like most of this class
        if event.status == GoalStatus.ACTIVE:
            self.running_events.add(event.name)
        else:
            if event.name in self.running_events:
                self.running_events.remove(event.name)

            if event.name == 'look_look_at_gripper':
                return

            if event.status == GoalStatus.SUCCEEDED:
                self.succeeded_events.append(event.name)
            elif event.status == GoalStatus.ABORTED:
                self.aborted_events.append((event.name, pickle.loads(event.context),))
            elif event.status == GoalStatus.PREEMPTED:
                self.preempted_events.append(event.name)
            else:
                rospy.logwarn("Unrecognized event status: {}".format(event.status))


# TODO: This is a temporary debug measure
if __name__ == '__main__':
    rospy.init_node('execution_trace_monitor')
    monitor = ExecutionTraceMonitor()

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print("Running: {}".format(monitor.running_events))
        print("Succeeded: {}".format(monitor.succeeded_events))
        print("Preempted: {}".format(monitor.preempted_events))
        print("Aborted: {}".format(monitor.aborted_events))

        rate.sleep()
