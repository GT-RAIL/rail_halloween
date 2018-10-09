#!/usr/bin/env python
# Monitor the trace of events that are sent out as traces

from __future__ import print_function, division

import collections

import rospy

from assistance_msgs.msg import ExecutionEvent


# The class definition

class TraceMonitor(object):
    """
    This class monitors the execution trace messages and compiles the data into
    a events trace stream
    """

    EXECUTION_TRACE_TOPIC = 'execution_monitor/trace'
    MAX_TRACE_LENGTH = 9999

    def __init__(self):
        # Book-keeping variables to keep track of the trace state
        self.trace = collections.deque(maxlen=TraceMonitor.MAX_TRACE_LENGTH)

        # Setup the subscriber to track the trace
        self._trace_sub = rospy.Subscriber(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            self._on_trace_event
        )

    def _on_trace_event(self, event):
        self.trace.append(event)
