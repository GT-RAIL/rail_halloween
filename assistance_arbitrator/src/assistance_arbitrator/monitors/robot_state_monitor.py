#!/usr/bin/env python
# Monitor the robot driver and report an error in case any of the breakers
# changes state

from __future__ import print_function, division

import collections

import rospy

from assistance_msgs.msg import ExecutionEvent, MonitorMetadata
from fetch_driver_msgs.msg import RobotState

from assistance_arbitrator.monitors.trace_monitor import TraceMonitor


# The class definition

class RobotStateMonitor(object):
    """
    Monitor the robot state and send out an event alert when there are
    meaningful robot state changes
    """

    ROBOT_STATE_TOPIC = "/robot_state"
    ROBOT_STATE_MONITOR_EVENT_NAME = "robot_state_update"
    MAX_CHANGES_TO_LOG = 9999

    def __init__(self):
        self.last_breaker_states = None
        self.logged_changes = collections.deque(maxlen=RobotStateMonitor.MAX_CHANGES_TO_LOG)

        # Initialize the trace publisher
        self._trace = rospy.Publisher(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=1
        )

        # Set up the subscriber
        self._robot_state_sub = rospy.Subscriber(
            RobotStateMonitor.ROBOT_STATE_TOPIC,
            RobotState,
            self._on_robot_state
        )

    def _on_robot_state(self, state_msg):
        if self.last_breaker_states is None:
            self.last_breaker_states = state_msg.breakers
            return

        # We only care about the breakers, so iterate through them looking for
        # a change
        changes_to_log = set()
        for idx, breaker_state_msg in enumerate(state_msg.breakers):
            if breaker_state_msg.state != self.last_breaker_states[idx].state:
                changes_to_log.add(
                    (breaker_state_msg.name,
                     self.last_breaker_states[idx].state,
                     breaker_state_msg.state)
                )

        # If the state has changed, log an event
        if len(changes_to_log) > 0:
            self.logged_changes.append(changes_to_log)
            trace_event = ExecutionEvent(
                stamp=rospy.Time.now(),
                name=RobotStateMonitor.ROBOT_STATE_MONITOR_EVENT_NAME,
                type=ExecutionEvent.MONITOR_EVENT
            )
            trace_event.monitor_metadata.topics.append(RobotStateMonitor.ROBOT_STATE_TOPIC)
            self._trace.publish(trace_event)

        # Save the last state
        self.last_breaker_states = state_msg.breakers


# For debug only
if __name__ == '__main__':
    rospy.init_node('robot_state_monitor')
    monitor = RobotStateMonitor()
    rospy.spin()
