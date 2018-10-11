#!/usr/bin/env python
# Monitor the robot's connection to the WiFi

from __future__ import print_function, division

import re
import psutil
import subprocess

import rospy

from assistance_msgs.msg import ExecutionEvent

from assistance_arbitrator.monitors.trace_monitor import TraceMonitor


# The class definition

class WifiMonitor(object):
    """
    Monitors the strength of the wifi signal on the robot and flags poor wifi
    if the signal strength goes below some threshold
    """

    WIFI_MONITOR_EVENT_NAME = "wifi_update"
    WIFI_POOR_SIGNAL_THRESHOLD = -67  # Signal strength in dBm below which we might have problems
    WIFI_BAD_SIGNAL_THRESHOLD = -75  # Signal strength in dBm below which we are screwed
    WIFI_MIN_SIGNAL = -120  # The absolute minimum wifi level

    MONITOR_DURATION = rospy.Duration(5.0)  # Duration at which to check wifi strength
    MONITOR_COMMAND = ["sudo", "iwconfig", "wlan0"]
    MONITOR_REGEX = re.compile(r'.*Signal level=(?P<signal_level>-\d+) dBm.*')

    def __init__(self):
        self.signal_level = WifiMonitor.WIFI_MIN_SIGNAL

        # Set the trace publisher
        self._trace = rospy.Publisher(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=1
        )

        # Set the timer to monitor the Wifi
        self._monitor_timer = rospy.Timer(
            WifiMonitor.MONITOR_DURATION,
            self._monitor_func,
            oneshot=False
        )

    def _monitor_func(self, evt):
        try:
            proc = psutil.Popen(WifiMonitor.MONITOR_COMMAND, stdout=subprocess.PIPE)
            (out, _) = proc.communicate()
        except Exception as e:
            rospy.logerr("Error checking WiFi status: {}".format(e))
            return

        # Try to get the signal level
        old_level = self.signal_level
        try:
            matches = WifiMonitor.MONITOR_REGEX.search(out)
            self.signal_level = int(matches.groupdict()['signal_level'])
        except Exception as e:
            rospy.logerr("Error parsing the Signal Level: {}".format(e))
            self.signal_level = WifiMonitor.WIFI_MIN_SIGNAL

        # Now check if we've crossed thresholds
        event_status = None
        if old_level < WifiMonitor.WIFI_POOR_SIGNAL_THRESHOLD <= self.signal_level:
            event_status = ExecutionEvent.WIFI_GOOD_CONNECTION
            rospy.loginfo("WiFi connection is GOOD")
        elif old_level < WifiMonitor.WIFI_BAD_SIGNAL_THRESHOLD <= self.signal_level:
            event_status = ExecutionEvent.WIFI_POOR_CONNECTION
            rospy.loginfo("WiFi connection is POOR")
        elif self.signal_level < WifiMonitor.WIFI_BAD_SIGNAL_THRESHOLD <= old_level:
            event_status = ExecutionEvent.WIFI_BAD_CONNECTION
            rospy.loginfo("WiFi connection is BAD")
        elif self.signal_level < WifiMonitor.WIFI_POOR_SIGNAL_THRESHOLD <= old_level:
            event_status = ExecutionEvent.WIFI_POOR_CONNECTION
            rospy.loginfo("WiFi connection is POOR")

        # Send out an event if thresholds are crossed
        if event_status is not None:
            trace_event = ExecutionEvent(
                stamp=rospy.Time.now(),
                name=WifiMonitor.WIFI_MONITOR_EVENT_NAME,
                type=ExecutionEvent.WIFI_EVENT,
                wifi_metadata=event_status
            )
            self._trace.publish(trace_event)


# For Debug purposes only
if __name__ == '__main__':
    rospy.init_node('wifi_monitor')
    monitor = WifiMonitor()
    rospy.spin()
