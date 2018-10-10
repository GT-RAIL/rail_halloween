#!/usr/bin/env python
# Monitor the localization of the robot and send out an event if it updates by
# more than a threshold within some time

from __future__ import print_function, division

import numpy as np

import rospy
import tf

from assistance_msgs.msg import ExecutionEvent, MonitorMetadata

from assistance_arbitrator.monitors.trace_monitor import TraceMonitor


# The class definition

class LocalizationMonitor(object):
    """
    Monitors tf and sends out an alert event if the localization of the robot
    updates by more than some pre-specified threshold.
    """

    LOCALIZATION_MONITOR_EVENT_NAME = "localization_update"
    LOCALIZATION_MONITOR_NODE = "/amcl"

    MONITORING_LOOP_RATE = 10  # Hz
    MONITORING_LINEAR_ERROR_BOUND = 0.03  # 3 cm bounds on the error between checks
    MONITORING_ANGULAR_ERROR_BOUND = 0.05  # Approx 6 degrees of error between checks

    MAP_FRAME = "/map"
    BASE_FRAME = "/base_link"

    MOVE_BASE_LOCAL_PLANNER_PARAM = "/move_base/base_local_planner"
    EBAND_MAX_LINEAR_VELOCITY_PARAM = "/move_base/EBandPlannerROS/max_vel_lin"
    EBAND_MAX_ANGULAR_VELOCITY_PARAM = "/move_base/EBandPlannerROS/max_vel_th"
    TRAJ_MAX_LINEAR_VELOCITY_PARAM = "/move_base/TrajectoryPlannerROS/max_vel_x"
    TRAJ_MAX_ANGULAR_VELOCITY_PARAM = "/move_base/TrajectoryPlannerROS/max_vel_theta"

    def __init__(self):
        # Initialize the tf listener
        self._listener = tf.TransformListener()

        # Get the max velocity params from move_base
        base_local_planner = rospy.get_param(LocalizationMonitor.MOVE_BASE_LOCAL_PLANNER_PARAM)
        if base_local_planner == "eband_local_planner/EBandPlannerROS":
            self.max_vel_linear = rospy.get_param(LocalizationMonitor.EBAND_MAX_LINEAR_VELOCITY_PARAM)
            self.max_vel_angular = rospy.get_param(LocalizationMonitor.EBAND_MAX_ANGULAR_VELOCITY_PARAM)
        elif base_local_planner == "base_local_planner/TrajectoryPlannerROS":
            self.max_vel_linear = rospy.get_param(LocalizationMonitor.TRAJ_MAX_LINEAR_VELOCITY_PARAM)
            self.max_vel_angular = rospy.get_param(LocalizationMonitor.TRAJ_MAX_ANGULAR_VELOCITY_PARAM)
        else:
            raise KeyError("Unknown base_local_planner: {}".format(base_local_planner))

        # Stub values to save the last check
        self._last_position = np.zeros((3,))
        self._last_orientation = np.zeros((4,))

        # Publisher for the trace event
        self._trace = rospy.Publisher(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=1
        )

        # Start the monitor
        self._monitor_timer = rospy.Timer(
            rospy.Duration(1 / LocalizationMonitor.MONITORING_LOOP_RATE),
            self._monitor_func,
            oneshot=False
        )

    def _monitor_func(self, evt):
        # First try to lookup the transform and convert to numpy arrays
        try:
            (trans, rot) = self._listener.lookupTransform(
                LocalizationMonitor.MAP_FRAME,
                LocalizationMonitor.BASE_FRAME,
                rospy.Time(0)
            )
            trans, rot = np.array(trans), np.array(rot)
        except (tf.LookupException, tf.ExtrapolationException):
            return

        # Check the limits
        max_bound_linear = (
            self.max_vel_linear / LocalizationMonitor.MONITORING_LOOP_RATE
            + LocalizationMonitor.MONITORING_LINEAR_ERROR_BOUND
        )
        max_bound_angular = (
            self.max_vel_angular / LocalizationMonitor.MONITORING_LOOP_RATE
            + LocalizationMonitor.MONITORING_ANGULAR_ERROR_BOUND
        )
        is_outside_bounds = False
        if np.linalg.norm(trans - self._last_position) >= max_bound_linear:
            is_outside_bounds = True
        if (1 - np.dot(rot, self._last_orientation)**2) >= max_bound_angular:
            is_outside_bounds = True

        # If outside the bounds, send an event update
        if is_outside_bounds:
            trace_event = ExecutionEvent(
                stamp=rospy.Time.now(),
                name=LocalizationMonitor.LOCALIZATION_MONITOR_EVENT_NAME
            )
            trace_event.monitor_metadata.nodes.append(
                LocalizationMonitor.LOCALIZATION_MONITOR_NODE
            )
            self._trace.publish(trace_event)

        # Update the values of the saved pose
        self._last_position = trans
        self._last_orientation = rot


# Used for debugging only
if __name__ == '__main__':
    rospy.init_node("localization_monitor")
    monitor = LocalizationMonitor()
    rospy.spin()
