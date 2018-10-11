#!/usr/bin/env python
# Monitor the costmap and robot's footprint to check if the robot is in
# collision

from __future__ import print_function, division

import numpy as np

import rospy
import tf

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from assistance_msgs.msg import ExecutionEvent

from assistance_arbitrator.monitors.trace_monitor import TraceMonitor


# The class definition

class BaseCollisionMonitor(object):
    """
    Monitor the local costmap and the robot's footprint to check if there is a
    collision
    """

    # Monitor specific configs
    BASE_COLLISION_MONITOR_EVENT_NAME = "base_collision_update"
    BASE_COLLISION_MONITOR_NODES = [
        "/move_base",
        "/amcl"
    ]
    MONITOR_DURATION = rospy.Duration(1.0)  # Duration at which to check collisions

    # Configuration for the costmap and other navigation params
    COSTMAP_TOPIC = "/move_base/local_costmap/costmap"
    ROBOT_RADIUS_PARAM = "/move_base/local_costmap/robot_radius"
    ROBOT_FOOTPRINT_NUM_POINTS = 20  # We're going to be naive and simply check the points themselves
    INSCRIBED_RADIUS_PARAM = "/move_base/local_costmap/inscribed_radius"
    CIRCUMSCRIBED_RADIUS_PARAM = "/move_base/local_costmap/circumscribed_radius"
    ROBOT_FRAME = "/base_link"

    # Constants defined in move_base
    CONST_NO_INFORMATION = 255
    CONST_LETHAL_OBSTACLE = 254
    CONST_INSCRIBED_INFLATED_OBSTACLE = 253
    CONST_FREE_SPACE = 0

    def __init__(self):
        self.base_in_collision = False
        self._latest_costmap = None

        # Get the move_base parameter values. Translating C++ -> Python here...
        self._footprint = self._make_footprint(rospy.get_param(BaseCollisionMonitor.ROBOT_RADIUS_PARAM))
        self._inscribed_radius = rospy.get_param(BaseCollisionMonitor.INSCRIBED_RADIUS_PARAM, 0.325)
        self._circumscribed_radius = rospy.get_param(BaseCollisionMonitor.CIRCUMSCRIBED_RADIUS_PARAM, 0.46)
        self._cost_translation_func = self._make_cost_translation_func()

        # tf listener
        self._listener = tf.TransformListener()

        # The trace publisher
        self._trace = rospy.Publisher(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=1
        )

        # The subscriber to the costmap
        self._costmap_sub = rospy.Subscriber(
            BaseCollisionMonitor.COSTMAP_TOPIC,
            OccupancyGrid,
            self._on_costmap
        )

        # Start the timer to periodically monitor for collisions
        self._monitor_timer = rospy.Timer(
            BaseCollisionMonitor.MONITOR_DURATION,
            self._monitor_func,
            oneshot=False
        )

    def _make_footprint(self, radius):
        footprint = []
        for i in xrange(BaseCollisionMonitor.ROBOT_FOOTPRINT_NUM_POINTS):
            angle = i * 2 * np.pi / BaseCollisionMonitor.ROBOT_FOOTPRINT_NUM_POINTS
            footprint.append(Point(
                x=(radius * np.cos(angle)),
                y=(radius * np.sin(angle))
            ))
        return footprint

    def _make_cost_translation_func(self):
        internal_table = {
            0: BaseCollisionMonitor.CONST_FREE_SPACE,
            -1: BaseCollisionMonitor.CONST_NO_INFORMATION,
            99: BaseCollisionMonitor.CONST_INSCRIBED_INFLATED_OBSTACLE,
            100: BaseCollisionMonitor.CONST_LETHAL_OBSTACLE,
        }
        func = lambda x: internal_table.get(x, int(1 + (251 * (x - 1)) / 97))
        return func

    def _get_map_coords(self, wx, wy, costmap):
        ox, oy, res = (costmap.info.origin.position.x,
                       costmap.info.origin.position.y,
                       costmap.info.resolution)
        size_x, size_y = costmap.info.width, costmap.info.height

        # Sanity check the inputs
        assert not (wx < ox or wy < oy), ("World coordinates {} not in map with origin {}"
                                          .format((wx, wy,), (ox, oy,)))

        # Calculate the map coordinate
        mx = (wx - ox) / res
        my = (wy - oy) / res

        # Sanity check the outputs
        assert (mx < size_x and my < size_y), ("Calculated coords {} larger than map size {}"
                                               .format((mx, my,), (size_x, size_y,)))

        # Return
        return (int(mx), int(my),)

    def _check_collision(self, x, y, costmap):
        base_in_collision = False
        for point in self._footprint:
            px, py = x + point.x, y + point.y
            mx, my = self._get_map_coords(px, py, costmap)
            cost = self._cost_translation_func(costmap.data[my * costmap.info.width + mx])
            base_in_collision = base_in_collision or (
                cost in [BaseCollisionMonitor.CONST_NO_INFORMATION,
                         BaseCollisionMonitor.CONST_LETHAL_OBSTACLE,
                         BaseCollisionMonitor.CONST_INSCRIBED_INFLATED_OBSTACLE]
            )

        return base_in_collision

    def _on_costmap(self, costmap_msg):
        self._latest_costmap = costmap_msg

    def _monitor_func(self, evt):
        # Cache the pointer to the costmap
        costmap = self._latest_costmap
        if costmap is None:
            return

        # Get the robot's position
        try:
            (trans, rot) = self._listener.lookupTransform(
                costmap.header.frame_id,
                BaseCollisionMonitor.ROBOT_FRAME,
                rospy.Time(0)
            )
            # Don't need the orientation as we have a circular footprint
            # _, _, theta = tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ExtrapolationException):
            return

        # Check the collision of each point in the robot's footprint
        old_collision_state = self.base_in_collision
        try:
            self.base_in_collision = self._check_collision(trans[0], trans[1], costmap)
        except Exception as e:
            rospy.logerr("Error checking collisions: {}".format(e))
            self.base_in_collision = False

        # If there has been an update in the collision check, then send an event
        if old_collision_state != self.base_in_collision:
            trace_event = ExecutionEvent(
                stamp=rospy.Time.now(),
                name=BaseCollisionMonitor.BASE_COLLISION_MONITOR_EVENT_NAME,
                type=ExecutionEvent.MONITOR_EVENT
            )
            trace_event.monitor_metadata.nodes.extend(BaseCollisionMonitor.BASE_COLLISION_MONITOR_NODES)
            trace_event.monitor_metadata.topics.append(BaseCollisionMonitor.COSTMAP_TOPIC)
            self._trace.publish(trace_event)


# For Debug only
if __name__ == '__main__':
    rospy.init_node('base_collision_monitor')
    monitor = BaseCollisionMonitor()
    rospy.spin()
