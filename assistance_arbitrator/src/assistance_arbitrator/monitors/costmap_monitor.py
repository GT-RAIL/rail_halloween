#!/usr/bin/env python
# Monitor the costmap and store the percentage of occupied vs. free space

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy

from nav_msgs.msg import OccupancyGrid


# The class definition

class CostmapMonitor(object):
    """
    Monitor the local costmap and provide an indication of amount of free space.
    This is a dumb algorithm for now
    """

    COSTMAP_TOPIC = "/move_base/local_costmap/costmap"

    def __init__(self):
        self._free_space_perc = 1.0
        self._lock = Lock()

        # Subscribe to the costmap
        self._costmap_sub = rospy.Subscriber(
            CostmapMonitor.COSTMAP_TOPIC,
            OccupancyGrid,
            self._on_costmap
        )

    @property
    def free_space_perc(self):
        with self._lock:
            return self._free_space_perc

    @free_space_perc.setter
    def free_space_perc(self, free_space_perc):
        with self._lock:
            self._free_space_perc = free_space_perc

    def _on_costmap(self, costmap_msg):
        self.free_space_perc = 1 - (np.count_nonzero(costmap_msg.data) / len(costmap_msg.data))


# For debug only
if __name__ == '__main__':
    rospy.init_node('costmap_monitor')
    monitor = CostmapMonitor()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo("Percentage of free space: {}".format(monitor.free_space_perc))
        rate.sleep()
