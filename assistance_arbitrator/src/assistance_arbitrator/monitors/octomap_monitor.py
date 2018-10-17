#!/usr/bin/env python
# Monitor the octomap and check if the threshold of clutter in the octomap is
# too high. This is almost never likely to be the actual cause of issues

from __future__ import print_function, division

import numpy as np

from threading import Lock

import rospy

from assistance_msgs.msg import MonitorMetadata
from moveit_msgs.msg import PlanningSceneComponents, PlanningScene
from moveit_msgs.srv import GetPlanningScene

from assistance_arbitrator.monitoring import AbstractFaultMonitor


# The class definition

class OctomapMonitor(AbstractFaultMonitor):
    """
    Monitor the octomap and return an indication of the amount of free space.
    This is a dumb algorithm for now
    """

    OCTOMAP_MONITOR_EVENT_NAME = "octomap_update"
    OCTOMAP_TOPIC = "/planning_scene"
    OCTOMAP_SERVICE = "/get_planning_scene"

    MONITOR_DURATION = rospy.Duration(5.0)  # Duration at which to check the octomap

    OCTOMAP_OCCUPIED_PROBABILITY_THRESHOLD = 0.5
    FREE_PERC_FAULT_THRESHOLD = 0.7  # Pretty arbitrary number

    def __init__(self):
        super(OctomapMonitor, self).__init__()
        self.set_metadata(topics=[OctomapMonitor.OCTOMAP_TOPIC])

        self._octomap = None
        self._octomap_lock = Lock()

        # The service for getting the octomap
        self._get_octomap = rospy.ServiceProxy(OctomapMonitor.OCTOMAP_SERVICE, GetPlanningScene)

        # The topic for listening to the planning scene
        self._octomap_sub = rospy.Subscriber(
            OctomapMonitor.OCTOMAP_TOPIC,
            PlanningScene,
            self._on_planning_scene
        )

        # Start the timer to periodically
        self._monitor_timer = rospy.Timer(
            OctomapMonitor.MONITOR_DURATION,
            self._monitor_func,
            oneshot=False
        )

    def _on_planning_scene(self, scene_msg):
        if len(scene_msg.world.octomap.octomap.data) > 0:
            with self._octomap_lock:
                self._octomap = np.array(scene_msg.world.octomap.octomap.data)

    def _monitor_func(self, evt):
        try:
            planning_scene = self._get_octomap(components=PlanningSceneComponents(PlanningSceneComponents.OCTOMAP))
        except rospy.ServiceException as e:
            rospy.logerr("Error fetching planning scene: {}".format(e))
            return

        # Update the octomap and then check the threshold
        new_octomap = np.array(planning_scene.scene.world.octomap.octomap.data)
        free_space_num = np.sum(
            (1 - (1 / (1 + np.exp(new_octomap))))  # Convert Log-Odds to Probabilities
            <= OctomapMonitor.OCTOMAP_OCCUPIED_PROBABILITY_THRESHOLD
        )
        free_space_perc = free_space_num / len(new_octomap)

        # Update the trace
        self.update_trace(
            OctomapMonitor.OCTOMAP_MONITOR_EVENT_NAME,
            free_space_perc < OctomapMonitor.FREE_PERC_FAULT_THRESHOLD,
            { 'free_space_perc': free_space_perc }
        )

        # Update the pointer to the octomap
        with self._octomap_lock:
            self._octomap = new_octomap


# When running the monitor in standalone mode
if __name__ == '__main__':
    rospy.init_node('global_plan_monitor')
    monitor = OctomapMonitor()
    rospy.spin()
