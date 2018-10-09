#!/usr/bin/env python
# Monitor and update the rosgraph dependencies when a request to do so is sent

from __future__ import print_function, division

import os
import networkx as nx

from threading import Lock

import rospy

from assistance_msgs.msg import ExecutionEvent
from ros_topology_msgs.msg import Connection, Graph as GraphMsg


# The class definition

class ROSGraphMonitor(object):
    """
    This class monitors and maintains the knowledge of ROS graph dependencies
    related to the task executor node
    """

    ROSGRAPH_TOPOLOGY_TOPIC = "/topology"
    TASK_EXECUTOR_SERVER_NAME = "/task_executor"
    ACTION_SUFFIXES = ["/feedback", "/status", "/cancel", "/goal", "/result"]
    SERVICES_OF_INTEREST = set([
        # arm
        "/database/arm_pose",
        "/database/trajectory",
        # find_grasps
        "/suggester/suggest_grasps",
        "/suggester/pairwise_rank",
        # find_object
        "/database/object_constraints",
        "/rail_segmentation/segment_objects",
        "/grasp_executor/add_object",
        "/grasp_executor/clear_objects",
        # listen
        "/get_last_speech_cmd",
        # move
        "/database/waypoints",
        # place
        "/grasp_executor/drop_object",
        # toggle_breakers
        "/arm_breaker",
        "/base_breaker",
        "/gripper_breaker",
    ])

    def __init__(self):
        self.graph = nx.Graph()  # The ROS graph represented in a graph structure
        self._graph_lock = Lock()

        # FIXME: Debug
        self.graph.add_nodes_from([1,2,2,3,4,5,5,6,])

        # Subscribe to the topology updates
        self._topology_sub = rospy.Subscriber(
            ROSGraphMonitor.ROSGRAPH_TOPOLOGY_TOPIC,
            GraphMsg,
            self._on_topology
        )

    def _on_topology(self, graph_msg):
        graph = nx.Graph()


# For debug only
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    rospy.init_node('graph_monitor')
    monitor = ROSGraphMonitor()
    rate = rospy.Rate(1)
    nx.draw(monitor.graph)
    fig = plt.gcf()
    fig.show()
    fig.canvas.draw()
    while not rospy.is_shutdown():
        plt.clf()
        nx.draw(monitor.graph)
        fig.canvas.draw()
        rospy.loginfo("Showing plot")
        rate.sleep()
