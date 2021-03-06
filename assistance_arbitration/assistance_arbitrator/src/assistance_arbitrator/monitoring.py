#!/usr/bin/env python
# The code in this file monitors the status of a task execution. It does so by
# listening in on the execution trace, monitoring outputs from fault detectors,
# and monitoring critical node outputs in the rosgraph. The output is combined
# monitoring data that can be used by some other diagnosis node.

from __future__ import print_function, division

import abc
import pickle
import collections
import networkx as nx

from threading import Lock

import rospy

from actionlib_msgs.msg import GoalStatus
from ros_topology_msgs.msg import Connection, Node as NodeMsg, Graph as GraphMsg
from assistance_msgs.msg import ExecutionEvent, MonitorMetadata


# Helper classes and functions

def get_action_name_from_topic(topic_name):
    """Get the name of the action from the topic's name"""
    for action_suffix in ROSGraphMonitor.ACTION_SUFFIXES:
        if topic_name.endswith(action_suffix):
            return (topic_name[:-len(action_suffix)], action_suffix)
    return (topic_name, "")


class Node(object):
    def __init__(self, name):
        self.name = name
        self.topics_provided = set()
        self.topics_used = set()
        self.actions_provided = set()
        self.actions_used = set()
        self.services_provided = set()
        self.services_used = set()

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.__str__()

    def provides_topic(self, topic):
        self.topics_provided.add(topic)
        return self

    def uses_topic(self, topic):
        self.topics_used.add(topic)
        return self

    def provides_action(self, action):
        self.actions_provided.add(action)
        return self

    def uses_action(self, action):
        self.actions_used.add(action)
        return self

    def provides_service(self, service):
        self.services_provided.add(service)
        return self

    def uses_service(self, service):
        self.services_used.add(service)
        return self


# The ROS class definitions


class AbstractFaultMonitor(object):
    """All fault monitors should derive from this base class"""

    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.fault_status = MonitorMetadata.NOMINAL
        self.topics = tuple()
        self.services = tuple()
        self.actions = tuple()
        self.nodes = tuple()

        # The trace publisher
        self._trace = rospy.Publisher(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=10
        )

    def set_metadata(self, topics=None, services=None, actions=None, nodes=None):
        """Set the metadata of the monitor. If an arg is None, its value is not
        updated"""
        self.topics = tuple(topics) if topics is not None else self.topics
        self.services = tuple(services) if services is not None else self.services
        self.actions = tuple(actions) if actions is not None else self.actions
        self.nodes = tuple(nodes) if nodes is not None else self.nodes

    def update_trace(self, event_name, fault_status, context=None, force=False):
        """
        Update the trace. If the fault_status is unchanged, do nothing,
        unless the force flag is set.

        fault_status can be a boolean value. If so, then a value of 'True'
        indicates the presence of a fault, while a value of 'False' indicates
        nominal operation.
        """
        if type(fault_status) == bool:
            fault_status = MonitorMetadata.NOMINAL if not fault_status else MonitorMetadata.FAULT

        if fault_status == self.fault_status and not force:
            return

        # Update the fault_status
        self.fault_status = fault_status
        trace_event = ExecutionEvent(
            stamp=rospy.Time.now(),
            name=event_name,
            type=ExecutionEvent.MONITOR_EVENT,
            monitor_metadata=MonitorMetadata(
                fault_status=self.fault_status,
                context=(pickle.dumps(context) if context is not None else ''),
                topics=self.topics,
                services=self.services,
                actions=self.actions,
                nodes=self.nodes
            )
        )
        self._trace.publish(trace_event)


class TraceMonitor(object):
    """
    This class monitors the execution trace messages and compiles the data into
    a events trace stream
    """

    EXECUTION_TRACE_TOPIC = '/execution_monitor/trace'
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


class ROSGraphMonitor(object):
    """
    This class monitors and maintains the knowledge of ROS graph dependencies
    related to the task executor node
    """

    ROSGRAPH_TOPOLOGY_TOPIC = "/topology"
    ROSGRAPH_MONITOR_EVENT_NAME = "rosgraph_update"
    TASK_EXECUTOR_SERVER_NAME = "/task_executor"
    ACTION_SUFFIXES = set(["/feedback", "/status", "/cancel", "/goal", "/result"])
    TOPICS_OF_NO_INTEREST = set([
        "/rosout",
        "/diagnostics",
        "/clock",
        "/topology",
        "/tf",
        "/tf_static"
    ])
    # The following are services that task_executor uses and ones that we're
    # interested in knowing the providers of. TODO: Automate this process...?
    SERVICES_OF_INTEREST = set([
        # arm
        "/database/arm_gripper_pose",
        "/database/arm_joint_pose",
        "/database/trajectory",
        "/execute_kinematic_path",
        # # find_grasps
        # "/suggester/suggest_grasps",
        # "/suggester/pairwise_rank",
        # # find_object
        # "/database/object_constraints",
        # "/rail_segmentation/segment_objects",
        # "/grasp_executor/add_object",
        # "/grasp_executor/clear_objects",
        # listen
        "/get_last_speech_cmd",
        # # move
        # "/database/waypoints",
        # # place
        # "/grasp_executor/drop_object",
        # toggle_breakers
        "/arm_breaker",
        "/base_breaker",
        "/gripper_breaker",
        # verify_grasp
        "/grasp_verifier/verify",
    ])

    def __init__(self):
        self._graph = nx.DiGraph()  # The ROS graph represented in a graph structure
        self._graph_lock = Lock()

        # Publisher for events to the trace
        self._trace = rospy.Publisher(
            TraceMonitor.EXECUTION_TRACE_TOPIC,
            ExecutionEvent,
            queue_size=1
        )

        # Subscribe to the topology updates
        self._topology_sub = rospy.Subscriber(
            ROSGraphMonitor.ROSGRAPH_TOPOLOGY_TOPIC,
            GraphMsg,
            self._on_topology
        )

    @property
    def graph(self):
        with self._graph_lock:
            graph = self._graph
        return graph

    @graph.setter
    def graph(self, graph):
        with self._graph_lock:
            self._graph = graph

    def _on_topology(self, graph_msg):
        # First parse out the nodes from the graph message
        nodes = {}
        topic_users = {}
        action_users = {}
        service_users = {}  # At this time, this is only /task_executor

        # Iterate through the message
        for node_msg in graph_msg.nodes:
            node = Node(node_msg.name)

            # Parse through the topics
            publishes = set(node_msg.publishes)
            subscribes = set(node_msg.subscribes)
            candidate_actions = {}  # action: [hypothesis-of-provide/use, set-of-seen-suffixes]
            for topic_name in publishes | subscribes:
                action_name, suffix = get_action_name_from_topic(topic_name)
                if suffix:  # This is potentially an action
                    if action_name not in candidate_actions.keys():
                        candidate_actions[action_name] = [Connection.BOTH, set()]

                    if suffix == "/goal" and topic_name in publishes:
                        candidate_actions[action_name][0] = Connection.OUT
                    elif suffix == "/goal" and topic_name in subscribes:
                        candidate_actions[action_name][0] = Connection.IN

                    candidate_actions[action_name][1].add(suffix)

                elif topic_name not in ROSGraphMonitor.TOPICS_OF_NO_INTEREST:  # This is a topic
                    if topic_name in publishes:
                        node.provides_topic(topic_name)

                    if topic_name in subscribes:
                        node.uses_topic(topic_name)
                        if topic_name not in topic_users:
                            topic_users[topic_name] = set()
                        topic_users[topic_name].add(node)

            # Now iterate through the potential actions. If they are actions,
            # add them as such. Otherwise, add to the topics
            for action_name, action_hyp in candidate_actions.iteritems():
                if action_hyp[0] != Connection.BOTH and action_hyp[1] == ROSGraphMonitor.ACTION_SUFFIXES:
                    if action_hyp[0] == Connection.OUT:
                        node.uses_action(action_name)
                        if action_name not in action_users:
                            action_users[action_name] = set()
                        action_users[action_name].add(node)
                    else:
                        node.provides_action(action_name)
                else:
                    for suffix in action_hyp[1]:
                        topic_name = action_name + suffix
                        if topic_name in publishes:
                            node.provides_topic(topic_name)
                        else:
                            node.uses_topic(topic_name)
                            if topic_name not in topic_users:
                                topic_users[topic_name] = set()
                            topic_users[topic_name].add(node)

            # If this is the task executor, then add in the services
            if node.name == ROSGraphMonitor.TASK_EXECUTOR_SERVER_NAME:
                for service_name in ROSGraphMonitor.SERVICES_OF_INTEREST:
                    node.uses_service(service_name)
                    if service_name not in service_users:
                        service_users[service_name] = set()
                    service_users[service_name].add(node)

            # Iterate through the services and add them accordingly
            for service_msg in node_msg.provides:
                if service_msg.name in ROSGraphMonitor.SERVICES_OF_INTEREST:
                    node.provides_service(service_msg.name)

            # Finally add this node to the set of nodes
            nodes[node.name] = node

        # Create the new graph from the graph message
        graph = nx.DiGraph()
        for node_name, node in nodes.iteritems():
            graph.add_node(node)

            # Add the action connections
            for action_name in node.actions_provided:
                if action_name not in action_users:
                    continue

                for action_user in action_users[action_name]:
                    graph.add_edge(node, action_user)
                    if len(graph[node][action_user]) == 0:
                        graph[node][action_user]['services'] = set()
                        graph[node][action_user]['actions'] = set()
                        graph[node][action_user]['topics'] = set()
                    graph[node][action_user]['actions'].add(action_name)

            # Add the service connections
            for service_name in node.services_provided:
                if service_name not in service_users:
                    continue

                for service_user in service_users[service_name]:
                    graph.add_edge(node, service_user)
                    if len(graph[node][service_user]) == 0:
                        graph[node][service_user]['services'] = set()
                        graph[node][service_user]['actions'] = set()
                        graph[node][service_user]['topics'] = set()
                    graph[node][service_user]['services'].add(service_name)

            # Add the topic connections
            for topic_name in node.topics_provided:
                if topic_name not in topic_users:
                    continue

                for topic_user in topic_users[topic_name]:
                    graph.add_edge(node, topic_user)
                    if len(graph[node][topic_user]) == 0:
                        graph[node][topic_user]['services'] = set()
                        graph[node][topic_user]['actions'] = set()
                        graph[node][topic_user]['topics'] = set()
                    graph[node][topic_user]['topics'].add(topic_name)

        # Finally update the old graph with this new graph and let the trace
        # know
        self.graph = graph
        self._trace.publish(ExecutionEvent(
            stamp=rospy.Time.now(),
            name=ROSGraphMonitor.ROSGRAPH_MONITOR_EVENT_NAME,
            type=ExecutionEvent.ROSGRAPH_EVENT
        ))


class ExecutionMonitor(object):
    """
    This class is the entrypoint into the combined information available to
    diagnosis nodes
    """

    def __init__(self):
        # Supplementary execution monitors
        self.rosgraph_monitor = ROSGraphMonitor()
        self.trace_monitor = TraceMonitor()

    def start(self):
        # Nothing needs to be done on a start
        pass


# For debug only
if __name__ == '__main__':
    # ROS Graph debug
    import matplotlib.pyplot as plt
    rospy.init_node('graph_monitor')
    monitor = ROSGraphMonitor()
    rate = rospy.Rate(1)
    nx.draw_shell(monitor.graph, with_labels=True, node_size=500)
    fig = plt.gcf()
    fig.show()
    fig.canvas.draw()
    while not rospy.is_shutdown():
        plt.clf()
        nx.draw_shell(monitor.graph, with_labels=True, node_size=500)
        fig.canvas.draw()
        rospy.loginfo("Showing plot")
        rate.sleep()
