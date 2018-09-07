#!/usr/bin/env python
# This action server decides the arbitration method given an incoming request

from __future__ import print_function, division

import pickle

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from assistance_msgs.msg import RequestAssistanceAction, RequestAssistanceFeedback


# The server arbitrates who to send the request to

class AssistanceArbitrationServer(object):
    """
    Given a request for assistance, and some TBD models, the server uses
    the logic in this class to decide whether to request help from local or from
    remote human.
    """

    def __init__(self):
        # Instantiate the action clients connecting to each recovery strategy
        self._local_strategy_client = \
            actionlib.SimpleActionClient("local_strategy", RequestAssistanceAction)

        # Instantiate the action server to provide the arbitration
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            RequestAssistanceAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        rospy.loginfo("Connecting to local strategies...")
        self._local_strategy_client.wait_for_server()
        rospy.loginfo("...local strategies connected")

        self._server.start()
        rospy.loginfo("Assistance arbitration node ready...")

    def execute(self, goal):
        """Arbitrate an incoming request for assistance"""
        request_received = rospy.Time.now()

        # Pick the strategy
        feedback = RequestAssistanceFeedback(strategy="local_strategy")
        self._server.publish_feedback(feedback)

        # Forward directly to the local strategy client. Preempt if a preempt
        # request has also appeared
        self._local_strategy_client.send_goal(goal)
        while not self._local_strategy_client.wait_for_result(rospy.Duration(0.5)):
            if self._server.is_preempt_requested():
                self._local_strategy_client.cancel_goal()

        # Update the result
        status = self._local_strategy_client.get_state()
        result = self._local_strategy_client.get_result()
        result.stats.request_received = request_received

        # Return based on status
        if status == GoalStatus.SUCCEEDED:
            self._server.set_succeeded(result)
        elif status == GoalStatus.PREEMPTED:
            self._server.set_preempted(result)
        else:  # Usually, GoalStatus.ABORTED
            self._server.set_aborted(result)

    def stop(self):
        pass
