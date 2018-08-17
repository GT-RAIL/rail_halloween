#!/usr/bin/env
# An action server to execute a task plan.

from __future__ import print_function, division

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from task_executor.msg import ExecuteAction
from std_srvs.srv import Trigger, TriggerResponse


# The actual action server to execute the tasks

class TaskServer(object):
    """
    Given the task to perform, this server yields control to sub clients. When
    the clients are done, it moves on to the next task. If the task fails, the
    server sends context to an arbitrator. Based on decisions from the
    arbitrator, the server then yields control to a recovery interface
    """

    def __init__(self):
        # Provide a service to reload, and then reload
        self._reload_service = rospy.Service('~reload', Trigger, self.reload)
        self.reload(None)

        # Instantiate the action server
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            ExecuteAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        self._server.start()

    def reload(self, req):
        # Instantiate the DB of locations and objects
        self.locations = rospy.get_param('~locations')
        self.objects = rospy.get_param('~objects')
        self.task_plan = rospy.get_param('~task')

        # Instantiate the registry of actions
        return TriggerResponse(success=True)

    def execute(self, goal):
        """
        Execute the given goal. Has a spec of ExecuteGoal.
        """
        result = self.get_default_result()
        rospy.loginfo("Executing goal: {}".format(goal.name))

        # Instantiate the dictionary of local variables
        var = dict()

        # Go through each step of the specified plan
        for idx, step in self.task_plan:

            # Continue running the step unless we request a preempt, or the goal
            # is no longer active
            while not self.is_preempt_requested() and self.is_active():
                # TODO: Execute parts of the task plan
                pass
