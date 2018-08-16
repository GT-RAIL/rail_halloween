#!/usr/bin/env
# An action server to execute a task plan.

from __future__ import print_function, division

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from task_executor.msg import ExecuteAction

from fetch_gazebo_demo import (MoveBaseClient,
                               FollowTrajectoryClient,
                               PointHeadClient,
                               GraspingClient)


# The actual action server to execute the tasks

class TaskServer(object):
    """
    Given the task to perform, this server yields control to sub clients. When
    the clients are done, it moves on to the next task. If the task fails, the
    server sends context to an arbitrator. Based on decisions from the
    arbitrator, the server then yields control to a recovery interface
    """

    def __init__(self):
        self._base = MoveBaseClient()
        self._torso = FollowTrajectoryClient(
            "torso_controller",
            ["torso_lift_joint"]
        )
        self._head = PointHeadClient()
        self._grasp = GraspingClient()

        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            ExecuteAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        self._server.start()

    def execute(self, goal):
        """
        Execute the given goal. Has a spec of ExecuteGoal.
        """
        result = self.get_default_result()
        rospy.loginfo("Executing goal: {}".format(goal.name))

        while not self.is_preempt_requested() and self.is_active():
            # TODO: Execute parts of the task plan
            pass
