#!/usr/bin/env
# The torso action in a task plan

import rospy

from fetch_gazebo_demo import FollowTrajectoryClient

from task_executor.abstract_action import AbstractAction


class TorsoAction(AbstractAction):

    def __init__(self):
        self._torso_client = None

    def init(self, locations, objects, scene):
        self._torso_client = FollowTrajectoryClient(
            "torso_controller",
            ["torso_lift_joint"]
        )

    def run(self, height):
        rospy.loginfo("Torso to height: {}".format(height))
        yield {}
        self._torso_client.move_to([height,])
