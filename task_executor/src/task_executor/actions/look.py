#!/usr/bin/env
# The torso action in a task plan

import rospy

from fetch_gazebo_demo import PointHeadClient

from task_executor.abstract_action import AbstractAction


class LookAction(AbstractAction):

    def __init__(self):
        self._look_client = None

    def init(self, locations, objects, scene):
        self._look_client = PointHeadClient()

    def run(self, pose):
        rospy.loginfo("Looking at point: {}".format(pose))
        yield {}
        self._look_client.look_at(**pose)
