#!/usr/bin/env
# The torso action in a task plan

import rospy

from fetch_gazebo_demo import GraspingClient

from task_executor.abstract_action import AbstractAction


class PerceiveAction(AbstractAction):

    def __init__(self):
        self._perceive_client = None
        self._objects = None

    def init(self, locations, objects, scene):
        self._objects = objects
        self._perceive_client = GraspingClient(scene)

    def run(self, obj):
        obj = obj.split('.', 1)[1]

        rospy.loginfo("Inspecting scene for object: {}".format(obj))
        bounds = self._objects[obj]['bounds']
        location = self._objects[obj]['location']
        yield {}

        self._perceive_client.updateScene()
        yield {}
        cube, grasps = self._perceive_client.getGraspableCube(bounds, location)
        if cube is not None:
            yield {'cube': cube, 'grasps': grasps}
