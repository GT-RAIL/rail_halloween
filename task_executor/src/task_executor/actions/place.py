#!/usr/bin/env python
# The place action in a task plan

import rospy

from fetch_gazebo_demo import GraspingClient

from task_executor.abstract_action import AbstractAction


class PlaceAction(AbstractAction):

    def __init__(self):
        self._place_client = None

    def init(self, locations, objects, scene):
        self._place_client = GraspingClient(scene)

    def run(self, cube, pose_stamped, pick_result):
        rospy.loginfo(
            "Placing {} at: {}".format(
                cube.name,
                [pose_stamped.pose.position.x,
                 pose_stamped.pose.position.y,
                 pose_stamped.pose.position.z]
            )
        )
        yield {}

        # Drop off 5 cm higher just to avoid collisions
        pose_stamped.pose.position.z += 0.05
        success = self._place_client.place(cube, pose_stamped, pick_result)

        # This is a bit weird and non-conventional. We yield on error; else just
        # stop execution
        if not success:
            yield {'success': success}
