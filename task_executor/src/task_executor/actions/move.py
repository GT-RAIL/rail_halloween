#!/usr/bin/env
# The move action in a task plan

import rospy

from fetch_gazebo_demo import MoveBaseClient

from task_executor.abstract_action import AbstractAction


class MoveAction(AbstractAction):
    """Move to a location"""

    def __init__(self):
        self._move_base_client = None
        self._locations = None

    def init(self, locations, objects):
        self._locations = locations
        self._move_base_client = MoveBaseClient()

    def run(self, location):
        # Parse out the location. This will error if the location format is
        # incorrect
        location = location.split('.', 1)[1]

        rospy.loginfo("Moving to location: {}".format(location))
        coords = self._locations[location]
        yield {}

        for coord in coords:
            self._move_base_client.goto(**coord)
            yield {}
