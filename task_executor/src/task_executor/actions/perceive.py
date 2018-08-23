#!/usr/bin/env python
# The perceive action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from .find_grasps import FindGraspsAction
from .find_object import FindObjectAction


class PerceiveAction(AbstractAction):
    # TODO: This should be another "task". Need to figure out the semantics

    def init(self, **kwargs):
        self._find_grasps = FindGraspsAction()
        self._find_object = FindObjectAction()

        self._find_grasps.init(**kwargs)
        self._find_object.init(**kwargs)

    def run(self, obj):
        # First find the object
        for variables in self._find_object.run(obj):
            yield self.set_running(**variables)

        # If the find failed
        if not self._find_object.is_succeeded():
            if self._find_object.is_preempted():
                yield self.set_preempted(**variables)
            else:
                yield self.set_aborted(**variables)
            raise StopIteration()

        # Then get the grasps
        found_idx, found_obj = variables['found_idx'], variables['found_obj']
        for variables in self._find_grasps.run(found_obj):
            yield self.set_running(**variables)

        grasps = variables['grasps']

        # Set the final return variables
        if self._find_grasps.is_succeeded():
            yield self.set_succeeded(grasps=grasps, cube_idx=found_idx)
        elif self._find_grasps.is_preempted():
            yield self.set_preempted(**variables)
        else:
            yield self.set_aborted(**variables)

    def stop(self):
        # Propagate the stopped flag to the sub actions
        self._find_grasps.stop()
        self._find_object.stop()
