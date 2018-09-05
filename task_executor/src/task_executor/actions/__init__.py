#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_step import AbstractStep

from .move import MoveAction
from .torso import TorsoAction
from .look import LookAction
from .gripper import GripperAction
from .find_object import FindObjectAction
from .find_grasps import FindGraspsAction
from .pick import PickAction
from .arm_pose import ArmPoseAction
from .place import PlaceAction


class Actions(object):
    """Registry of actions"""

    def __init__(self, registry):
        self.registry = registry

        # Quick sanity check because I don't trust people
        for key, action in self.registry.iteritems():
            assert isinstance(action, AbstractStep)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self, **kwargs):
        for key, action in self.registry.iteritems():
            action.init(**kwargs)


# The default actions contain all the action interfaces that are known to this
# package
default_actions = Actions({
    'move': MoveAction(),
    'torso': TorsoAction(),
    'look': LookAction(),
    'gripper': GripperAction(),
    'find_object': FindObjectAction(),
    'find_grasps': FindGraspsAction(),
    'pick': PickAction(),
    'arm': ArmPoseAction(),
    'place': PlaceAction(),
})
