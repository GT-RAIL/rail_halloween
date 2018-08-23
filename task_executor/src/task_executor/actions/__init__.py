#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_action import AbstractAction

from .move import MoveAction
from .torso import TorsoAction
from .look import LookAction
from .find_object import FindObjectAction
from .find_grasps import FindGraspsAction
from .perceive import PerceiveAction
from .pick import PickAction
from .arm_pose import ArmPoseAction
from .place import PlaceAction


class Actions(object):
    """Contains all the actions. DON'T USE THIS"""

    def __init__(self):
        # Painful manual specification of the actions
        self.registry = {
            'move': MoveAction(),
            'torso': TorsoAction(),
            'look': LookAction(),
            'find_object': FindObjectAction(),
            'find_grasps': FindGraspsAction(),
            'perceive': PerceiveAction(),
            'pick': PickAction(),
            'arm': ArmPoseAction(),
            'place': PlaceAction(),
        }

        # Quick sanity check because I don't trust people
        for key, action in self.registry.iteritems():
            assert isinstance(action, AbstractAction)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self, **kwargs):
        for key, action in self.registry.iteritems():
            action.init(**kwargs)


# Make sure to only ever import actions
actions = Actions()
