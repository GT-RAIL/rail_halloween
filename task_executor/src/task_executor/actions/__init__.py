#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_action import AbstractAction

from .move import MoveAction
from .torso import TorsoAction
from .look import LookAction
from .perceive import PerceiveAction
from .pick import PickAction
from .tuck import TuckAction
from .place import PlaceAction


class Actions(object):
    """Contains all the actions. DON'T USE THIS"""

    def __init__(self):
        # Painful manual specification of the actions
        self.registry = {
            'move': MoveAction(),
            'torso': TorsoAction(),
            'look': LookAction(),
            'perceive': PerceiveAction(),
            'pick': PickAction(),
            'tuck': TuckAction(),
            'place': PlaceAction(),
        }

        # Quick sanity check because I don't trust people
        for key, action in self.registry.iteritems():
            assert isinstance(action, AbstractAction)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self, locations, objects):
        for key, action in self.registry.iteritems():
            action.init(locations, objects)


# Make sure to only ever import actions
actions = Actions()
