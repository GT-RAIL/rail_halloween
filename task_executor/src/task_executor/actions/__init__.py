#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_step import AbstractStep

from .arm_pose import ArmPoseAction
from .beep import BeepAction
from .compliant_mode import CompliantModeAction
from .find_closest_person import FindClosestPersonAction
from .find_grasps import FindGraspsAction
from .find_object import FindObjectAction
from .gripper import GripperAction
from .listen import ListenAction
from .look import LookAction
from .look_at_closest_person import LookAtClosestPersonAction
from .move import MoveAction
from .pick import PickAction
from .place import PlaceAction
from .speak import SpeakAction
from .torso import TorsoAction


class Actions(object):
    """Registry of actions"""

    def __init__(self, registry):
        self.registry = registry

        # Quick sanity check because I don't trust people. Also set the action
        # as an attribute for '.' based referencing
        for key, action in self.registry.iteritems():
            assert isinstance(action, AbstractStep)
            setattr(self, key, action)

    def __getitem__(self, key):
        return self.registry[key]

    def init(self):
        for key, action in self.registry.iteritems():
            action.init(key)


# The default actions contain all the action interfaces that are known to this
# package
default_actions_dict = {
    'arm': ArmPoseAction(),
    'beep': BeepAction(),
    'compliant_mode': CompliantModeAction(),
    'find_closest_person': FindClosestPersonAction(),
    'find_grasps': FindGraspsAction(),
    'find_object': FindObjectAction(),
    'gripper': GripperAction(),
    'listen': ListenAction(),
    'look': LookAction(),
    'look_at_closest_person': LookAtClosestPersonAction(),
    'move': MoveAction(),
    'pick': PickAction(),
    'place': PlaceAction(),
    'speak': SpeakAction(),
    'torso': TorsoAction(),
}
default_actions = Actions(default_actions_dict)
