#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

from task_executor.abstract_step import AbstractStep

from .arm import ArmAction
from .background_task import BackgroundTaskAction
from .beep import BeepAction
from .detach_candy import DetachCandyAction
from .drop_candy import DropCandyAction
from .find_bag_pose import FindBagPoseAction
from .gripper import GripperAction
from .hat import HatAction
from .hotword_trigger import HotwordTriggerAction
from .joystick_trigger import JoystickTriggerAction
from .look import LookAction
from .look_at_gripper import LookAtGripperAction
# from .move import MoveAction
from .pick_candy import PickCandyAction
from .speak import SpeakAction
from .stir import StirAction
from .toggle_breakers import ToggleBreakersAction
from .torso import TorsoAction
from .verify_grasp import VerifyGraspAction
from .wait import WaitAction


class Actions(object):
    """Registry of actions"""

    def __init__(self, registry):
        """
        Args:
            registry (dict) : This is a dict of name -> Action class mappings
        """
        self.registry = { key: klass() for key, klass in registry.iteritems() }

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
    'arm': ArmAction,
    'background_task': BackgroundTaskAction,
    'beep': BeepAction,
    'detach_candy': DetachCandyAction,
    'drop_candy': DropCandyAction,
    'find_bag_pose': FindBagPoseAction,
    'gripper': GripperAction,
    'hat': HatAction,
    # 'hotword_trigger': HotwordTriggerAction,
    'joystick_trigger': JoystickTriggerAction,
    'look': LookAction,
    'look_at_gripper': LookAtGripperAction,
    # 'move': MoveAction,
    'pick_candy': PickCandyAction,
    'speak': SpeakAction,
    'stir': StirAction,
    'toggle_breakers': ToggleBreakersAction,
    'torso': TorsoAction,
    'verify_grasp': VerifyGraspAction,
    'wait': WaitAction,
}

def get_default_actions():
    return Actions(default_actions_dict)
