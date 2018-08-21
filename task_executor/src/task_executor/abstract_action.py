#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

import abc
from actionlib_msgs.msg import GoalStatus

class AbstractAction(object):
    """All actions are derived from this class"""

    __metaclass__ = abc.ABCMeta
    RUNNING_GOAL_STATES = [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.RECALLING, GoalStatus.PREEMPTING]

    def __init__(self):
        self._status = GoalStatus.LOST

    def set_running(self, **kwargs):
        """
        Returns a status denoting that the action is still running
        """
        self._status = GoalStatus.ACTIVE
        return kwargs

    def set_succeeded(self, **kwargs):
        """
        Returns a status denoting that the action has succeeded
        """
        self._status = GoalStatus.SUCCEEDED
        return kwargs

    def set_aborted(self, **kwargs):
        """
        Returns a status denoting that the action has failed
        """
        self._status = GoalStatus.ABORTED
        return kwargs

    def set_preempted(self, **kwargs):
        """
        Returns a status denoting that the action was preempted
        """
        self._status = GoalStatus.PREEMPTED
        return kwargs

    def is_running(self):
        """
        Checks to see if the current action is running. Counterpoint to
        `AbstractAction.running()`
        """
        return self._status == GoalStatus.ACTIVE

    def is_succeeded(self):
        """
        Checks to see if the current action is running. Counterpoint to
        `AbstractAction.succeeded()`
        """
        return self._status == GoalStatus.SUCCEEDED

    def is_preempted(self):
        """
        Checks to see if the current action was preempted. Counterpoint to
        `AbstractAction.preempted()`
        """
        return self._status == GoalStatus.PREEMPTED

    def is_aborted(self):
        """
        Checks to see if the current action is running. Counterpoint to
        `AbstractAction.aborted()`
        """
        return not (self.is_running() or self.is_succeeded() or self.is_preempted())

    @abc.abstractmethod
    def init(self, locations, objects):
        """
        Initialize the action.

        TODO: For now, we pass in the DB of locations and objects. In the near
        future, these should be provided by a separate node
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def run(self, **params):
        """
        Run the action. This method returns a generator. So don't return;
        instead yield an empty dictionary of return values to keep executing.
        Stop yielding values or raise StopIteration to stop the run.

        Use `running()`, `succeeded()`, `aborted()`, `preempted()` helper
        functions to set status when yielding
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        """
        Stop the action. How this is handled by the client is up to the client.
        """
        raise NotImplementedError()
