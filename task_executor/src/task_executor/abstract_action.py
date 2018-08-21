#!/usr/bin/env python
# Creates the registry of action definitions to actions to use

from __future__ import print_function

import abc
from actionlib_msgs.msg import GoalStatus

class AbstractAction(object):
    """All actions are derived from this class"""

    __metaclass__ = abc.ABCMeta
    STATUS_KEY = '_status'
    RUNNING_GOAL_STATES = [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.RECALLING, GoalStatus.PREEMPTING]

    def running(self, **kwargs):
        """
        Returns a status denoting that the action is still running
        """
        return {}

    def succeeded(self, **kwargs):
        """
        Returns a status denoting that the action has succeeded
        """
        kwargs[AbstractAction.STATUS_KEY] = GoalStatus.SUCCEEDED
        return kwargs

    def aborted(self, **kwargs):
        """
        Returns a status denoting that the action has failed
        """
        kwargs[AbstractAction.STATUS_KEY] = GoalStatus.ABORTED
        return kwargs

    def preempted(self, **kwargs):
        """
        Returns a status denoting that the action was preempted
        """
        kwargs[AbstractAction.STATUS_KEY] = GoalStatus.PREEMPTED
        return kwargs

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
        Yield a non-empty dictionary or raise StopIteration to stop the run.

        Use `running()`, `succeeded()`, `aborted()`, `preempted()` helper
        functions to set status
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        """
        Stop the action. How this is handled by the client is up to the client.
        """
        raise NotImplementedError()
