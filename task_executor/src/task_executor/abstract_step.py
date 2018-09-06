#!/usr/bin/env python
# The abstract base class that defines an executable step in the task execution
# process

from __future__ import print_function

import abc
from actionlib_msgs.msg import GoalStatus

class AbstractStep(object):
    """All steps in a task are derived from this class"""

    __metaclass__ = abc.ABCMeta
    RUNNING_GOAL_STATES = [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.RECALLING, GoalStatus.PREEMPTING]

    def __init__(self):
        self._status = GoalStatus.LOST

    def set_running(self, **kwargs):
        """
        Returns a status denoting that the step is still running
        """
        self._status = GoalStatus.ACTIVE
        return kwargs

    def set_succeeded(self, **kwargs):
        """
        Returns a status denoting that the step has succeeded
        """
        self._status = GoalStatus.SUCCEEDED
        return kwargs

    def set_aborted(self, **kwargs):
        """
        Returns a status denoting that the step has failed
        """
        self._status = GoalStatus.ABORTED
        return kwargs

    def set_preempted(self, **kwargs):
        """
        Returns a status denoting that the step was preempted
        """
        self._status = GoalStatus.PREEMPTED
        return kwargs

    @property
    def status(self):
        """Current status of this step"""
        return self._status

    def is_running(self):
        """
        Checks to see if the current step is running. Counterpoint to
        `AbstractStep.running()`
        """
        return self._status == GoalStatus.ACTIVE

    def is_succeeded(self):
        """
        Checks to see if the current step is running. Counterpoint to
        `AbstractStep.succeeded()`
        """
        return self._status == GoalStatus.SUCCEEDED

    def is_preempted(self):
        """
        Checks to see if the current step was preempted. Counterpoint to
        `AbstractStep.preempted()`
        """
        return self._status == GoalStatus.PREEMPTED

    def is_aborted(self):
        """
        Checks to see if the current step is running. Counterpoint to
        `AbstractStep.aborted()`
        """
        return not (self.is_running() or self.is_succeeded() or self.is_preempted())

    @abc.abstractmethod
    def init(self, name, *args, **kwargs):
        """
        Initialize the step.
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def run(self, **params):
        """
        Run the step. This method returns a generator. So don't return;
        instead yield an empty dictionary of return values to keep executing.
        Stop yielding values or raise StopIteration to stop the run.

        Use `running()`, `succeeded()`, `aborted()`, `preempted()` helper
        functions to set status when yielding
        """
        raise NotImplementedError()

    @abc.abstractmethod
    def stop(self):
        """
        Stop the step. How this is handled by the client is up to the client.
        """
        raise NotImplementedError()
