#!/usr/bin/env python
# The abstract base class that defines an executable step in the task execution
# process

from __future__ import print_function

import abc
import pickle
import rospy

from actionlib_msgs.msg import GoalStatus
from task_executor.msg import ExecutionEvent

class AbstractStep(object):
    """All steps in a task are derived from this class"""

    EXECUTION_TRACE_TOPIC = '/execution_trace'

    __metaclass__ = abc.ABCMeta
    RUNNING_GOAL_STATES = [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.RECALLING, GoalStatus.PREEMPTING]

    def __init__(self):
        # Set the attributes that all steps have
        self.name = None
        self._status = GoalStatus.LOST
        self._trace = rospy.Publisher(AbstractStep.EXECUTION_TRACE_TOPIC, ExecutionEvent, queue_size=1)

    def _update_trace(self, context):
        event = ExecutionEvent(
            stamp=rospy.Time.now(),
            name=self.name,
            status=self.status,
            context=pickle.dumps(context)
        )
        self._trace.publish(event)

    def set_running(self, **context):
        """
        Returns a status denoting that the step is still running
        """
        self._status = GoalStatus.ACTIVE
        self._update_trace(context)
        return context

    def set_succeeded(self, **context):
        """
        Returns a status denoting that the step has succeeded
        """
        self._status = GoalStatus.SUCCEEDED
        self._update_trace(context)
        return context

    def set_aborted(self, **context):
        """
        Returns a status denoting that the step has failed
        """
        self._status = GoalStatus.ABORTED
        self._update_trace(context)
        return context

    def set_preempted(self, **context):
        """
        Returns a status denoting that the step was preempted
        """
        self._status = GoalStatus.PREEMPTED
        self._update_trace(context)
        return context

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

    def __call__(self, **params):
        """
        Calls the run generator internally and returns this node's status. This
        is a blocking call.

        Returns:
            status - actionlib_msgs/GoalStatus status_code
            variables - the last yielded dictionary from the run command
        """
        for variables in self.run(**params):
            if rospy.is_shutdown():
                break

        return (self.status, variables,)
