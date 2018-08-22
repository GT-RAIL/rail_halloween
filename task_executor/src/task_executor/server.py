#!/usr/bin/env
# An action server to execute a task plan.

from __future__ import print_function, division

import copy

import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from task_executor.msg import ExecuteAction
from std_srvs.srv import Trigger, TriggerResponse
from actionlib_msgs.msg import GoalStatus

from task_executor.actions import actions, AbstractAction
from task_executor import ops


# The actual action server to execute the tasks

class TaskServer(object):
    """
    Given the task to perform, this server yields control to sub clients. When
    the clients are done, it moves on to the next task. If the task fails, the
    server sends context to an arbitrator. Based on decisions from the
    arbitrator, the server then yields control to a recovery interface
    """

    def __init__(self):
        # Provide a service to reload, and then reload
        self._reload_service = rospy.Service('~reload', Trigger, self.reload)
        self.reload(None)

        # Instantiate the action server
        self._server = actionlib.SimpleActionServer(
            rospy.get_name(),
            ExecuteAction,
            self.execute,
            auto_start=False
        )

    def start(self):
        self._server.start()

    def reload(self, req):
        # Instantiate the DB of locations and objects
        self.locations = self._validate_locations(rospy.get_param('~locations'))
        self.objects = self._validate_objects(rospy.get_param('~objects'))
        self.task_plan = rospy.get_param('~task')

        # Instantiate the registry of actions
        actions.init(self.locations, self.objects)

        return TriggerResponse(success=True)

    def execute(self, goal):
        """
        Execute the given goal. Has a spec of ExecuteGoal.
        """
        result = self._server.get_default_result()
        rospy.loginfo("Executing goal: {}".format(goal.name))

        # Instantiate the dictionary of local variables
        var = dict()

        # Go through each step of the specified plan
        idx = 0
        while idx < len(self.task_plan):
            step = self.task_plan[idx]

            action = actions[step['action']]
            params = {
                name: self._resolve_param(value, var)
                for name, value in step.get('params', {}).iteritems()
            }

            print(step, params)
            for variables in action.run(**params):
                # First check to see if we've been preempted. If we have, then
                # set the preempt flag and wait for the action to return
                if self._server.is_preempt_requested() \
                        or not self._server.is_active():
                    action.stop()
                    continue

                # Check to see if something has stopped the action. If so, then
                # exit out of this for loop
                if action.is_preempted() or action.is_aborted():
                    break

            # If we've failed for some reason. Return
            if action.is_preempted():
                rospy.logwarn(
                    "Step {}, Action {}: PREEMPTED. Context: {}"
                    .format(idx, step['action'], variables)
                )
                self._server.set_preempted(result)
                return

            if action.is_aborted():
                rospy.logerr(
                    "Step {}, Action {}: FAIL. Context: {}"
                    .format(idx, step['action'], variables)
                )
                self._server.set_aborted(result)
                return

            # Verify the variables
            if not self._validate_variables(step.get('var', []), variables):
                rospy.logerr(
                    "Step {}, Action {}: Variable validation failed. Variables: {}"
                    .format(idx, step['action'], variables)
                )
                self._server.set_aborted(result)
                return

            # Update the variables that we're keeping track of
            for name, value in variables.iteritems():
                var[name] = value

            idx += 1

        # Otherwise, signal complete
        result.success = True
        self._server.set_succeeded(result)

    def _validate_locations(self, locations):
        # TODO: We don't need to validate yet. But perhaps soon
        return locations

    def _validate_objects(self, objects):
        # TODO: We don't need to validate yet. But perhaps soon
        return objects

    def _validate_variables(self, expected_var, actual_var):
        if sorted(actual_var.keys()) == sorted(expected_var):
            return True
        else:
            return False

    def _resolve_var(self, param, var):
        # Check type
        if type(param) == str:
            splits = param.split('.', 1)  # Split up the param
            if len(splits) > 1 and splits[0] == 'var':
                return var[splits[1]]
        return param

    def _resolve_op(self, param, var):
        op_name, args = param.split('(', 1)
        args = args.strip(')').split(',')
        args = [self._resolve_var(arg, var) for arg in args]

        # Finally, get the op and provide it with the args
        return getattr(ops, op_name)(*args)

    def _resolve_param(self, param, var):
        if type(param) == str:
            splits = param.split('.', 1)  # Split up the param

            # Check if this requires an ops resolution. TODO: Use regex?
            if len(splits) > 1 and splits[0] == 'ops' and '(' in splits[1] \
                    and ')' in splits[1]:
                param = self._resolve_op(splits[1], var)

        return self._resolve_var(param, var)
