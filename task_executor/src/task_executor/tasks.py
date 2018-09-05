#!/usr/bin/env python
# The task executor executes a task that is defined in a YAML config file

from __future__ import print_function, division

import rospy

from task_executor.abstract_step import AbstractStep
from task_executor import ops


# The actual executor of tasks

class Task(AbstractStep):
    """
    Given a task to perform, the executor follows the operations defined in the
    YAML file until it is asked to stop or until the actions it is executing
    indicate that they have finished, either successfully or not.
    """

    def init(self, name, tasks, actions, steps, params=[], var=[], **kwargs):
        # Knowledge of all the tasks and actions
        self.name = name
        self.tasks = tasks
        self.actions = actions

        # Specific to this task
        self.params = params
        self.var = var
        self.steps = steps

        # Flag to indicate if the task is stopped
        self._stopped = False

    def run(self, **params):
        rospy.loginfo("Task {}: EXECUTING.".format(self.name))

        # First validate the params that we might have received
        if not self._validate_params(self.params, params):
            error_msg = (
                "Task {}: FAIL. Unexpected Params. Expected: {}. Received: {}."
                .format(self.name, self.params, params)
            )
            rospy.logerr(error_msg)
            yield self.set_aborted(exception=Exception(error_msg))
            raise StopIteration()

        # Setup to run the task
        var = dict()
        self._stopped = False

        # Go through each step of the specified task plan as appropriate
        idx = 0
        while idx < len(self.steps):
            step = self.steps[idx]
            step_name = step.get('task', step.get('action', step.get('op')))

            # First resolve any and all params for this step
            step_params = {
                name: self._resolve_param(value, var, params)
                for name, value in step.get('params', {}).iteritems()
            }
            variables = {}

            # Check to see if this is an op. If so, run the op
            if step.has_key('op'):
                variables = getattr(ops, step['op'])(**step_params)

            # Otherwise, execute the action/task:
            else:
                executor = self.actions[step['action']] if step.has_key('action') else self.tasks[step['task']]

                # Run and stop/yield as necessary
                for variables in executor.run(**step_params):
                    # First check to see if we've been preempted. If we have,
                    # set the preempt flag and wait for the action to return
                    if self._stopped:
                        executor.stop()
                        continue

                    # Check to see if the action/task has been preempted. If so,
                    # exit out of this loop
                    if executor.is_preempted() or executor.is_aborted():
                        break

                    # Otherwise, yield a running
                    yield self.set_running(**variables)

                # If the reason we stopped is a failure, then return
                if executor.is_preempted():
                    error_msg = (
                        "Task {}, Step {}({}): PREEMPTED. Context: {}"
                        .format(self.name, idx, step_name, variables)
                    )
                    rospy.logwarn(error_msg)
                    yield self.set_preempted(exception=Exception(error_msg))
                    raise StopIteration()

                if executor.is_aborted():
                    error_msg = (
                        "Task {}, Step {}({}): FAIL. Context: {}"
                        .format(self.name, idx, step_name, variables)
                    )
                    rospy.logerr(error_msg)
                    yield self.set_aborted(exception=Exception(error_msg))
                    raise StopIteration()

            # Validate the variables
            if not self._validate_variables(step.get('var', []), variables):
                error_msg = (
                    "Task {}, Step {}({}): FAIL. Variables Invalid. Expected: {}. Received: {}."
                    .format(self.name, idx, step_name, step.get('var', []), variables.keys())
                )
                rospy.logerr(error_msg)
                yield self.set_aborted(exception=Exception(error_msg))
                raise StopIteration()

            # Update the variables that we're keeping track of
            for name, value in variables.iteritems():
                var[name] = value

            idx += 1

        # Finally, yield succeeded with the variables that should be local stack
        # of variables that we're keeping track of
        rospy.loginfo("Task {}: SUCCESS.".format(self.name))
        yield self.set_succeeded(**{
            var_name: var[var_name] for var_name in self.var
        })

    def stop(self):
        self._stopped = True

    def _validate_params(self, expected_params, actual_params):
        return sorted(actual_params.keys()) == sorted(expected_params)

    def _validate_variables(self, expected_var, actual_var):
        return sorted(actual_var.keys()) == sorted(expected_var)

    def _resolve_param(self, param, var, task_params):
        if type(param) == str:
            splits = param.split('.', 1)  # Split up the param

            # Check if this requires a var resolution
            if len(splits) > 1 and splits[0] == 'var':
                return var[splits[1]]

            # Check if this requires a task_param resolution
            if len(splits) > 1 and splits[0] == 'params':
                return task_params[splits[1]]

        # Otherwise, this param should be used as is
        return param
