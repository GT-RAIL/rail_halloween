#!/usr/bin/env
# An action server to execute a task plan.

from __future__ import print_function, division

import copy

import rospy
import actionlib

from task_executor.actions import default_actions
from task_executor.tasks import Task

from actionlib_msgs.msg import GoalID
from task_executor.msg import ExecuteAction
from std_srvs.srv import Trigger, TriggerResponse


# The actual action server to execute the tasks

class TaskServer(object):
    """
    Given the task to perform, this server yields control to sub clients. When
    the clients are done, it moves on to the next task. If the task fails, the
    server sends context to an arbitrator. The arbitrator yields control to a
    recovery interface, which then returns a signal back up the stack to this
    server with hints on how execution should proceed.
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
        self.poses = self._validate_poses(rospy.get_param('~poses'))
        self.trajectories = self._validate_trajectories(rospy.get_param('~trajectories'))

        tasks_config = self._validate_tasks(rospy.get_param('~tasks'))
        self.tasks = { key: Task() for key, _ in tasks_config.iteritems() }

        # Instantiate the registry of actions
        default_actions.init(
            locations=self.locations,
            objects=self.objects,
            poses=self.poses,
            trajectories=self.trajectories
        )

        # Instantiate the registry of tasks
        for key, task in self.tasks.iteritems():
            task.init(
                name=key,
                tasks=self.tasks,
                actions=default_actions,
                **tasks_config[key]
            )

        return TriggerResponse(success=True)

    def execute(self, goal):
        """
        Execute the given goal. Has a spec of ExecuteGoal.
        """
        result = self._server.get_default_result()
        if goal.name not in self.tasks:
            rospy.logerr("Unrecognized task: {}".format(goal.name))
            self._server.set_aborted(result)
            return

        rospy.loginfo("Executing task: {}".format(goal.name))

        # Execute the task. TODO: Include params in the execution request of a
        # task?
        task = self.tasks[goal.name]
        for variables in task.run():
            # First check to see if we've been preempted. If we have, then
            # set the preempt flag and wait for the task to return
            if self._server.is_preempt_requested() or not self._server.is_active():
                task.stop()
                continue

            # Check to see if something has stopped the task. If so, then
            # exit out of this for loop
            if task.is_preempted() or task.is_aborted():
                break

        # If we've failed for some reason. Return an error
        if task.is_preempted():
            rospy.logwarn("Task {}: PREEMPTED. Context: {}".format(task.name, variables))
            self._server.set_preempted(result)
            return

        if task.is_aborted():
            rospy.logerr("Task {}: FAIL. Context: {}".format(task.name, variables))
            self._server.set_aborted(result)
            return

        # TODO: Perhaps task should be allowed to return values. In which case,
        # there needs to be another round of variable validation here.

        # Otherwise, signal complete
        result.success = True
        rospy.loginfo("Task {}: SUCCESS".format(task.name))
        self._server.set_succeeded(result)

    def stop(self):
        # Cancel all current goals
        cancel_pub = rospy.Publisher(rospy.get_name() + '/cancel', GoalID)
        cancel_pub.publish(GoalID(stamp=rospy.Time.now()))

        # Wait a bit
        rospy.sleep(0.5)

    def _validate_locations(self, locations):
        # We don't need to validate yet. But perhaps soon
        return locations

    def _validate_objects(self, objects):
        # We don't need to validate yet. But perhaps soon
        return objects

    def _validate_poses(self, poses):
        # We don't need to validate yet. But perhaps soon
        return poses

    def _validate_trajectories(self, trajectories):
        # We don't need to validate yet. But perhaps soon
        return trajectories

    def _validate_tasks(self, tasks):
        # We don't need to validate yet. But perhaps soon
        return tasks
