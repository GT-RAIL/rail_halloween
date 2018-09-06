#!/usr/bin/env
# The gripper action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from actionlib_msgs.msg import GoalStatus


class GripperAction(AbstractStep):

    def init(self, name):
        self.name = name
        self._gripper_client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action",
            GripperCommandAction
        )

        rospy.loginfo("Connecting to gripper_controller...")
        self._gripper_client.wait_for_server()
        rospy.loginfo("...gripper_controller connected")

    def run(self, command):
        if type(command) != str or command.lower() not in ['close', 'open']:
            rospy.logerr("Action: {}. FAIL. Unrecognized: {}".format(self.name, command))
            self.set_aborted(
                action=self.name,
                cause="Unrecognized",
                context=command
            )
            raise StopIteration()

        rospy.loginfo("Action {}: Gripper {}".format(self.name, command))

        # Create and send the goal pose
        goal = GripperCommandGoal()
        if command.lower() == 'close':
            goal.command.position = 0.0
            goal.command.max_effort = 200
        elif command.lower() == 'open':
            goal.command.position = 0.15

        self._gripper_client.send_goal(goal)

        # Yield an empty dict while we're executing
        while self._gripper_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Wait for a result and yield based on how we exited
        status = self._gripper_client.get_state()
        self._gripper_client.wait_for_result()
        result = self._gripper_client.get_result()

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=command,
                result=result
            )

    def stop(self):
        self._gripper_client.cancel_goal()
