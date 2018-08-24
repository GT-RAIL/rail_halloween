#!/usr/bin/env python
# The pick action in a task plan

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import ExecuteGraspAction, ExecuteGraspGoal, \
                                       ExecuteGraspResult


class PickAction(AbstractAction):

    def init(self, **kwargs):
        self._grasp_client = actionlib.SimpleActionClient(
            "grasp_executor/execute_grasp",
            ExecuteGraspAction
        )

        rospy.loginfo("Connecting to grasp_executor...")
        self._grasp_client.wait_for_server()
        rospy.loginfo("...grasp_executor connected")

    def run(self, cube_idx, grasps):
        rospy.loginfo("Picking up object at index: {}".format(cube_idx))

        # Create the template goal
        goal = ExecuteGraspGoal()
        goal.index = cube_idx
        goal.grasp_pose.header.frame_id = grasps.header.frame_id

        # Iterate through all the poses, and report an error if all of them
        # failed
        status = GoalStatus.LOST
        for idx, grasp_pose in enumerate(grasps.poses):
            rospy.loginfo("Attempting grasp {}/{}".format(idx + 1, len(grasps.poses)))

            goal.grasp_pose.pose = grasp_pose
            goal.grasp_pose.header.stamp = rospy.Time.now()
            self._grasp_client.send_goal(goal)

            # Yield running while the client is executing
            while self._grasp_client.get_state() in AbstractAction.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status. Exit if we've succeeded
            status = self._grasp_client.get_state()
            result = self._grasp_client.get_result()
            if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
                break

            # Check to see the result. If we've failed, but we failed during the
            # lift phase, then break out an report an error
            rospy.sleep(0.2)  # Sleep to allow the result to fill up
            assert result is not None
            if result.failure_point in [ExecuteGraspResult.PICK_UP_PLAN, ExecuteGraspResult.PICK_UP_EXECUTION]:
                break

        # Finally, return based on status after all tries
        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted()
        else:
            yield self.set_aborted()

    def stop(self):
        self._grasp_client.cancel_goal()
