#!/usr/bin/env python
# The pick action in a task plan

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import ExecuteGraspAction, ExecuteGraspGoal, \
                                       ExecuteGraspResult


class PickAction(AbstractStep):

    def init(self, name):
        self.name = name
        self._grasp_client = actionlib.SimpleActionClient(
            "grasp_executor/execute_grasp",
            ExecuteGraspAction
        )

        rospy.loginfo("Connecting to grasp_executor...")
        self._grasp_client.wait_for_server()
        rospy.loginfo("...grasp_executor connected")

    def run(self, cube_idx, grasps):
        rospy.loginfo("Action {}: Picking up object at index {}".format(self.name, cube_idx))

        # Create the template goal
        goal = ExecuteGraspGoal()
        goal.index = cube_idx
        goal.grasp_pose.header.frame_id = grasps.header.frame_id

        # Iterate through all the poses, and report an error if all of them
        # failed
        status = GoalStatus.LOST
        for grasp_num, grasp_pose in enumerate(grasps.poses):
            rospy.loginfo("Action {}: Attempting grasp {}/{}"
                          .format(self.name, grasp_num + 1, len(grasps.poses)))

            goal.grasp_pose.pose = grasp_pose
            goal.grasp_pose.header.stamp = rospy.Time.now()
            self._grasp_client.send_goal(goal)

            # Yield running while the client is executing
            while self._grasp_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
                yield self.set_running()

            # Check the status. Exit if we've succeeded
            status = self._grasp_client.get_state()
            if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
                break

        # Wait for a result and yield based on how we exited
        self._grasp_client.wait_for_result()
        result = self._grasp_client.get_result()

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=cube_idx,
                num_grasps=len(grasps.poses),
                grasp_num=grasp_num,
                grasps=grasps,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=cube_idx,
                num_grasps=len(grasps.poses),
                grasp_num=grasp_num,
                grasps=grasps,
                result=result
            )

    def stop(self):
        self._grasp_client.cancel_goal()
