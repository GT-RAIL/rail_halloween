#!/usr/bin/env python
# The arm pose action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import PresetJointsMoveAction, PresetJointsMoveGoal


class ArmPoseAction(AbstractAction):

    def init(self, poses, trajectories, **kwargs):
        self._pose_client = actionlib.SimpleActionClient(
            "grasp_executor/preset_position",
            PresetJointsMoveAction
        )
        self._poses = poses
        self._trajectories = trajectories

        self._max_attempts = 5

        rospy.loginfo("Connecting to arm_pose_executor...")
        self._pose_client.wait_for_server()
        rospy.loginfo("...arm_pose_executor connected")

    def run(self, poses):
        # Parse out the pose. This will error if the pose format is incorrect
        db_name, poses = poses.split('.', 1)
        rospy.loginfo("Moving to poses: {}".format(poses))

        if db_name == 'poses':
            poses = [self._poses[poses]]
        elif db_name == 'trajectories':
            poses = [pose for pose in self._trajectories[poses]]

        status = GoalStatus.LOST
        for pose in poses:
            rospy.loginfo("Going to pose: {}".format(pose))

            # Create and send the goal
            goal = PresetJointsMoveGoal()
            goal.name.extend([
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "upperarm_roll_joint",
                "elbow_flex_joint",
                "forearm_roll_joint",
                "wrist_flex_joint",
                "wrist_roll_joint",
            ])
            goal.position.extend(pose)
            assert len(goal.name) == len(goal.position)

            for attempt in xrange(self._max_attempts):
                rospy.loginfo("Attempt {}/{}".format(attempt + 1, self._max_attempts))
                self._pose_client.send_goal(goal)

                # Yield running while the client is executing
                while self._pose_client.get_state() in AbstractAction.RUNNING_GOAL_STATES:
                    yield self.set_running()

                # Yield based on the server's status
                status = self._pose_client.get_state()

                # Exit if we have succeeded or been preempted
                if status == GoalStatus.SUCCEEDED or status == GoalStatus.PREEMPTED:
                    break

            # If we haven't succeeded in reaching this intermediate pose, then
            # break. Otherwise, move on to the next pose
            if status != GoalStatus.SUCCEEDED:
                break

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted()
        else:
            yield self.set_aborted()

    def stop(self):
        self._pose_client.cancel_goal()
