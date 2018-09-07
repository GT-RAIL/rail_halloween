#!/usr/bin/env python
# The arm pose action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from fetch_grasp_suggestion.msg import PresetJointsMoveAction, PresetJointsMoveGoal
from task_executor.msg import ArmPose
from task_executor.srv import GetArmPose, GetTrajectory


class ArmPoseAction(AbstractStep):

    def init(self, name):
        self.name = name

        self._pose_client = actionlib.SimpleActionClient(
            "grasp_executor/preset_position",
            PresetJointsMoveAction
        )
        self._get_arm_pose_srv = rospy.ServiceProxy("database/arm_pose", GetArmPose)
        self._get_trajectory_srv = rospy.ServiceProxy("database/trajectory", GetTrajectory)

        self._max_attempts = 1

        rospy.loginfo("Connecting to arm_pose_executor...")
        self._pose_client.wait_for_server()
        rospy.loginfo("...arm_pose_executor connected")

        rospy.loginfo("Connecting to database services...")
        self._get_arm_pose_srv.wait_for_service()
        self._get_trajectory_srv.wait_for_service()
        rospy.loginfo("...database services connected")

    def run(self, poses):
        # Parse out the pose waypoints
        pose_waypoints = self._parse_poses(poses)
        if pose_waypoints is None:
            rospy.logerr("Action {}: FAIL. Unknown format: {}".format(self.name, poses))
            yield self.set_aborted(
                action=self.name,
                cause="Unknown format",
                context=poses
            )
            raise StopIteration()

        rospy.logdebug("Action {}: Moving to arm pose(s): {}".format(self.name, pose_waypoints))

        status = GoalStatus.LOST
        attempt_num = -1
        for pose in pose_waypoints:
            rospy.loginfo("Action {}: Going to arm pose: {}".format(self.name, pose.angles))

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
            goal.position = pose.angles
            assert len(goal.name) == len(goal.position)

            for attempt_num in xrange(self._max_attempts):
                rospy.loginfo("Action {}: Attempt {}/{}".format(self.name, attempt_num + 1, self._max_attempts))
                self._pose_client.send_goal(goal)

                # Yield running while the client is executing
                while self._pose_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
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

        # Wait for a result and yield based on how we exited
        self._pose_client.wait_for_result()
        result = self._pose_client.get_result()

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded()
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                orig_goal=poses,
                goal=pose_waypoints,
                attempt_num=attempt_num,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                orig_goal=poses,
                goal=pose_waypoints,
                attempt_num=attempt_num,
                result=result
            )

    def stop(self):
        self._pose_client.cancel_goal()

    def _parse_poses(self, poses):
        pose_waypoints = None

        if type(poses) == str:
            # This is a reference to stored poses in the DB
            db_name, poses = poses.split('.', 1)
            if db_name == 'poses':
                pose_waypoints = [self._get_arm_pose_srv(poses).pose,]
            elif db_name == 'trajectories':
                pose_waypoints = self._get_trajectory_srv(poses).trajectory
        elif (type(poses) == list or type(poses) == tuple) \
                and len(poses) > 0 \
                and (type(poses[0]) == list or type(poses[0]) == tuple):
            # YAML definition of a trajectory
            pose_waypoints = [ArmPose(angles=x) for x in poses]
        elif (type(poses) == list or type(poses) == tuple) and len(poses) > 0:
            # YAML definition of a pose
            pose_waypoints = [ArmPose(angles=poses),]

        return pose_waypoints
