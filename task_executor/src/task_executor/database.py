#!/usr/bin/env python
# A service server that gives access to different data as needed

from __future__ import print_function, division

import rospy

from task_executor.msg import Waypoint, Bounds, ObjectConstraints, ArmPose
from std_srvs.srv import Trigger, TriggerResponse
from task_executor.srv import (GetWaypoints, GetWaypointsResponse,
                               GetObjectConstraints, GetObjectConstraintsResponse,
                               GetArmPose, GetArmPoseResponse,
                               GetTrajectory, GetTrajectoryResponse)


# The actual database node

class DatabaseServer(object):
    """
    Based on the ROS params that are loaded from a YAML file, provide a set of
    services that other nodes can use to query waypoints, arm trajectories, etc.
    by name
    """

    def __init__(self):
        # Provide a service to reload, then reload
        self._reload_service = rospy.Service('~reload', Trigger, self.reload)
        self.reload(None)

        # Start up the service servers for the different query types
        self._waypoints_service = rospy.Service(
            '~waypoints', GetWaypoints, self.get_waypoints
        )
        self._object_constraints_service = rospy.Service(
            '~object_constraints', GetObjectConstraints, self.get_object_constraints
        )
        self._arm_pose_service = rospy.Service(
            '~arm_pose', GetArmPose, self.get_arm_pose
        )
        self._trajectory_service = rospy.Service(
            '~trajectory', GetTrajectory, self.get_trajectory
        )

    def start(self):
        # This is a no-op at the moment
        rospy.loginfo("Database node ready...")

    def reload(self, req):
        # Validate the data in each of the expected rosparams and populate the
        # database
        self.waypoints = self._validate_waypoints(rospy.get_param('~waypoints'))
        self.object_constraints = self._validate_object_constraints(rospy.get_param('~object_constraints'))
        self.arm_poses = self._validate_arm_poses(rospy.get_param('~arm_poses'))
        self.trajectories = self._validate_trajectories(rospy.get_param('~trajectories'))

    def get_waypoints(self, req):
        resp = GetWaypointsResponse(waypoints=self.waypoints[req.name])
        return resp

    def get_object_constraints(self, req):
        resp = GetObjectConstraintsResponse(constraints=self.object_constraints[req.name])
        return resp

    def get_arm_pose(self, req):
        resp = GetArmPoseResponse(pose=self.arm_poses[req.name])
        return resp

    def get_trajectory(self, req):
        resp = GetTrajectoryResponse(trajectory=self.trajectories[req.name])
        return resp

    def _validate_waypoints(self, wp_defs):
        # Reload the waypoints
        waypoints = {}
        for name, wp_def in wp_defs.iteritems():
            waypoints[name] = [Waypoint(**x) for x in wp_def]

        return waypoints

    def _validate_object_constraints(self, oc_defs):
        # Reload the object constraints
        object_constraints = {}
        for name, oc_def in oc_defs.iteritems():
            object_constraints[name] = ObjectConstraints(
                use_bounds=oc_def.get('bounds') is not None,
                use_location=oc_def.get('location') is not None
            )

            if oc_def.get('bounds') is not None:
                object_constraints[name].bounds = Bounds(**oc_def['bounds'])

            if oc_def.get('location') is not None:
                object_constraints[name].location = Bounds(**oc_def['location'])

        return object_constraints

    def _validate_arm_poses(self, ap_defs):
        # Reload the arm poses
        arm_poses = {}
        for name, ap_def in ap_defs.iteritems():
            arm_poses[name] = ArmPose(angles=ap_def)

        return arm_poses

    def _validate_trajectories(self, traj_defs):
        # Reload the trajectories
        trajectories = {}
        for name, traj_def in traj_defs.iteritems():
            trajectories[name] = [ArmPose(angles=x) for x in traj_def]

        return trajectories
