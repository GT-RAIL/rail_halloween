#!/usr/bin/env python
# The operations that can be specified in the YAML

from __future__ import print_function, division

import rospy

from geometry_msgs.msg import PoseStamped


# The Operations

def pose_of_cube(obj):
    """
    Given an object returned by the perceive operation, return its pose.

    Args:
        obj (grasping_msgs/Object) : The object to get the position of
    Returns:
        geometry_msgs/PoseStamped with the pose of that object
    """
    pose = PoseStamped()
    pose.pose = obj.primitive_poses[0]
    pose.header.frame_id = obj.header.frame_id
    return pose
