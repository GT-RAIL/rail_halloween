#!/usr/bin/env python

import rospy
import actionlib
from bagposefromperson.msg import *

def bag_pose_from_person_client(detection_mode):
    # rospy.wait_for_service('get_bag_pose_from_person')
    # try:
    #     get_bag_pose_from_person = rospy.ServiceProxy('get_bag_pose_from_person', GetBagPoseAction)
    #     return get_bag_pose_from_person(detection_mode)
    # except rospy.ServiceException, e:
    #     print "Service call failed: %s"%e
    client = actionlib.SimpleActionClient("get_bag_pose_from_person", GetBagPoseAction)
    client.wait_for_server()
    goal = GetBagPoseGoal(detection_mode=detection_mode)
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result().bag_pose

if __name__ == "__main__":
    print "Requesting bag pose"
    rospy.init_node('bag_pose_client')
    while(True):
        print "bag pose = %s"%bag_pose_from_person_client(detection_mode='normal')
