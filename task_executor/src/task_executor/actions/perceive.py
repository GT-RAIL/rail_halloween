#!/usr/bin/env
# The torso action in a task plan

from __future__ import print_function, division

from threading import Lock

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from actionlib_msgs.msg import GoalStatus
from rail_manipulation_msgs.msg import SegmentedObjectList
from fetch_grasp_su
from rail_manipulation_msgs.srv import SegmentObjects


class PerceiveAction(AbstractAction):

    def __init__(self):
        # Objects DB
        self._objects = None

        # The segmentation interface
        self._segment_objects_srv = rospy.ServiceProxy(
            "/rail_segmentation/segment_objects",
            SegmentObjects
        )

    def _find_obj(self, obj, segmented_objects):
        """
        Find the object with key `obj` in the `segmented_objects` based on the
        details of that object in `self._objects`
        """
        bounds = self._objects[obj]['bounds']
        location = self._objects[obj]['location']

        found_idx, found_obj = -1, None
        for idx, segmented_object in enumerate(segmented_objects.objects):
            # We assume that the segmentation config is going to take care of
            # putting the point cloud in the appropriate frame for us

            # Check to see if the point cloud is in approximately the expected
            # location
            if segmented_object.center.x < location['xmin'] or \
                    segmented_object.center.x > location['xmax'] or \
                    segmented_object.center.y < location['ymin'] or \
                    segmented_object.center.y > location['ymax'] or \
                    segmented_object.center.z < location['zmin'] or \
                    segmented_object.center.z > location['zmax']:
                continue

            # Check to see if the point cloud has approximately the expected
            # dimensions
            if segmented_object.bounding_volume.dimensions.x < bounds['xmin'] or \
                    segmented_object.bounding_volume.dimensions.x > bounds['xmax'] or \
                    segmented_object.bounding_volume.dimensions.y < bounds['ymin'] or \
                    segmented_object.bounding_volume.dimensions.y > bounds['ymax'] or \
                    segmented_object.bounding_volume.dimensions.z < bounds['zmin'] or \
                    segmented_object.bounding_volume.dimensions.z > bounds['zmax']:
                continue

            # I think we've found the object!
            found_idx, found_obj = idx, segmented_object
            break

        return found_idx, found_obj

    def init(self, locations, objects):
        self._objects = objects

        # Wait for a connection to rail_segmentation
        rospy.loginfo("Connecting to rail_segmentation...")
        self._segment_objects_srv.wait_for_service()
        rospy.loginfo("...rail_segmentation connected")

    def run(self, obj):
        # Fetch the object from the database and determing the bounds and
        # location within which the object will be
        obj = obj.split('.', 1)[1]
        rospy.loginfo("Inspecting scene for object: {}".format(obj))

        # Ask for a segmentation and then identify the object that we want
        segmented_objects = self._segment_objects_srv().segmented_objects
        found_idx, found_obj = self._find_obj(obj, segmented_objects)
        if found_idx == -1:
            yield self.aborted(found_objects=segmented_objects)
            raise StopIteration()

        # Given the segmentation and the objects, now ask for grasps
        pass

    def stop(self):
        # Can this even be stopped?
        pass
