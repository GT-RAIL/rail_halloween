#!/usr/bin/env python
# The find object action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_action import AbstractAction

from rail_manipulation_msgs.msg import SegmentedObjectList
from rail_manipulation_msgs.srv import SegmentObjects


class FindObjectAction(AbstractAction):

    def init(self, objects, **kwargs):
        # Objects DB
        self._objects = objects

        # The segmentation interface
        self._segment_objects_srv = rospy.ServiceProxy(
            "rail_segmentation/segment_objects",
            SegmentObjects
        )

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to rail_segmentation
        rospy.loginfo("Connecting to rail_segmentation...")
        self._segment_objects_srv.wait_for_service()
        rospy.loginfo("...rail_segmentation connected")

    def run(self, obj):
        # Fetch the object from the database and determing the bounds and
        # location within which the object will be
        obj = obj.split('.', 1)[1]
        rospy.loginfo("Inspecting scene for object: {}".format(obj))
        self._stopped = False

        # Ask for a segmentation and then identify the object that we want
        try:
            segmented_objects = self._segment_objects_srv().segmented_objects
            found_idx, found_obj = self._find_obj(obj, segmented_objects)
            yield self.set_running()  # Check on the status of the server

            if self._stopped:
                yield self.set_preempted()
            elif found_idx > -1:
                yield self.set_succeeded(found_obj=found_obj, found_idx=found_idx)
            else:
                raise IndexError("{} not found among {}".format(obj, segmented_objects))
        except Exception as e:  # On any exception, make sure that we are set to aborted
            yield self.set_aborted(exception=e)

    def stop(self):
        self._stopped = True

    def _find_obj(self, obj, segmented_objects):
        """
        Find the object with key `obj` in the `segmented_objects` based on the
        details of that object in `self._objects`. Our find is rudimentary;
        based solely on the expected location and bounds of the object
        """
        bounds = self._objects[obj].get('bounds')
        location = self._objects[obj].get('location')

        found_idx, found_obj = -1, None
        for idx, segmented_object in enumerate(segmented_objects.objects):
            # We assume that the segmentation config is going to take care of
            # putting the point cloud in the appropriate frame for us

            # Check to see if the point cloud is in approximately the expected
            # location
            if location is not None and (
                    segmented_object.center.x < location['xmin'] or
                    segmented_object.center.x > location['xmax'] or
                    segmented_object.center.y < location['ymin'] or
                    segmented_object.center.y > location['ymax'] or
                    segmented_object.center.z < location['zmin'] or
                    segmented_object.center.z > location['zmax']):
                continue

            # Check to see if the point cloud has approximately the expected
            # dimensions
            if bounds is not None and (
                    segmented_object.bounding_volume.dimensions.x < bounds['xmin'] or
                    segmented_object.bounding_volume.dimensions.x > bounds['xmax'] or
                    segmented_object.bounding_volume.dimensions.y < bounds['ymin'] or
                    segmented_object.bounding_volume.dimensions.y > bounds['ymax'] or
                    segmented_object.bounding_volume.dimensions.z < bounds['zmin'] or
                    segmented_object.bounding_volume.dimensions.z > bounds['zmax']):
                continue

            # I think we've found the object!
            found_idx, found_obj = idx, segmented_object
            break

        return found_idx, found_obj
