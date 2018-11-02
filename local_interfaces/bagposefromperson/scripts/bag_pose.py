#!/usr/bin/env python
##
## Assumes rail_pose_detector is running

import numpy as np

from math import isnan
from struct import unpack_from
from threading import Lock

import rospy
import actionlib

from bagposefromperson.msg import *
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from rail_pose_estimation_msgs.msg import Poses
from sensor_msgs.msg import PointCloud2, Image


# Global definition of a nanpose for soft error situations
NANPOSE = PoseStamped()
NANPOSE.pose.position = Point(*([float('nan')] * 3))


class BagPoseFromPerson():

    def __init__(self):
        self._server = None

        self.bag_pose = NANPOSE
        self.cur_person_pose = None
        self.cur_points = None
        self.min_depth_pixel = (float('nan'),float('nan'))

        self.point_cloud = None # Stored point cloud
        self.pc_point_size = None
        self.pc_row_size = None
        self.pc_width = None
        self.pc_height = None
        self.pc_fields = None
        self.point_cloud_lock = Lock()

    def _pose_callback(self, data):
        # TODO(balloch): make this a moving average
        if data.people:
            # print "poses_data=%s"%data.people[0].right_elbow_x
            self.cur_person_pose = data.people[0] #.data

    def _points_callback(self, point_cloud_msg):
        # TODO(balloch): make this a moving average
        # print "Points_data=%s"%len(point_cloud_msg.data)
        if self.point_cloud_lock.acquire(False):
            self.point_cloud = point_cloud_msg
            self.pc_point_size = point_cloud_msg.point_step
            self.pc_row_size = point_cloud_msg.row_step
            self.pc_width = point_cloud_msg.width
            self.pc_height = point_cloud_msg.height
            self.pc_fields = point_cloud_msg.fields
            self.point_cloud_lock.release()

    def _depth_callback(self, depth_msg):
        np_depth = np.asarray(depth_msg)
        self.min_depth_pixel = np.unravel_index(np_depth.argmin(), np_depth.shape)

    def _handle_bag_pose_from_person(self, goal):
        try:
            result = GetBagPoseResult(bag_pose=self._calc_bag_pose(goal.detection_mode))
            if not self._server.is_preempt_requested():
                self._server.set_succeeded(result)
            else:
                self._server.set_preempted(result)
        except Exception as e:
            rospy.logerr("BagPose: {}".format(e))
            result = GetBagPoseResult(bag_pose=NANPOSE)
            self._server.set_aborted(result)

    def _pixel_to_3D(self, pixel_point):
        """
        Gets 3D point in the optical frame from PointCloud2 points and pixel values
        :param pixel_point: Tuple of the pixel we need a 3D position for
        :return point: a PoseStamped of the corresponding pixel. None if fail
        """

        with self.point_cloud_lock:
            if self.point_cloud is None:
                return NANPOSE

            header = self.point_cloud.header # Use this header later

            # Check that the point is not at the boundaries
            if (pixel_point[0] >= self.pc_width-1
                or pixel_point[0] < 0
                or pixel_point[1] >= self.pc_height-1
                or pixel_point[1] < 0):
                return NANPOSE #pc_point = None

            # The position of the point is actually the mean of nearby points
            neighbours = [(pixel_point[0], pixel_point[1]),
                          (pixel_point[0], pixel_point[1]+1),
                          (pixel_point[0]+1, pixel_point[1]),
                          (pixel_point[0]+1, pixel_point[1]+1)]

            # Only use those points for which we do have positions
            is_valid = False
            neighbour_pos = np.empty((len(neighbours), 3))
            for counter, neighbour in enumerate(neighbours):
                x_idx = (self.pc_row_size * neighbour[1]
                         + neighbour[0] * self.pc_point_size)
                y_idx = (self.pc_row_size * neighbour[1]
                         + neighbour[0] * self.pc_point_size + 4)
                z_idx = (self.pc_row_size * neighbour[1]
                         + neighbour[0] * self.pc_point_size + 8)
                float_x = unpack_from('f', self.point_cloud.data, x_idx)
                float_y = unpack_from('f', self.point_cloud.data, y_idx)
                float_z = unpack_from('f', self.point_cloud.data, z_idx)

                if isnan(float_x[0]) or isnan(float_y[0]) or isnan(float_z[0]):
                    continue
                is_valid = True
                neighbour_pos[counter, :] = (float_x[0], float_y[0], float_z[0])

            # Don't consider this point if none of the nearby points have a
            # position in the point cloud
            if not is_valid:
                return NANPOSE # pc_point = None
            else:
                pc_point = np.mean(neighbour_pos, axis=0) #[0:num_valid,:], axis=0)

        if pc_point is None: # This sequence is deprecated with current version
            point = NANPOSE
        else:
            point = PoseStamped(header=header)
            point.pose.position = Point(*pc_point)
            point.pose.orientation.w = 1.0
        return point

    def _calc_bag_pose(self, detection_mode):
        #need lock here
        if detection_mode is GetBagPoseGoal.D_MODE_ALT:
            if isnan(self.min_depth_pixel[0]) or isnan(self.min_depth_pixel[1]):
                return NANPOSE
            else:
                self.bag_pose = self._pixel_to_3D(self.min_depth_pixel)
        else:  # detection_mode == D_MODE_STD
            if self.cur_person_pose:
                center = [int((self.cur_person_pose.left_elbow_x +
                               self.cur_person_pose.right_elbow_x)/2),
                          int((self.cur_person_pose.left_elbow_y +
                               self.cur_person_pose.right_elbow_y)/2)]
                print center
                self.bag_pose = self._pixel_to_3D(center)
                # if self.bag_pose.point.z > 1.5:
                #     return PointStamped(header=header, point=Point(*([float(-1)] * 3)))
                # else:
            else:
                return NANPOSE

        return self.bag_pose

    def run(self):
        depth_topic = '/head_camera/depth_registered/image'
        rospy.Subscriber(depth_topic, Image, self._depth_callback)
        poses_topic = '/rail_pose_estimator_node/poses'
        rospy.Subscriber(poses_topic, Poses, self._pose_callback)
        points_topic = '/head_camera/depth_registered/points'
        rospy.Subscriber(points_topic, PointCloud2, self._points_callback)

        self._server = actionlib.SimpleActionServer(
            'get_bag_pose_from_person',
            GetBagPoseAction,
            self._handle_bag_pose_from_person,
            auto_start=False
        )
        self._server.start()
        print "Ready to calculate bag pose"
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('bag_pose_from_person_server')
    bag_pose_server = BagPoseFromPerson()
    bag_pose_server.run()
