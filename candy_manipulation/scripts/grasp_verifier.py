#!/usr/bin/env python
# Created by Andrew Silva on 10/26/18
import numpy as np

import rospy
import cv2

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from fetch_driver_msgs.msg import GripperState
from std_srvs.srv import Trigger, TriggerResponse


class GraspVerifier(object):
    """
    Runs the grasp verification tools for ghouls from schools of fools following rules
    """

    VERIFY_SERVICE = '~verify'

    IMAGE_TOPIC = '/head_camera/depth/image_rect'
    GRIPPER_STATE_TOPIC = '/gripper_state'

    DEFAULT_SUCCEED_SERVICE = '~default_success'
    DEFAULT_FAIL_SERVICE = '~default_fail'
    SIMULATION_PARAMETER = '/use_sim_time'

    def __init__(self):
        self.bridge = CvBridge()
        self.last_image = np.zeros((640, 480))

        # We don't actually want to verify in sim
        self._in_simulation = rospy.get_param(GraspVerifier.SIMULATION_PARAMETER, False)

        # The variables to threshold detection on
        self.img_sum = 0.0
        self.gripper_pos = 0.05

        # Create the subscribers
        self._depth_sub = rospy.Subscriber(GraspVerifier.IMAGE_TOPIC, Image, self._parse_image)
        self._gripper_sub = rospy.Subscriber(GraspVerifier.GRIPPER_STATE_TOPIC, GripperState, self._parse_gripper_state)

        # Should we succeed or fail by default?
        self._default_succeed = False
        self._default_succeed_service = rospy.Service(
            GraspVerifier.DEFAULT_SUCCEED_SERVICE,
            Trigger,
            self._set_default_succeed
        )
        self._default_fail_service = rospy.Service(
            GraspVerifier.DEFAULT_FAIL_SERVICE,
            Trigger,
            self._set_default_fail
        )

        # Create the services
        self._verify_service = rospy.Service(GraspVerifier.VERIFY_SERVICE, Trigger, self._verify_grasp)

    def _set_default_succeed(self, req):
        self._default_succeed = True
        return TriggerResponse(success=True)

    def _set_default_fail(self, req):
        self._default_succeed = False
        return TriggerResponse(success=True)

    def _verify_grasp(self, req):
        success = self._default_succeed
        msg = str(self.img_sum)
        if not self._in_simulation:
            if self.img_sum < 11000 or self.gripper_pos >= 0.0078:
                success = True
            else:
                success = False

        return TriggerResponse(success=success, message=msg)

    def _parse_gripper_state(self, gripper_msg):
        try:
            self.gripper_pos = gripper_msg.joints[0].position
        except Exception as e:
            rospy.logerr("Grasp Verifier: {}".format(e))

    def _parse_image(self, image_msg):
        try:
            image_cv = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")  # The encoding is 16UC1
        except CvBridgeError as e:
            rospy.logerr("Grasp Verifier: {}".format(e))
            return
        image_cv = np.array(image_cv[145:270, 365:465])
        image_cv[image_cv > 20] = 0
        image_cv[np.isnan(image_cv)] = 0
        self.img_sum = np.sum(image_cv)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('grasp_verifier')
    verification = GraspVerifier()
    verification.run()
