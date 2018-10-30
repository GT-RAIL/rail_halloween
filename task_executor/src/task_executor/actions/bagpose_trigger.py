#!/usr/bin/env python
# The trigger based on detecting a person from the pose detector

import rospy

from task_executor.abstract_step import AbstractStep

from bagposefromperson.srv import GetBagPose, GetBagPoseRequest

from .joystick_trigger import JoystickTriggerAction


# The class definition

class BagposeTriggerAction(AbstractStep):
    """
    Trigger based on person pose estimation
    """

    BAGPOSE_SERVICE_NAME = "/get_bag_pose_from_person"

    def init(self, name):
        self.name = name

        # Initialize the bagpose verification service
        self._bagpose_srv = rospy.ServiceProxy(
            BagposeTriggerAction.BAGPOSE_SERVICE_NAME,
            GetBagPose
        )
        rospy.loginfo("Connecting to bag pose service...")
        self._connect_to_bagpose_timer = rospy.Timer(rospy.Duration(0.5), self._connect_to_bagpose, oneshot=True)

        # Initialize the joystick trigger
        self._joystick_trigger = JoystickTriggerAction()
        self._joystick_trigger.init('joystick_trigger_bagpose')

    def run(self, timeout=0.0):
        pass

    def stop(self):
        pass

    def _connect_to_bagpose(self, evt):
        if self._bagpose_srv.wait_for_service(0.1):
            rospy.loginfo("...bag pose service connected")
            self._connect_to_bagpose_timer = None
            return

        # Schedule the next test of the bagpose timer
        self._connect_to_bagpose_timer = rospy.Timer(rospy.Duration(0.5), self._connect_to_bagpose, oneshot=True)
