#!/usr/bin/env python
# Verify the grasp and return true if grasp is present

import rospy

from task_executor.abstract_step import AbstractStep

from std_srvs.srv import Trigger


# The action class

class VerifyGraspAction(AbstractStep):
    """
    Return true if there is something in the robot's grasp. Else, return false
    """

    VERIFICATION_SERVICE_NAME = "/grasp_verifier/verify"

    def init(self, name):
        self.name = name

        # The grasp verification service
        self._verification_srv = rospy.ServiceProxy(
            VerifyGraspAction.VERIFICATION_SERVICE_NAME,
            Trigger
        )

        # Wait for the service
        rospy.loginfo("Connecting to the verification service...")
        self._verification_srv.wait_for_service()
        rospy.loginfo("...verification service connected")

    def run(self):
        rospy.loginfo("Action {}: Checking for candy".format(self.name))
        candy_picked = self._verification_srv().success
        self.notify_service_called(VerifyGraspAction.VERIFICATION_SERVICE_NAME)
        yield self.set_succeeded(candy_picked=candy_picked)

    def stop(self):
        # Can't cancel this action
        pass
