#!/usr/bin/env python
# The compliant-mode action in a task plan

import rospy

from task_executor.abstract_step import AbstractStep

from power_msgs.msg import BreakerState
from power_msgs.srv import BreakerCommand


class CompliantModeAction(AbstractStep):

    ARM_BREAKER_SERVICE_NAME = "/arm_breaker"
    BASE_BREAKER_SERVICE_NAME = "/base_breaker"
    GRIPPER_BREAKER_SERVICE_NAME = "/gripper_breaker"

    def init(self, name):
        self.name = name

        # Service to communicate with the breakers
        self._arm_breaker_srv = rospy.ServiceProxy(
            CompliantModeAction.ARM_BREAKER_SERVICE_NAME,
            BreakerCommand
        )
        self._base_breaker_srv = rospy.ServiceProxy(
            CompliantModeAction.BASE_BREAKER_SERVICE_NAME,
            BreakerCommand
        )
        self._gripper_breaker_srv = rospy.ServiceProxy(
            CompliantModeAction.GRIPPER_BREAKER_SERVICE_NAME,
            BreakerCommand
        )

        # Initialize our connections to the robot driver
        rospy.loginfo("Connecting to robot driver...")
        self._arm_breaker_srv.wait_for_service()
        self._base_breaker_srv.wait_for_service()
        self._gripper_breaker_srv.wait_for_service()
        rospy.loginfo("...robot driver connected")

    def run(self, enable):
        rospy.loginfo("Action {}: {}".format(self.name, "Enable" if enable else "Disable"))

        # For each of the breakers, set the compliance accordingly and test the
        # returned setting
        if enable:
            # Send a disable message to the base and gripper
            self._validate_response(
                self._base_breaker_srv(enable=False),
                BreakerState.STATE_DISABLED
            )
            yield self.set_running()
            self._validate_response(
                self._gripper_breaker_srv(enable=False),
                BreakerState.STATE_DISABLED
            )
            yield self.set_running()

            # Disable and reenable the arm so that we have gravity comp
            self._validate_response(
                self._arm_breaker_srv(enable=False),
                BreakerState.STATE_DISABLED
            )
            yield self.set_running()
            self._validate_response(
                self._arm_breaker_srv(enable=True),
                BreakerState.STATE_ENABLED
            )

        else:
            # Reenable the base and the gripper
            self._validate_response(
                self._base_breaker_srv(enable=True),
                BreakerState.STATE_ENABLED
            )
            yield self.set_running()
            self._validate_response(
                self._gripper_breaker_srv(enable=True),
                BreakerState.STATE_ENABLED
            )

        yield self.set_succeeded()

    def stop(self):
        # This action cannot be stopped
        pass

    def _validate_response(self, response, expected_value):
        assert response.status.state == expected_value
