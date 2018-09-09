#!/usr/bin/env python
# Simulate the /robot_driver so that interfaces to it can operate the same on
# the real robot and in simulation

from __future__ import print_function, division

from threading import Lock

import rospy

from fetch_driver_msgs.msg import RobotState
from power_msgs.msg import BreakerState
from std_srvs.srv import SetBool, SetBoolResponse  # FIXME: This is not the one used on the Fetch?


# This is the class that acts as the stub to the robot driver

class SimulatedRobotDriver(object):
    """
    In simulation, implement the minimum amount of logic necessary to correctly
    spoof the behaviour of the robot driver
    """

    def __init__(self):
        # Internal parameters for the functions of this driver
        self._publish_rate = 20  # Hz rate. TODO: Match the robot value

        # The state of the arm, gripper, and base breakers
        self._arm_breaker_state = BreakerState(
            name="arm_breaker",
            state=BreakerState.STATE_ENABLED
        )
        self._base_breaker_state = BreakerState(
            name="base_breaker",
            state=BreakerState.STATE_ENABLED
        )
        self._gripper_breaker_state = BreakerState(
            name="gripper_breaker",
            state=BreakerState.STATE_ENABLED
        )

        # The cached state of the robot
        self._robot_state = RobotState(
            ready=True,
            breakers=[self._arm_breaker_state, self._base_breaker_state, self._gripper_breaker_state]
        )
        self._robot_state_lock = Lock()

        # The services to set and reset the breakers
        self._arm_breaker_service = rospy.Service("/arm_breaker", SetBool, self.set_arm_breaker)
        self._base_breaker_service = rospy.Service("/base_breaker", SetBool, self.set_base_breaker)
        self._gripper_breaker_service = rospy.Service("/gripper_breaker", SetBool, self.set_gripper_breaker)

        # Publishers
        self._robot_state_publisher = rospy.Publisher('/robot_state', RobotState, queue_size=1)

    def _calculate_robot_state(self):
        # Make sure to acquire the lock to the robot state before calling this
        # function. TODO: Make sure this matches the robot
        self._robot_state.ready = (
            self._arm_breaker_state.state == BreakerState.STATE_ENABLED
            and self._base_breaker_state.state == BreakerState.STATE_ENABLED
            and self._gripper_breaker_state.state == BreakerState.STATE_ENABLED
        )
        self._robot_state.runstopped = (
            self._arm_breaker_state.state == BreakerState.STATE_DISABLED
            or self._base_breaker_state.state == BreakerState.STATE_DISABLED
            or self._gripper_breaker_state.state == BreakerState.STATE_DISABLED
        )
        self._robot_state.faulted = not (
            self._robot_state.ready
            or self._robot_state.runstopped
        )

    def set_arm_breaker(self, req):
        with self._robot_state_lock:
            self._arm_breaker_state.state = BreakerState.STATE_ENABLED if req.data else BreakerState.STATE_DISABLED
            self._calculate_robot_state()
        return SetBoolResponse(success=True)

    def set_base_breaker(self, req):
        with self._robot_state_lock:
            self._base_breaker_state.state = BreakerState.STATE_ENABLED if req.data else BreakerState.STATE_DISABLED
            self._calculate_robot_state()
        return SetBoolResponse(success=True)

    def set_gripper_breaker(self, req):
        with self._robot_state_lock:
            self._gripper_breaker_state.state =BreakerState.STATE_ENABLED if req.data else BreakerState.STATE_DISABLED
            self._calculate_robot_state()
        return SetBoolResponse(success=True)

    def spin(self):
        rate = rospy.Rate(self._publish_rate)
        while not rospy.is_shutdown():
            with self._robot_state_lock:
                self._robot_state.header.stamp = rospy.Time.now()
                self._robot_state.header.seq += 1
                self._robot_state_publisher.publish(self._robot_state)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('robot_driver')
    driver = SimulatedRobotDriver()
    driver.spin()
