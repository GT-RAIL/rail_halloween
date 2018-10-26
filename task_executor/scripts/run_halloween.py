#!/usr/bin/env python
# Runs the Halloween task in a loop

from __future__ import print_function, division

import numpy as np

from threading import Thread

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from task_executor.msg import ExecuteAction, ExecuteGoal
from std_srvs.srv import Trigger, TriggerResponse


# Helpers

def goal_status_from_code(status):
    mapping = {
        GoalStatus.SUCCEEDED: "SUCCEEDED",
        GoalStatus.PREEMPTED: "PREEMPTED",
        GoalStatus.ABORTED: "ABORTED",
    }
    return mapping.get(status, status)


# The class for the halloween executor

class Halloween(object):
    """
    Runs the halloween task in a loop until it is shutdown
    """

    TASK_EXECUTOR_SERVER = '/task_executor'
    MAIN_TASK_NAME = 'main'
    SETUP_TASK_NAME = 'setup'

    JOY_TOPIC = '/joy'
    ESTOP_BUTTONS = [9, 11]
    BUTTON_PRESS_TIMEOUT = rospy.Duration(0.5)

    START_SERVICE = '~start'
    STOP_SERVICE = '~stop'
    SETUP_SERVICE = '~setup'
    TRIGGER_SERVICE = '~trigger'

    def __init__(self):
        # The background spinner
        self._background_spinner = None
        self._spin = False
        self._trigger = False  # TODO: Use a wake-word or something smarter

        # The joystick subscriber
        self._joy_sub = rospy.Subscriber(Halloween.JOY_TOPIC, Joy, self._on_joy)
        self._is_estop_pressed = lambda msg: (
            msg.buttons[Halloween.ESTOP_BUTTONS[0]] > 0 and msg.buttons[Halloween.ESTOP_BUTTONS[1]] > 0
        )
        self._last_pressed_time = rospy.Time(0)  # Debounce helper

        # The action client to the task Halloween
        self.client = actionlib.SimpleActionClient(Halloween.TASK_EXECUTOR_SERVER, ExecuteAction)
        rospy.loginfo("Connecting to task_executor...")
        self.client.wait_for_server()
        rospy.loginfo("...task_executor connected")

        # Create the services
        self._start_service = rospy.Service(Halloween.START_SERVICE, Trigger, self._on_start)
        self._stop_service = rospy.Service(Halloween.STOP_SERVICE, Trigger, self._on_stop)
        self._setup_service = rospy.Service(Halloween.SETUP_SERVICE, Trigger, self._on_setup)
        self._trigger_service = rospy.Service(Halloween.TRIGGER_SERVICE, Trigger, self._on_trigger)

        rospy.loginfo("Halloween ready")

    def spin(self):
        """Continuously executes the task in the background"""
        while self._spin:
            # Wait for the trigger to start. TODO: Make this better
            self._trigger = False
            while not self._trigger and self._spin:
                rospy.sleep(0.5)

            # Exit if we should stop
            if not self._spin:
                break

            # Create the execution goal for the main task
            rospy.loginfo("Starting: {}".format(Halloween.MAIN_TASK_NAME))
            goal = ExecuteGoal(name=Halloween.MAIN_TASK_NAME)
            self.client.send_goal(goal)
            while not self.client.wait_for_result(rospy.Duration(0.5)):
                if not self._spin:
                    self.client.cancel_all_goals()

            # Get the status
            rospy.loginfo("Result: {}".format(goal_status_from_code(self.client.get_state())))

    def _on_start(self, req):
        self._spin = True
        self._background_spinner = Thread(target=self.spin)
        self._background_spinner.start()
        return TriggerResponse(success=True)

    def _on_stop(self, req):
        self._spin = False
        return TriggerResponse(success=True)

    def _on_setup(self, req):
        rospy.loginfo("Starting: {}".format(Halloween.SETUP_TASK_NAME))
        goal = ExecuteGoal(name=Halloween.SETUP_TASK_NAME)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        rospy.loginfo("Result: {}".format(goal_status_from_code(self.client.get_state())))

        resp = TriggerResponse(success=self.client.get_result().success)
        return resp

    def _on_trigger(self, req):
        self._trigger = True
        return TriggerResponse(success=True)

    def _on_joy(self, joy_msg):
        if self._last_pressed_time + Halloween.BUTTON_PRESS_TIMEOUT >= rospy.Time.now():
            # Debounce
            if self._is_estop_pressed(joy_msg):
                self._last_pressed_time = rospy.Time.now()
            return
        elif self._last_pressed_time > rospy.Time(0) and \
                self._last_pressed_time + Halloween.BUTTON_PRESS_TIMEOUT > rospy.Time.now():
            # Reset the debounce
            self._last_pressed_time = rospy.Time(0)
            return

        # We only use this for sending cancel messages
        if self._is_estop_pressed(joy_msg):
            self._last_pressed_time = rospy.Time.now()
            self.client.cancel_all_goals()


# The main
if __name__ == '__main__':
    rospy.init_node('halloween')
    executor = Halloween()
    rospy.spin()
