#!/usr/bin/env python
# Runs the Halloween task in a loop

from __future__ import print_function, division

import itertools

import numpy as np

from threading import Thread

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Joy
from task_executor.msg import ExecuteAction, ExecuteGoal
from std_srvs.srv import Trigger, TriggerResponse

from task_executor.actions import JoystickTriggerAction, HotwordTriggerAction


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
    IDLE_EXECUTOR_SERVER = '/idle_executor'
    MAIN_TASK_NAME = 'main'
    SETUP_TASK_NAME = 'setup'

    JOY_TOPIC = '/joy'
    ESTOP_BUTTONS = [9, 11]
    BUTTON_PRESS_TIMEOUT = rospy.Duration(0.5)

    START_SERVICE = '~start'
    STOP_SERVICE = '~stop'
    SETUP_SERVICE = '~setup'
    TRIGGER_SERVICE = '~trigger'

    IDLE_TASKS = ["idle1"]
    IDLE_WAIT_LOW = 20
    IDLE_WAIT_HIGH = 60

    def __init__(self):
        # The background spinner
        self._background_spinner = None
        self._spin = False
        self._trigger = False  # The combined result of all triggers

        # The background idle tasks
        self._idle_timer = None
        self._idle_is_running = False

        # The trigger actions
        self._joystick_trigger = JoystickTriggerAction()
        self._joystick_trigger.init('joystick_trigger')

        self._hotword_trigger = HotwordTriggerAction()
        self._hotword_trigger.init('hotword_trigger')

        # The joystick subscriber
        self._joy_sub = rospy.Subscriber(Halloween.JOY_TOPIC, Joy, self._on_joy)
        self._is_estop_pressed = lambda msg: (
            msg.buttons[Halloween.ESTOP_BUTTONS[0]] > 0 and msg.buttons[Halloween.ESTOP_BUTTONS[1]] > 0
        )
        self._last_pressed_time = rospy.Time(0)  # Debounce helper

        # The action clients to the Halloween tasks
        self.task_client = actionlib.SimpleActionClient(Halloween.TASK_EXECUTOR_SERVER, ExecuteAction)
        self.idle_client = actionlib.SimpleActionClient(Halloween.IDLE_EXECUTOR_SERVER, ExecuteAction)

        rospy.loginfo("Connecting to task_executor...")
        self.task_client.wait_for_server()
        rospy.loginfo("...task_executor connected")

        rospy.loginfo("Connecting to idle_executor...")
        self.idle_client.wait_for_server()
        rospy.loginfo("...idle_executor connected")

        # Create the services
        self._start_service = rospy.Service(Halloween.START_SERVICE, Trigger, self._on_start)
        self._stop_service = rospy.Service(Halloween.STOP_SERVICE, Trigger, self._on_stop)
        self._setup_service = rospy.Service(Halloween.SETUP_SERVICE, Trigger, self._on_setup)
        self._trigger_service = rospy.Service(Halloween.TRIGGER_SERVICE, Trigger, self._on_trigger)

        rospy.loginfo("Halloween ready")

    def spin(self):
        """Continuously executes the task in the background"""
        status = GoalStatus.LOST
        while self._spin:
            # Wait for the trigger to start. TODO: Make this better
            self._trigger = False
            self._idle_timer = rospy.Timer(
                rospy.Duration(np.random.random_integers(Halloween.IDLE_WAIT_LOW,
                                                         Halloween.IDLE_WAIT_HIGH)),
                self._run_idle,
                oneshot=True
            )

            # Use the hotword trigger only if the previous run was not preempted
            # or aborted
            iterators_to_use = [self._joystick_trigger.run(),]
            if status not in [GoalStatus.PREEMPTED, GoalStatus.ABORTED]:
                iterators_to_use.append(self._hotword_trigger.run())

            for vars_tuple in itertools.izip_longest(*iterators_to_use):
                # Update the value of trigger
                self._trigger = self._trigger or np.any([x.has_key('choice') for x in vars_tuple])

                # Check to see if the thread is shutdown
                if not self._spin:
                    break

                # Check to see if one of the trigger methods has been called
                if self._trigger:
                    if self._joystick_trigger.is_running():
                        self._joystick_trigger.stop()
                    if self._hotword_trigger.is_running():
                        self._hotword_trigger.stop()

                # Otherwise, wait a bit
                rospy.sleep(0.5)

            # If the idle task is running, then wait for it to finish
            while self._idle_is_running:
                rospy.sleep(0.5)
            self._idle_timer.shutdown()
            self._idle_timer = None

            # Exit if we should stop
            if not self._spin:
                break

            # Create the execution goal for the main task
            rospy.loginfo("Starting: {}".format(Halloween.MAIN_TASK_NAME))
            goal = ExecuteGoal(name=Halloween.MAIN_TASK_NAME)
            self.task_client.send_goal(goal)
            while not self.task_client.wait_for_result(rospy.Duration(0.5)):
                if not self._spin:
                    self.task_client.cancel_all_goals()

            # Get the status
            status = self.task_client.get_state()
            rospy.loginfo("Result: {}".format(goal_status_from_code(status)))

    def _run_idle(self, evt):
        # Signal that you've started, and select a task
        self._idle_is_running = True
        idle_to_run = np.random.choice(Halloween.IDLE_TASKS)

        # Send the task to the server and wait
        rospy.loginfo("Running IDLE: {}".format(idle_to_run))
        goal = ExecuteGoal(name=idle_to_run)
        self.idle_client.send_goal(goal)
        while not self.idle_client.wait_for_result(rospy.Duration(0.5)):
            pass

        # Get the status
        rospy.loginfo("Result IDLE: {}".format(goal_status_from_code(self.idle_client.get_state())))

        # Notify that you're done and reset the timer for the next idle
        self._idle_is_running = False
        self._idle_timer = rospy.Timer(
            rospy.Duration(np.random.random_integers(Halloween.IDLE_WAIT_LOW,
                                                     Halloween.IDLE_WAIT_HIGH)),
            self._run_idle,
            oneshot=True
        )

    def _on_start(self, req):
        if not self._spin:
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
        self.task_client.send_goal(goal)
        self.task_client.wait_for_result()
        rospy.loginfo("Result: {}".format(goal_status_from_code(self.task_client.get_state())))

        resp = TriggerResponse(success=self.task_client.get_result().success)
        return resp

    def _on_trigger(self, req):
        if self._spin:
            self._trigger = True
            return TriggerResponse(success=True)
        else:
            return TriggerResponse(success=False)

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
            self.task_client.cancel_all_goals()


# The main
if __name__ == '__main__':
    rospy.init_node('halloween')
    executor = Halloween()
    rospy.spin()
