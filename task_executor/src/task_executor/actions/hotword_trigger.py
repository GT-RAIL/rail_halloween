#!/usr/bin/env python
# Trigger based on hotword detection

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from hotword_detector.msg import DetectHotWordAction, DetectHotWordGoal


# The class definition

class HotwordTriggerAction(AbstractStep):

    HOTWORD_ACTION_SERVER = '/detect_hotword'

    def init(self, name):
        self.name = name
        self._hotword_client = rospy.ServiceProxy(
            HotwordTriggerAction.HOTWORD_ACTION_SERVER,
            DetectHotWordAction
        )

        rospy.loginfo("Connecting to hotword detector...")
        self._hotword_client.wait_for_server()
        rospy.loginfo("...hotword detector connected")

    def run(self, timeout=0.0):
        rospy.loginfo("Action {}: Waiting for a hotword trigger within time {}s"
                      .format(self.name, timeout))

        # Create and send the goal. Wait until a result
        goal = DetectHotWordGoal(timeout=timeout)
        self._hotword_client.send_goal(goal)
        self.notify_action_send_goal(HotwordTriggerAction.HOTWORD_ACTION_SERVER, goal)
        while self._hotword_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:
            yield self.set_running()

        # Get the result and yield based on the result from the detector
        status = self._hotword_client.get_state()
        self._hotword_client.wait_for_result()
        result = self._hotword_client.get_result()
        self.notify_action_recv_result(HotwordTriggerAction.HOTWORD_ACTION_SERVER, status, result)

        if status == GoalStatus.SUCCEEDED:
            yield self.set_succeeded(choice=result.reply)
        elif status == GoalStatus.PREEMPTED:
            yield self.set_preempted(
                action=self.name,
                status=status,
                goal=goal,
                result=result
            )
        else:
            yield self.set_aborted(
                action=self.name,
                status=status,
                goal=goal,
                result=result
            )

    def stop(self):
        self._hotword_client.cancel_goal()
        self.notify_action_cancel(HotwordTriggerAction.HOTWORD_ACTION_SERVER)
