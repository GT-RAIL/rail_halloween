#!/usr/bin/env python
# The place action in a task plan

import rospy

from task_executor.abstract_action import AbstractAction

from std_srvs.srv import Empty

from .arm_pose import ArmPoseAction


class PlaceAction(AbstractAction):

    def init(self, **kwargs):
        self._drop_object_srv = rospy.ServiceProxy(
            "grasp_executor/drop_object",
            Empty
        )
        self._arm_pose = ArmPoseAction()
        self._drop_pose_name = "poses.drop"  # This is from the poses database

        # Set a stop flag
        self._stopped = False

        rospy.loginfo("Connecting to drop_service...")
        self._drop_object_srv.wait_for_service()
        rospy.loginfo("...drop_service connected")
        self._arm_pose.init(**kwargs)

    def run(self):
        rospy.loginfo("Placing object that are in hand")
        self._stopped = False

        # First move to the desired pose
        for variables in self._arm_pose.run(self._drop_pose_name):
            yield self.set_running(**variables)

        # Check to see if the arm pose failed
        if not self._arm_pose.is_succeeded():
            if self._arm_pose.is_preempted():
                yield self.set_preempted(**variables)
            else:
                yield self.set_aborted(**variables)
            raise StopIteration()

        # Then call the client to perform the grasps
        try:
            self._drop_object_srv()  # There is no feedback from this service...
            if self._stopped:
                yield self.set_preempted()
            else:
                yield self.set_succeeded()
        except Exception as e:  # On any exception, make sure we are set to aborted
            yield self.set_aborted(exception=e)


    def stop(self):
        # Propagate the stop signal and also set a personal stopped flag
        self._arm_pose.stop()
        self._stopped = True
