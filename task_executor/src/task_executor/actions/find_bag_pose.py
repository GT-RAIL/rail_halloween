#!/usr/bin/env python
# The trigger based on detecting a person from the pose detector

import numpy as np

import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs

from task_executor.abstract_step import AbstractStep

from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from bagposefromperson.msg import GetBagPoseAction, GetBagPoseGoal
from task_executor.srv import GetArmGripperPose

from .joystick_trigger import JoystickTriggerAction


# Helpers

NANPOSE = PoseStamped()
NANPOSE.pose.position = Point(*([float('nan')] * 3))


def pose_is_nanpose(pose):
    global NANPOSE
    return pose.header.frame_id == '' \
        and np.isnan(pose.pose.position.x) \
        and np.isnan(pose.pose.position.y) \
        and np.isnan(pose.pose.position.z) \
        and pose.pose.orientation == NANPOSE.pose.orientation


def get_xy_pose_distance(poseA):
    """Assume base_link framed PoseStamped messages"""
    return np.sqrt(poseA.pose.position.x ** 2 + poseA.pose.position.y ** 2)


def pose_to_dict(pose):
    return {
        'frame': pose.header.frame_id,
        'position': {
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'z': pose.pose.position.z,
        },
        'orientation': {
            'x': pose.pose.orientation.x,
            'y': pose.pose.orientation.y,
            'z': pose.pose.orientation.z,
            'w': pose.pose.orientation.w,
        }
    }


# The class definition

class FindBagPoseAction(AbstractStep):
    """
    Trigger based on person pose estimation
    """

    BAGPOSE_SERVER = "/get_bag_pose_from_person"
    BAGPOSE_DISTANCE_THRESHOLD = 1.5

    ARM_GRIPPER_POSES_SERVICE_NAME = "/database/arm_gripper_pose"
    DEFAULT_BAG_POSE_NAME = "default_drop"

    BASE_FRAME = "base_link"
    BAG_POSE_OFFSET = [-0.05, 0, 0.1]  # 5 cm back, 10 cm up (from base_link)

    def init(self, name):
        self.name = name
        self._tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        # Initialize the bagpose verification service
        self._bagpose_client = actionlib.SimpleActionClient(
            FindBagPoseAction.BAGPOSE_SERVER,
            GetBagPoseAction
        )
        rospy.loginfo("Connecting to bag pose service...")
        self._connect_to_bagpose_timer = rospy.Timer(rospy.Duration(0.5), self._connect_to_bagpose, oneshot=True)

        # Initialize the database service
        get_arm_gripper_pose_srv = rospy.ServiceProxy(
            FindBagPoseAction.ARM_GRIPPER_POSES_SERVICE_NAME,
            GetArmGripperPose
        )
        rospy.loginfo("Connecting to database services...")
        get_arm_gripper_pose_srv.wait_for_service()
        rospy.loginfo("...database services connected")

        # Get the default pose and initialize the pose visualizer
        self._default_pose = get_arm_gripper_pose_srv(FindBagPoseAction.DEFAULT_BAG_POSE_NAME).pose
        self.notify_service_called(FindBagPoseAction.ARM_GRIPPER_POSES_SERVICE_NAME)
        self._pose_debug_topic = rospy.Publisher(
            FindBagPoseAction.BAGPOSE_SERVER + '/pose',
            PoseStamped,
            queue_size=1
        )

        # Initialize the joystick trigger
        self._joystick_trigger = JoystickTriggerAction()
        self._joystick_trigger.init('joystick_trigger_bagpose')

    def run(self, timeout=0.0, detection_mode=GetBagPoseGoal.D_MODE_STD):
        # Timeout of 0 implies infinite. The timeout only applies to the pose
        # detector. It does not apply to the joystick trigger based 'detection'
        rospy.loginfo("Action {}: Waiting for a trigger on pose detector within time {}s"
                      .format(self.name, timeout))

        # Get a response from the client if the client has been connected
        bag_pose = NANPOSE
        if self._connect_to_bagpose_timer is None:
            start_time = rospy.Time.now()

            # Send the goal
            goal = GetBagPoseGoal(detection_mode)
            self._bagpose_client.send_goal(goal)
            self.notify_action_send_goal(FindBagPoseAction.BAGPOSE_SERVER, goal)

            # Wait for a result
            while self._bagpose_client.get_state() in AbstractStep.RUNNING_GOAL_STATES:

                # Check the time
                if timeout > 0 and rospy.Time.now() - start_time > rospy.Duration(timeout):
                    self._bagpose_client.cancel_goal()

                yield self.set_running()

            # Get the status and the result
            status = self._bagpose_client.get_state()
            self._bagpose_client.wait_for_result()
            result = self._bagpose_client.get_result()
            self.notify_action_recv_result(FindBagPoseAction.BAGPOSE_SERVER, status, result)

            # If the server was preempted, then stop. Else change the frame,
            # update the location and publish on the bag pose topic
            bag_pose = result.bag_pose
            if status == GoalStatus.PREEMPTED and timeout == 0:
                yield self.set_preempted(
                    action=self.name,
                    goal=goal,
                    status=status,
                    result=result
                )
                raise StopIteration()
            elif not pose_is_nanpose(bag_pose):
                try:
                    # First transform the frame
                    transform = self._tf_buffer.lookup_transform(
                        FindBagPoseAction.BASE_FRAME,
                        bag_pose.header.frame_id,
                        rospy.Time(0),
                        rospy.Duration(1.0)
                    )
                    bag_pose = tf2_geometry_msgs.do_transform_pose(bag_pose, transform)

                    # Then modify it a bit
                    bag_pose.pose.position.x += FindBagPoseAction.BAG_POSE_OFFSET[0]
                    bag_pose.pose.position.y += FindBagPoseAction.BAG_POSE_OFFSET[1]
                    bag_pose.pose.position.z += FindBagPoseAction.BAG_POSE_OFFSET[2]
                    bag_pose.pose.orientation = Quaternion(*[0., 0., 0., 1.])

                    # Finally output it as debug
                    self._pose_debug_topic.publish(bag_pose)
                    self.notify_topic_published(
                        FindBagPoseAction.BAGPOSE_SERVER + '/pose',
                        bag_pose
                    )
                except (tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    rospy.logerr("Exception looking up tf transform!")
                    bag_pose = NANPOSE

            yield self.set_running()

        # Wait for a confirmation from the joystick before proceeding
        for variables in self._joystick_trigger.run():
            yield self.set_running()

        # If joystick is preempted or aborted, then so are we
        if self._joystick_trigger.is_preempted():
            yield self.set_preempted(**variables)
            raise StopIteration()
        elif self._joystick_trigger.is_aborted():
            yield self.set_aborted(**variables)
            raise StopIteration()

        # If bag_pose is invalid, then continue to default
        if pose_is_nanpose(bag_pose) \
                or get_xy_pose_distance(bag_pose) > FindBagPoseAction.BAGPOSE_DISTANCE_THRESHOLD \
                or not variables['choice']:
            yield self.set_succeeded(bag_pose=pose_to_dict(self._default_pose))
        else:
            yield self.set_succeeded(bag_pose=pose_to_dict(bag_pose))

    def stop(self):
        # If the bag pose client exists, cancel its goal
        if self._connect_to_bagpose_timer is None:
            self._bagpose_client.cancel_goal()

        # Also cancel the joystick goal
        self._joystick_trigger.stop()

    def _connect_to_bagpose(self, evt):
        if self._bagpose_client.wait_for_server(rospy.Duration(0.1)):
            rospy.loginfo("...bag pose service connected")
            self._connect_to_bagpose_timer = None
            return

        # Schedule the next test of the bagpose timer
        self._connect_to_bagpose_timer = rospy.Timer(rospy.Duration(0.5), self._connect_to_bagpose, oneshot=True)
