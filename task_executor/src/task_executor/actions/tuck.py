import rospy

from fetch_gazebo_demo import GraspingClient

from task_executor.abstract_action import AbstractAction


class TuckAction(AbstractAction):

    def __init__(self):
        self._tuck_client = None

    def init(self, locations, objects):
        self._tuck_client = GraspingClient()

    def run(self):
        rospy.loginfo("Tucking arm")
        yield {}
        self._tuck_client.tuck()
