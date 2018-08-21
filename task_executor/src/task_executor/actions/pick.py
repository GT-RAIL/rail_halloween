import rospy

from fetch_gazebo_demo import GraspingClient

from task_executor.abstract_action import AbstractAction


class PickAction(AbstractAction):

    def __init__(self):
        self._pick_client = None

    def init(self, location, objects, scene):
        self._pick_client = GraspingClient(scene)

    def run(self, cube, grasps):
        rospy.loginfo("Picking cube: {}".format(cube.name))
        yield {}

        success, result = self._pick_client.pick(cube, grasps)
        if success:
            yield {'pick_result': result}