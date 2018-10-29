#! /usr/bin/env python

import rospy
import actionlib

import data_recorder.msg
from data_recorder.executor import TrajectoryExecutor

class PlaybackExecutor:
	STIR_FILENAME = 'stir.txt'
	HAT_REACH_FILENAME = 'stir.txt'  # Replace these with new trajectories
	HAT_MANEUVER_FILENAME = 'stir.txt'
	HAT_RETRACT_FILENAME = 'stir.txt'

	def __init__(self):
		rospy.init_node('primitive_playback_executor')

		self.stir_executor = TrajectoryExecutor(self.STIR_FILENAME)
		self.hat_reach_executor = TrajectoryExecutor(self.HAT_REACH_FILENAME)
		self.hat_maneuver_executor = TrajectoryExecutor(self.HAT_MANEUVER_FILENAME)
		self.hat_retract_executor = TrajectoryExecutor(self.HAT_RETRACT_FILENAME)

		self.playback_server = actionlib.SimpleActionServer('playback_primitive', data_recorder.msg.PlaybackAction, self.playback_cb, False)

		self.playback_server.register_preempt_callback(self.preempt_playback_cb)

		self.playback_server.start()

		rospy.spin()

	def playback_cb(self, goal):
		primitive_name = goal.primitive_name

		if primitive_name == 'stir':
			success = self.stir_executor.executeTrajectory()
		elif primitive_name == 'hat_reach':
			success = self.hat_reach_executor.executeTrajectory()
		elif primitive_name == 'hat_maneuver':
			success = self.hat_maneuver_executor.executeTrajectory()
		elif primitive_name == 'hat_reach':
			success = self.hat_retract_executor.executeTrajectory()
		else:
			rospy.logerr('Primitive Does not Exist')
			success = False

		result = data_recorder.msg.EmptyResult

		if success:
			self.playback_server.set_succeeded(result)
		else:
			self.playback_server.set_aborted(result, 'Playback Execution Failure!')

	def preempt_playback_cb(self):
		result = data_recorder.msg.EmptyResult
		self.stir_executor.preemptExecution()
		self.playback_server.set_preempted(result, 'Playback pre-empted by user')


if __name__ == '__main__':
	playback_executor = PlaybackExecutor()