#!/usr/bin/env python
# The find object action in a task plan

from __future__ import print_function, division

import rospy
import actionlib

from task_executor.abstract_step import AbstractStep

from fetch_grasp_suggestion.srv import SuggestGrasps, PairwiseRank


class FindGraspsAction(AbstractStep):

    def init(self):
        # The Grasp calculation interface
        self._suggest_grasps_srv = rospy.ServiceProxy(
            "suggester/suggest_grasps",
            SuggestGrasps
        )
        self._grasps_rank_srv = rospy.ServiceProxy(
            "suggester/pairwise_rank",
            PairwiseRank
        )

        # Set the max number of grasps to try. This can be a param lookup
        self._max_grasps = 10

        # Set a stop flag
        self._stopped = False

        # Wait for a connection to suggest_grasps
        rospy.loginfo("Connecting to suggest_grasps...")
        self._suggest_grasps_srv.wait_for_service()
        self._grasps_rank_srv.wait_for_service()
        rospy.loginfo("...suggest_grasps connected")

    def run(self, segmented_obj):
        rospy.loginfo("Calculating grasps on object at: {}".format(segmented_obj.center))
        self._stopped = False

        # Given the segmentation and the objects, now ask for grasps
        # On any exception, make sure that we are set to aborted
        try:
            grasps = self._suggest_grasps_srv(cloud=segmented_obj.point_cloud).grasp_list
            if len(grasps.poses) == 0:
                raise Exception("No grasps found")
            elif self._stopped:
                yield self.set_preempted()
            else:
                yield self.set_running()

            # Now that we have grasps, pairwise rank them
            grasps = self._grasps_rank_srv().grasp_list
            if len(grasps.poses) == 0:  # Something has gone horribly wrong
                raise Exception("Pairwise reranking of the grasps failed!")
            elif self._stopped:
                yield self.set_preempted()
            else:
                grasps.poses = grasps.poses[:self._max_grasps]
                yield self.set_succeeded(grasps=grasps)
        except Exception as e:
            yield self.set_aborted(exception=e.message)

    def stop(self):
        self._stopped = True
